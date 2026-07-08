import asyncio
import base64
import hashlib
import json
import os
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String

from palletizer_msgs.action import MoveXYZ
try:
    from palletizer_msgs.srv import (
        ClearFault,
        EnableAxis,
        GetDriverStatus,
        ReleaseStall,
        SetAxisLimits,
        SetGripper,
        SetZero,
    )
except ImportError:
    ClearFault = None
    EnableAxis = None
    GetDriverStatus = None
    ReleaseStall = None
    SetAxisLimits = None
    SetGripper = None
    SetZero = None

try:
    from palletizer_msgs.action import GoOrigin, HomeAxis
except ImportError:
    GoOrigin = None
    HomeAxis = None


AXIS_NAMES = ["X", "Y", "Z", "ALL", "A", "X1", "X2"]
PALLETIZER_CONNECTION_STALE_MS = int(os.environ.get("PALLETIZER_CONNECTION_STALE_MS", "1500"))
PALLETIZER_AVAILABILITY_HOLD_MS = int(os.environ.get("PALLETIZER_AVAILABILITY_HOLD_MS", "1200"))
ACTION_READY_TIMEOUT_SEC = float(os.environ.get("PALLETIZER_ACTION_READY_TIMEOUT_SEC", "0.02"))
SERVICE_READY_TIMEOUT_SEC = float(os.environ.get("PALLETIZER_SERVICE_READY_TIMEOUT_SEC", "0.02"))
PING_SERVICE_READY_TIMEOUT_SEC = float(os.environ.get("PALLETIZER_PING_SERVICE_READY_TIMEOUT_SEC", "0.02"))
UI_STATE_MIN_PERIOD_MS = int(os.environ.get("PALLETIZER_UI_STATE_MIN_PERIOD_MS", "33"))
MOTOR_COUNT = 5


def env_bool(name: str, default: str = "false") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS = env_bool("PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS")
PALLETIZER_ENABLE_AXIS_SERVICES = env_bool("PALLETIZER_ENABLE_AXIS_SERVICES")
PALLETIZER_ENABLE_EXTENDED_SERVICES = env_bool("PALLETIZER_ENABLE_EXTENDED_SERVICES")


def now_ms() -> int:
    return int(time.time() * 1000)


def parse_status_json(raw: str) -> Dict[str, Any]:
    if not raw:
        return {}
    try:
        value = json.loads(raw)
        return value if isinstance(value, dict) else {}
    except json.JSONDecodeError:
        return {"raw": raw}


def numeric_list(value: Any, length: int) -> list[float]:
    if not isinstance(value, list):
        return []
    values: list[float] = []
    for item in value[:length]:
        try:
            values.append(float(item))
        except (TypeError, ValueError):
            values.append(0.0)
    return values


def average(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def axis_values_from_motor_values(values: list[float]) -> list[float]:
    padded = (values + [0.0] * MOTOR_COUNT)[:MOTOR_COUNT]
    return [average([padded[0], padded[1]]), padded[2], padded[3]]


class PalletizerUiNode(Node):
    def __init__(self) -> None:
        super().__init__("palletizer_ui_backend")
        self._emit: Optional[Callable[[Dict[str, Any]], None]] = None
        self._lock = threading.Lock()
        self._last_seen_ms: Dict[str, int] = {}
        self._availability_seen_ms: Dict[str, int] = {}
        self._last_state_emit_ms = 0
        self._last_velocity_sample_ms: Optional[int] = None
        self._last_velocity_values: list[float] = []
        self._pending_actions: set[str] = set()
        self._active_actions: set[str] = set()
        self._state: Dict[str, Any] = {
            "stamp_ms": now_ms(),
            "ros": {
                "domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                "backend_node": "/palletizer_ui_backend",
                "palletizer_node": "/palletizer_controller",
            },
            "connections": {
                "palletizer_node": False,
                "ros_messages": False,
                "joint_states": False,
                "axis_position_mm": False,
                "motor_rpm": False,
                "status": False,
                "fault_state": False,
            },
            "last_seen_age_ms": {},
            "axis_position_mm": [0.0, 0.0, 0.0],
            "joint_states": {
                "name": [],
                "position": [],
                "velocity": [],
                "effort": [],
                "stamp": {"sec": 0, "nanosec": 0},
            },
            "motor_rpm": [0.0, 0.0, 0.0, 0.0, 0.0],
            "status_raw": "",
            "status": {},
            "fault_state": "",
            "last_action": None,
        }

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Float32MultiArray, "/palletizer/axis_position_mm", self._on_axis_position, 10)
        self.create_subscription(Float32MultiArray, "/palletizer/motor_rpm", self._on_motor_rpm, 10)
        self.create_subscription(String, "/palletizer/status", self._on_status, 10)
        self.create_subscription(String, "/palletizer/fault_state", self._on_fault_state, 10)
        self._emergency_pub = self.create_publisher(Bool, "/palletizer/emergency_stop", 10)
        self._command_pub = self.create_publisher(String, "/palletizer/command", 10)
        self._jog_delta_pub = self.create_publisher(Float32MultiArray, "/palletizer/jog_xyz_delta", 10)
        self._fast_move_pub = self.create_publisher(Float32MultiArray, "/palletizer/fast_move_xyz", 10)

        self._move_client = None
        self._home_client = None
        self._origin_client = None

        self._enable_axis_client = (
            self.create_client(EnableAxis, "/palletizer/enable_axis")
            if EnableAxis and PALLETIZER_ENABLE_AXIS_SERVICES else None
        )
        self._set_axis_limits_client = (
            self.create_client(SetAxisLimits, "/palletizer/set_axis_limits")
            if SetAxisLimits and PALLETIZER_ENABLE_AXIS_SERVICES else None
        )
        self._set_gripper_client = self.create_client(SetGripper, "/palletizer/set_gripper") if SetGripper else None
        self._set_zero_client = (
            self.create_client(SetZero, "/palletizer/set_zero")
            if SetZero and PALLETIZER_ENABLE_EXTENDED_SERVICES else None
        )
        self._clear_fault_client = (
            self.create_client(ClearFault, "/palletizer/clear_fault")
            if ClearFault and PALLETIZER_ENABLE_EXTENDED_SERVICES else None
        )
        self._release_stall_client = (
            self.create_client(ReleaseStall, "/palletizer/release_stall")
            if ReleaseStall and PALLETIZER_ENABLE_EXTENDED_SERVICES else None
        )
        self._get_driver_status_client = (
            self.create_client(GetDriverStatus, "/palletizer/get_driver_status")
            if GetDriverStatus and PALLETIZER_ENABLE_EXTENDED_SERVICES else None
        )

        self.create_timer(1.0, self._publish_backend_status)

    def set_emitter(self, emit: Callable[[Dict[str, Any]], None]) -> None:
        self._emit = emit

    def _connection_snapshot(self) -> tuple[Dict[str, bool], Dict[str, int]]:
        now = now_ms()
        with self._lock:
            last_seen = dict(self._last_seen_ms)

        tracked_topics = ["joint_states", "axis_position_mm", "motor_rpm", "status", "fault_state"]
        ages = {key: now - last_seen[key] for key in tracked_topics if key in last_seen}
        fresh = {
            key: key in ages and ages[key] <= PALLETIZER_CONNECTION_STALE_MS
            for key in tracked_topics
        }
        telemetry_messages = fresh["joint_states"] or fresh["axis_position_mm"] or fresh["motor_rpm"] or fresh["status"]
        motor_state_messages = fresh["status"] and (fresh["joint_states"] or fresh["motor_rpm"])

        try:
            node_names = set(self.get_node_names())
        except Exception:
            node_names = set()
        node_seen = "palletizer_controller" in node_names or "/palletizer_controller" in node_names

        # A graph node can remain visible briefly after USB/micro-ROS disconnects.
        # Treat the ESP32-S3 as physically connected only while fresh telemetry
        # from the firmware is still arriving.
        esp32_physical = bool(telemetry_messages)
        ros_communication = bool(telemetry_messages)
        connections = {
            "esp32_physical": esp32_physical,
            "ros_communication": ros_communication,
            "palletizer_graph_node": node_seen,
            "motor_state_messages": motor_state_messages,
            # Backward-compatible aliases used by older frontend panels.
            "palletizer_node": node_seen,
            "ros_messages": ros_communication,
            **fresh,
        }
        return connections, ages

    def snapshot(self) -> Dict[str, Any]:
        connections, ages = self._connection_snapshot()
        with self._lock:
            state = json.loads(json.dumps(self._state))
        state["connections"] = connections
        state["last_seen_age_ms"] = ages
        state["availability"] = self._availability_snapshot()
        return state

    def handle_command(self, message: Dict[str, Any]) -> None:
        command_type = str(message.get("type", ""))
        if command_type == "refresh":
            self._send({"type": "state", "state": self.snapshot()})
        elif command_type == "emergency_stop":
            self._publish_emergency(bool(message.get("data", True)))
        elif command_type == "move_xyz":
            self._send_move_goal(message.get("goal", {}))
        elif command_type == "jog_xyz":
            self._send_jog_command(message.get("goal", {}))
        elif command_type == "jog_xyz_delta":
            self._send_jog_delta_command(message.get("goal", {}))
        elif command_type == "home_axis":
            self._send_home_goal(message.get("goal", {}))
        elif command_type == "go_origin":
            self._send_origin_goal(message.get("goal", {}))
        elif command_type == "enable_axis":
            self._call_enable_axis(message.get("request", {}))
        elif command_type == "set_zero":
            self._call_set_zero(message.get("request", {}))
        elif command_type == "set_axis_limits":
            self._call_set_axis_limits(message.get("request", {}))
        elif command_type == "clear_fault":
            self._call_empty_service("clear_fault", self._clear_fault_client, ClearFault.Request() if ClearFault else None)
        elif command_type == "release_stall":
            self._call_release_stall(message.get("request", {}))
        elif command_type == "get_driver_status":
            self._call_empty_service("get_driver_status", self._get_driver_status_client, GetDriverStatus.Request() if GetDriverStatus else None)
        elif command_type == "raw_command":
            self._publish_command_text(str(message.get("command", "")))
        elif command_type == "set_aux_servo":
            self._set_aux_servo(message.get("request", {}))
        elif command_type == "set_gripper":
            self._call_set_gripper(message.get("request", {}))
        elif command_type == "ping":
            self._handle_ping()
        else:
            self._send({"type": "error", "message": f"unknown command: {command_type}"})

    def _ros_node_seen(self) -> bool:
        try:
            node_names = set(self.get_node_names())
        except Exception:
            return False
        return "palletizer_controller" in node_names or "/palletizer_controller" in node_names

    def _ping_report(self, ping_id: int, driver_status: Optional[Dict[str, Any]] = None, error: str = "") -> Dict[str, Any]:
        connections, ages = self._connection_snapshot()
        availability = self._availability_snapshot()
        topic_keys = ["status", "joint_states", "motor_rpm", "axis_position_mm"]
        topics_ok = all(bool(connections.get(key)) for key in topic_keys)
        services_ok = bool(availability.get("get_driver_status"))
        driver_ok = bool(driver_status and driver_status.get("success"))
        pending_driver_status = services_ok and driver_status is None and not error
        esp32_ok = bool(connections.get("esp32_physical")) and (driver_ok or pending_driver_status or topics_ok)
        return {
            "id": ping_id,
            "stamp_ms": now_ms(),
            "ok": bool(connections.get("ros_communication") and esp32_ok),
            "esp32_ok": esp32_ok,
            "ros_ok": bool(connections.get("ros_communication")),
            "graph_node_ok": self._ros_node_seen(),
            "topics_ok": topics_ok,
            "driver_status_ok": driver_ok,
            "driver_status_pending": pending_driver_status,
            "driver_status_error": error,
            "command_topic_subscribers": self._command_topic_subscribers(),
            "connections": connections,
            "last_seen_age_ms": ages,
            "availability": availability,
            "driver_status": driver_status,
            "message": self._ping_message(connections, topics_ok, services_ok, driver_ok, pending_driver_status, error),
        }

    def _ping_message(self, connections: Dict[str, bool], topics_ok: bool, services_ok: bool, driver_ok: bool, pending_driver_status: bool, error: str) -> str:
        if error:
            return f"get_driver_status fallo: {error}"
        if driver_ok:
            return "ESP32 respondio get_driver_status"
        if pending_driver_status:
            return "esperando respuesta get_driver_status"
        if topics_ok:
            return "telemetria fresca desde ESP32"
        if connections.get("ros_communication"):
            return "ROS recibe telemetria parcial"
        return "sin telemetria fresca desde ESP32"

    def _handle_ping(self) -> None:
        ping_id = now_ms()
        report = self._ping_report(ping_id)
        self._send({"type": "ping_result", "ping": report, "state": self.snapshot()})

        if GetDriverStatus is None or not self._service_ready(self._get_driver_status_client, PING_SERVICE_READY_TIMEOUT_SEC):
            return
        future = self._get_driver_status_client.call_async(GetDriverStatus.Request())
        future.add_done_callback(lambda f: self._on_ping_driver_status(ping_id, f))

    def _on_ping_driver_status(self, ping_id: int, future: Any) -> None:
        try:
            response = future.result()
            driver_status = self._service_response_dict(response)
            report = self._ping_report(ping_id, driver_status=driver_status)
        except Exception as exc:
            report = self._ping_report(ping_id, error=str(exc))
        self._send({"type": "ping_result", "ping": report, "state": self.snapshot()})

    def _set_connection(self, key: str) -> None:
        now = now_ms()
        self._last_seen_ms[key] = now
        self._state["connections"][key] = True
        self._state["stamp_ms"] = now

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._set_connection("joint_states")
            self._state["joint_states"] = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
            }
        self._send_state()

    def _on_axis_position(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._set_connection("axis_position_mm")
            values = list(msg.data)
            self._state["axis_position_mm"] = (values + [0.0, 0.0, 0.0])[:3]
        self._send_state()

    def _on_motor_rpm(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._set_connection("motor_rpm")
            values = list(msg.data)
            self._state["motor_rpm"] = (values + [0.0, 0.0, 0.0, 0.0, 0.0])[:5]
        self._send_state()

    def _on_status(self, msg: String) -> None:
        stamp_ms = now_ms()
        parsed = parse_status_json(msg.data)
        with self._lock:
            self._add_derived_motion_status(parsed, stamp_ms)
            self._set_connection("status")
            self._state["status_raw"] = msg.data
            self._state["status"] = parsed
        self._send_state()

    def _add_derived_motion_status(self, status: Dict[str, Any], stamp_ms: int) -> None:
        velocity = numeric_list(status.get("vel_mm_s"), MOTOR_COUNT)
        if len(velocity) < MOTOR_COUNT:
            return

        accel = [0.0] * MOTOR_COUNT
        if self._last_velocity_sample_ms is not None and len(self._last_velocity_values) >= MOTOR_COUNT:
            dt_s = max((stamp_ms - self._last_velocity_sample_ms) / 1000.0, 0.001)
            accel = [
                (velocity[index] - self._last_velocity_values[index]) / dt_s
                for index in range(MOTOR_COUNT)
            ]

        self._last_velocity_sample_ms = stamp_ms
        self._last_velocity_values = velocity
        status["axis_velocity_mm_s"] = axis_values_from_motor_values(velocity)
        status["derived_accel"] = accel
        status["axis_accel_mm_s2"] = axis_values_from_motor_values(accel)
        status["derived_accel_source"] = "/palletizer/status vel_mm_s"
        status["derived_accel_units"] = ["mm/s2", "mm/s2", "mm/s2", "mm/s2", "deg/s2"]

    def _on_fault_state(self, msg: String) -> None:
        with self._lock:
            self._set_connection("fault_state")
            self._state["fault_state"] = msg.data
        self._send_state()

    def _publish_backend_status(self) -> None:
        self._send({"type": "backend_status", "state": self.snapshot()})

    def _publish_emergency(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._emergency_pub.publish(msg)
        self._send({"type": "command_ack", "command": "emergency_stop", "data": active})

    def _axis_name(self, axis: int) -> str:
        return AXIS_NAMES[axis] if 0 <= axis < len(AXIS_NAMES) else "UNKNOWN"

    def _command_topic_subscribers(self) -> int:
        return self.count_subscribers("/palletizer/command")

    def _command_topic_ready(self) -> bool:
        return self._command_topic_subscribers() > 0

    def _palletizer_action_servers(self) -> set[str]:
        try:
            return {
                name
                for name, _types in get_action_server_names_and_types_by_node(
                    self,
                    "palletizer_controller",
                    "/",
                )
            }
        except Exception:
            return set()

    def _action_server_available(self, action_name: str) -> bool:
        return action_name in self._palletizer_action_servers()

    def _ensure_move_client(self) -> Any:
        if self._move_client is None:
            self._move_client = ActionClient(self, MoveXYZ, "/palletizer/move_xyz")
        return self._move_client

    def _ensure_home_client(self) -> Any:
        if self._home_client is None and HomeAxis and PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS:
            self._home_client = ActionClient(self, HomeAxis, "/palletizer/home_axis")
        return self._home_client

    def _ensure_origin_client(self) -> Any:
        if self._origin_client is None and GoOrigin and PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS:
            self._origin_client = ActionClient(self, GoOrigin, "/palletizer/go_origin")
        return self._origin_client

    def _fast_move_subscribers(self) -> int:
        return self.count_subscribers("/palletizer/fast_move_xyz")

    def _fast_move_ready(self) -> bool:
        return self._fast_move_subscribers() > 0

    def _jog_delta_subscribers(self) -> int:
        return self.count_subscribers("/palletizer/jog_xyz_delta")

    def _jog_delta_ready(self) -> bool:
        return self._jog_delta_subscribers() > 0

    def _action_busy(self) -> bool:
        with self._lock:
            return bool(self._pending_actions or self._active_actions)

    def _mark_action_pending(self, action: str) -> None:
        with self._lock:
            self._pending_actions.add(action)

    def _mark_action_accepted(self, action: str) -> None:
        with self._lock:
            self._pending_actions.discard(action)
            self._active_actions.add(action)

    def _clear_action(self, action: str) -> None:
        with self._lock:
            self._pending_actions.discard(action)
            self._active_actions.discard(action)

    def _publish_command_text(self, command: str, command_name: str = "raw_command") -> None:
        command = command.strip()
        if not command:
            self._send({"type": "error", "message": "empty raw command"})
            return
        subscribers = self._command_topic_subscribers()
        if subscribers <= 0:
            self._send({
                "type": "command_unavailable",
                "command": command_name,
                "data": command,
                "message": "no ROS subscriber on /palletizer/command",
            })
            return
        msg = String()
        msg.data = command
        self._command_pub.publish(msg)
        self._send({"type": "command_ack", "command": command_name, "data": command, "subscribers": subscribers})

    def _service_ready(self, client: Any, timeout_sec: float = SERVICE_READY_TIMEOUT_SEC) -> bool:
        if not client:
            return False
        if client.service_is_ready():
            return True
        return bool(client.wait_for_service(timeout_sec=timeout_sec))

    def _action_ready(self, client: Any, timeout_sec: float = ACTION_READY_TIMEOUT_SEC) -> bool:
        if not client:
            return False
        if client.server_is_ready():
            return True
        return bool(client.wait_for_server(timeout_sec=timeout_sec))

    def _send_jog_command(self, payload: Dict[str, Any]) -> None:
        if self._action_busy():
            self._send({
                "type": "command_rejected",
                "command": "jog_xyz",
                "message": "action active; wait for result or stop before manual jog",
            })
            return
        if self._fast_move_ready():
            msg = Float32MultiArray()
            msg.data = [
                float(payload.get("x_mm", 0.0)),
                float(payload.get("y_mm", 0.0)),
                float(payload.get("z_mm", 0.0)),
                float(payload.get("speed_mm_s", 25.0)),
                float(payload.get("accel_mm_s2", 50.0)),
            ]
            self._fast_move_pub.publish(msg)
            self._send({
                "type": "command_ack",
                "command": "jog_xyz",
                "transport": "fast_move_xyz",
                "data": msg.data,
                "subscribers": self._fast_move_subscribers(),
            })
            return
        command = (
            f"POSXYZ {float(payload.get('x_mm', 0.0)):.3f} "
            f"{float(payload.get('y_mm', 0.0)):.3f} "
            f"{float(payload.get('z_mm', 0.0)):.3f} "
            f"{float(payload.get('speed_mm_s', 25.0)):.3f} "
            f"{float(payload.get('accel_mm_s2', 50.0)):.3f}"
        )
        if self._command_topic_ready():
            self._publish_command_text(command, "jog_xyz")
            return
        self._send({"type": "command_unavailable", "command": "jog_xyz", "fallback": "move_xyz"})
        self._send_move_goal(payload)

    def _send_jog_delta_command(self, payload: Dict[str, Any]) -> None:
        if self._action_busy():
            self._send({
                "type": "command_rejected",
                "command": "jog_xyz_delta",
                "message": "action active; wait for result or stop before manual jog",
            })
            return
        if self._jog_delta_ready():
            msg = Float32MultiArray()
            msg.data = [
                float(payload.get("dx_mm", 0.0)),
                float(payload.get("dy_mm", 0.0)),
                float(payload.get("dz_mm", 0.0)),
                float(payload.get("speed_mm_s", 25.0)),
                float(payload.get("accel_mm_s2", 50.0)),
            ]
            self._jog_delta_pub.publish(msg)
            self._send({
                "type": "command_ack",
                "command": "jog_xyz_delta",
                "transport": "jog_xyz_delta",
                "data": msg.data,
                "subscribers": self._jog_delta_subscribers(),
            })
            return

        with self._lock:
            current = list(self._state.get("axis_position_mm", [0.0, 0.0, 0.0]))
        fallback_payload = {
            "x_mm": float(current[0] if len(current) > 0 else 0.0) + float(payload.get("dx_mm", 0.0)),
            "y_mm": float(current[1] if len(current) > 1 else 0.0) + float(payload.get("dy_mm", 0.0)),
            "z_mm": float(current[2] if len(current) > 2 else 0.0) + float(payload.get("dz_mm", 0.0)),
            "speed_mm_s": float(payload.get("speed_mm_s", 25.0)),
            "accel_mm_s2": float(payload.get("accel_mm_s2", 50.0)),
            "tolerance_mm": 1.0,
            "timeout_ms": 30000,
        }
        self._send({"type": "command_unavailable", "command": "jog_xyz_delta", "fallback": "jog_xyz"})
        self._send_jog_command(fallback_payload)

    def _call_set_gripper(self, payload: Dict[str, Any]) -> None:
        closed = bool(payload.get("closed", True))
        if SetGripper is None or not self._set_gripper_client:
            self._send({"type": "service_unavailable", "service": "set_gripper", "fallback": "raw_command"})
            self._publish_command_text("SERVO 0" if closed else "SERVO 180")
            return
        if not self._set_gripper_client.service_is_ready():
            if self._command_topic_ready():
                self._send({"type": "service_unavailable", "service": "set_gripper", "fallback": "raw_command"})
                self._publish_command_text("SERVO 0" if closed else "SERVO 180")
                return
            if not self._service_ready(self._set_gripper_client):
                self._send({"type": "service_unavailable", "service": "set_gripper", "fallback": "raw_command"})
                self._publish_command_text("SERVO 0" if closed else "SERVO 180")
                return
        request = SetGripper.Request()
        request.closed = closed
        self._call_service("set_gripper", self._set_gripper_client, request)

    def _set_aux_servo(self, payload: Dict[str, Any]) -> None:
        if payload.get("enabled", True) is False:
            self._publish_command_text("SERVO OFF")
            return
        if "pulse_us" in payload:
            self._publish_command_text(f"SERVO_US {int(payload.get('pulse_us', 1500))}")
            return
        self._publish_command_text(f"SERVO {float(payload.get('angle_deg', 90.0)):.1f}")

    def _send_move_goal(self, payload: Dict[str, Any]) -> None:
        if not self._action_server_available("/palletizer/move_xyz"):
            self._send({"type": "action_unavailable", "action": "move_xyz"})
            return
        move_client = self._ensure_move_client()
        if not self._action_ready(move_client):
            self._send({"type": "action_unavailable", "action": "move_xyz"})
            return

        goal = MoveXYZ.Goal()
        goal.x_mm = float(payload.get("x_mm", 0.0))
        goal.y_mm = float(payload.get("y_mm", 0.0))
        goal.z_mm = float(payload.get("z_mm", 0.0))
        goal.use_a = bool(payload.get("use_a", False))
        goal.a_deg = float(payload.get("a_deg", 0.0))
        goal.speed_mm_s = float(payload.get("speed_mm_s", 25.0))
        goal.accel_mm_s2 = float(payload.get("accel_mm_s2", 50.0))
        goal.angular_speed_deg_s = float(payload.get("angular_speed_deg_s", 0.0))
        goal.angular_accel_deg_s2 = float(payload.get("angular_accel_deg_s2", 0.0))
        goal.tolerance_mm = float(payload.get("tolerance_mm", 1.0))
        goal.angular_tolerance_deg = float(payload.get("angular_tolerance_deg", 1.0))
        goal.timeout_ms = int(payload.get("timeout_ms", 30000))
        self._send({"type": "action_goal_sent", "action": "move_xyz", "goal": payload})
        self._mark_action_pending("move_xyz")
        future = move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        future.add_done_callback(lambda f: self._on_action_goal_response("move_xyz", f))

    def _send_home_goal(self, payload: Dict[str, Any]) -> None:
        axis = int(payload.get("axis", 0))
        if HomeAxis is None or not PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS or not self._action_server_available("/palletizer/home_axis"):
            axis_name = self._axis_name(axis)
            self._send({"type": "action_unavailable", "action": "home_axis", "fallback": "raw_command"})
            if axis_name != "UNKNOWN":
                self._publish_command_text(f"HOME {axis_name}")
            return
        home_client = self._ensure_home_client()
        if not self._action_ready(home_client):
            axis_name = self._axis_name(axis)
            self._send({"type": "action_unavailable", "action": "home_axis", "fallback": "raw_command"})
            if axis_name != "UNKNOWN":
                self._publish_command_text(f"HOME {axis_name}")
            return

        goal = HomeAxis.Goal()
        goal.axis = axis
        goal.set_limits = bool(payload.get("set_limits", False))
        goal.min_mm = float(payload.get("min_mm", 0.0))
        goal.max_mm = float(payload.get("max_mm", 0.0))
        goal.fast_rpm = float(payload.get("fast_rpm", 300.0))
        goal.slow_rpm = float(payload.get("slow_rpm", 80.0))
        goal.timeout_ms = int(payload.get("timeout_ms", 120000))
        self._send({"type": "action_goal_sent", "action": "home_axis", "goal": payload})
        self._mark_action_pending("home_axis")
        future = home_client.send_goal_async(goal, feedback_callback=self._on_home_feedback)
        future.add_done_callback(lambda f: self._on_action_goal_response("home_axis", f))

    def _send_origin_goal(self, payload: Dict[str, Any]) -> None:
        axis = int(payload.get("axis", 0))
        if GoOrigin is None or not PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS or not self._action_server_available("/palletizer/go_origin"):
            axis_name = self._axis_name(axis)
            self._send({"type": "action_unavailable", "action": "go_origin", "fallback": "raw_command"})
            if axis_name != "UNKNOWN":
                self._publish_command_text(f"ORIGIN {axis_name}")
            return
        origin_client = self._ensure_origin_client()
        if not self._action_ready(origin_client):
            axis_name = self._axis_name(axis)
            self._send({"type": "action_unavailable", "action": "go_origin", "fallback": "raw_command"})
            if axis_name != "UNKNOWN":
                self._publish_command_text(f"ORIGIN {axis_name}")
            return

        goal = GoOrigin.Goal()
        goal.axis = axis
        goal.tolerance_mm = float(payload.get("tolerance_mm", 1.0))
        goal.timeout_ms = int(payload.get("timeout_ms", 30000))
        self._send({"type": "action_goal_sent", "action": "go_origin", "goal": payload})
        self._mark_action_pending("go_origin")
        future = origin_client.send_goal_async(goal, feedback_callback=self._on_origin_feedback)
        future.add_done_callback(lambda f: self._on_action_goal_response("go_origin", f))

    def _on_action_goal_response(self, action: str, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._clear_action(action)
            self._send({"type": "action_error", "action": action, "message": str(exc)})
            return
        if not goal_handle.accepted:
            self._clear_action(action)
            self._send({"type": "action_rejected", "action": action})
            return
        self._mark_action_accepted(action)
        self._send({"type": "action_accepted", "action": action})
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_action_result(action, f))

    def _on_action_result(self, action: str, future: Any) -> None:
        try:
            response = future.result()
            result = response.result
            status = int(response.status)
        except Exception as exc:
            self._clear_action(action)
            self._send({"type": "action_error", "action": action, "message": str(exc)})
            return
        payload = {"type": "action_result", "action": action, "status": status, "result": self._action_result_dict(action, result)}
        with self._lock:
            self._state["last_action"] = payload
        self._clear_action(action)
        self._send(payload)
        self._send_state(force=True)

    def _on_move_feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self._send({
            "type": "action_feedback",
            "action": "move_xyz",
            "feedback": {
                "current_x_mm": float(feedback.current_x_mm),
                "current_y_mm": float(feedback.current_y_mm),
                "current_z_mm": float(feedback.current_z_mm),
                "current_a_deg": float(getattr(feedback, "current_a_deg", 0.0)),
                "error_x_mm": float(feedback.error_x_mm),
                "error_y_mm": float(feedback.error_y_mm),
                "error_z_mm": float(feedback.error_z_mm),
                "error_a_deg": float(getattr(feedback, "error_a_deg", 0.0)),
                "progress": float(feedback.progress),
                "state": str(feedback.state),
            },
        })

    def _on_home_feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self._send({
            "type": "action_feedback",
            "action": "home_axis",
            "feedback": {
                "axis": int(feedback.axis),
                "current_position_mm": float(feedback.current_position_mm),
                "progress": float(feedback.progress),
                "state": str(feedback.state),
            },
        })

    def _on_origin_feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self._send({
            "type": "action_feedback",
            "action": "go_origin",
            "feedback": {
                "axis": int(feedback.axis),
                "current_position_mm": float(feedback.current_position_mm),
                "error_mm": float(feedback.error_mm),
                "progress": float(feedback.progress),
                "state": str(feedback.state),
            },
        })

    def _action_result_dict(self, action: str, result: Any) -> Dict[str, Any]:
        if action == "move_xyz":
            return {
                "success": bool(result.success),
                "message": str(result.message),
                "final_x_mm": float(result.final_x_mm),
                "final_y_mm": float(result.final_y_mm),
                "final_z_mm": float(result.final_z_mm),
                "final_a_deg": float(getattr(result, "final_a_deg", 0.0)),
            }
        return {
            "success": bool(result.success),
            "message": str(result.message),
            "final_position_mm": float(result.final_position_mm),
        }

    def _call_enable_axis(self, payload: Dict[str, Any]) -> None:
        axis = int(payload.get("axis", 3))
        enable = bool(payload.get("enable", True))
        if EnableAxis is None or not self._service_ready(self._enable_axis_client):
            axis_name = self._axis_name(axis)
            self._send({"type": "service_unavailable", "service": "enable_axis", "fallback": "raw_command"})
            if axis_name != "UNKNOWN":
                self._publish_command_text(f"{'ENABLE' if enable else 'DISABLE'} {axis_name}")
            return
        request = EnableAxis.Request()
        request.axis = axis
        request.enable = enable
        self._call_service("enable_axis", self._enable_axis_client, request)

    def _call_set_zero(self, payload: Dict[str, Any]) -> None:
        if SetZero is None:
            self._send({"type": "service_unavailable", "service": "set_zero"})
            return
        request = SetZero.Request()
        request.axis = int(payload.get("axis", 0))
        request.min_mm = float(payload.get("min_mm", 0.0))
        request.max_mm = float(payload.get("max_mm", 0.0))
        self._call_service("set_zero", self._set_zero_client, request)

    def _call_set_axis_limits(self, payload: Dict[str, Any]) -> None:
        if SetAxisLimits is None:
            self._send({"type": "service_unavailable", "service": "set_axis_limits"})
            return
        request = SetAxisLimits.Request()
        request.axis = int(payload.get("axis", 0))
        request.min_mm = float(payload.get("min_mm", 0.0))
        request.max_mm = float(payload.get("max_mm", 0.0))
        self._call_service("set_axis_limits", self._set_axis_limits_client, request)

    def _call_release_stall(self, payload: Dict[str, Any]) -> None:
        if ReleaseStall is None:
            self._send({"type": "service_unavailable", "service": "release_stall"})
            return
        request = ReleaseStall.Request()
        request.axis = int(payload.get("axis", 3))
        self._call_service("release_stall", self._release_stall_client, request)

    def _call_empty_service(self, name: str, client: Any, request: Any) -> None:
        self._call_service(name, client, request)

    def _call_service(self, name: str, client: Any, request: Any) -> None:
        if request is None or not self._service_ready(client):
            self._send({"type": "service_unavailable", "service": name})
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._on_service_result(name, f))
        self._send({"type": "service_call_sent", "service": name})

    def _on_service_result(self, name: str, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._send({"type": "service_error", "service": name, "message": str(exc)})
            return
        self._send({"type": "service_result", "service": name, "response": self._service_response_dict(response)})

    def _service_response_dict(self, response: Any) -> Dict[str, Any]:
        payload = {
            "success": bool(getattr(response, "success", False)),
            "message": str(getattr(response, "message", "")),
        }
        for field in [
            "safety_fault",
            "safety_reason",
            "homing_state",
            "online",
            "enabled_ok",
            "enabled",
            "stalled",
            "raw35_ok",
            "angle_error_ok",
            "enc31",
            "raw35",
            "position_mm",
            "velocity_mm_s",
            "angle_error",
            "home91",
            "home3b_single",
            "home3b_origin",
            "limits_configured",
            "limit_min_mm",
            "limit_max_mm",
            "closed",
            "angle_deg",
            "pulse_us",
        ]:
            if hasattr(response, field):
                value = getattr(response, field)
                payload[field] = list(value) if not isinstance(value, (str, bool, int, float)) else value
        return payload

    def _availability_snapshot(self) -> Dict[str, bool]:
        now = now_ms()
        connections, _ = self._connection_snapshot()
        ros_ready = bool(connections.get("ros_communication"))
        action_servers = self._palletizer_action_servers()
        raw = {
            "move_xyz": "/palletizer/move_xyz" in action_servers,
            "home_axis": PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS and "/palletizer/home_axis" in action_servers,
            "go_origin": PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS and "/palletizer/go_origin" in action_servers,
            "enable_axis": self._enable_axis_client.service_is_ready() if self._enable_axis_client else False,
            "set_zero": self._set_zero_client.service_is_ready() if self._set_zero_client else False,
            "set_axis_limits": self._set_axis_limits_client.service_is_ready() if self._set_axis_limits_client else False,
            "set_gripper": self._set_gripper_client.service_is_ready() if self._set_gripper_client else False,
            "clear_fault": self._clear_fault_client.service_is_ready() if self._clear_fault_client else False,
            "release_stall": self._release_stall_client.service_is_ready() if self._release_stall_client else False,
            "get_driver_status": self._get_driver_status_client.service_is_ready() if self._get_driver_status_client else False,
            "command_topic": self._command_topic_ready(),
            "jog_delta_topic": self._jog_delta_ready(),
            "fast_move_topic": self._fast_move_ready(),
            "fast_jog": (self._jog_delta_ready() or self._fast_move_ready() or self._command_topic_ready()) and not self._action_busy(),
            "aux_servo": (self._set_gripper_client.service_is_ready() if self._set_gripper_client else False) or self._command_topic_ready(),
        }
        stable = {key: self._stable_availability(key, value, now, ros_ready) for key, value in raw.items()}
        stable["fast_jog"] = (stable.get("jog_delta_topic") or stable.get("fast_move_topic") or stable.get("command_topic")) and not self._action_busy()
        stable["aux_servo"] = stable.get("set_gripper") or stable.get("command_topic")
        return stable

    def _stable_availability(self, key: str, current: bool, now: int, ros_ready: bool) -> bool:
        if current and ros_ready:
            self._availability_seen_ms[key] = now
            return True
        if not ros_ready:
            return False
        last_seen = self._availability_seen_ms.get(key)
        return last_seen is not None and now - last_seen <= PALLETIZER_AVAILABILITY_HOLD_MS

    def _send_state(self, force: bool = False) -> None:
        now = now_ms()
        if not force and now - self._last_state_emit_ms < UI_STATE_MIN_PERIOD_MS:
            return
        self._last_state_emit_ms = now
        self._send({"type": "state", "state": self.snapshot()})

    def _send(self, payload: Dict[str, Any]) -> None:
        if self._emit:
            self._emit(payload)


class WebSocketPeer:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer
        self._send_lock = asyncio.Lock()

    async def handshake(self) -> bool:
        request = await self.reader.readuntil(b"\r\n\r\n")
        lines = request.decode("utf-8", errors="ignore").split("\r\n")
        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key.strip().lower()] = value.strip()
        key = headers.get("sec-websocket-key")
        if not key:
            return False
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n"
            "\r\n"
        )
        self.writer.write(response.encode())
        await self.writer.drain()
        return True

    async def send_json(self, payload: Dict[str, Any]) -> None:
        async with self._send_lock:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            header = bytearray([0x81])
            length = len(data)
            if length < 126:
                header.append(length)
            elif length < 65536:
                header.append(126)
                header.extend(struct.pack("!H", length))
            else:
                header.append(127)
                header.extend(struct.pack("!Q", length))
            self.writer.write(bytes(header) + data)
            await self.writer.drain()

    async def read_json(self) -> Optional[Dict[str, Any]]:
        first = await self.reader.readexactly(2)
        opcode = first[0] & 0x0F
        masked = bool(first[1] & 0x80)
        length = first[1] & 0x7F
        if length == 126:
            length = struct.unpack("!H", await self.reader.readexactly(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", await self.reader.readexactly(8))[0]
        mask = await self.reader.readexactly(4) if masked else b"\x00\x00\x00\x00"
        data = await self.reader.readexactly(length) if length else b""
        if masked:
            data = bytes(byte ^ mask[index % 4] for index, byte in enumerate(data))
        if opcode == 0x8:
            return None
        if opcode == 0x9:
            await self._send_pong(data)
            return {}
        if opcode != 0x1:
            return {}
        return json.loads(data.decode("utf-8"))

    async def _send_pong(self, data: bytes) -> None:
        self.writer.write(bytes([0x8A, len(data)]) + data)
        await self.writer.drain()

    async def close(self) -> None:
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except Exception:
            pass


class WebSocketServer:
    def __init__(self, node: PalletizerUiNode, host: str, port: int) -> None:
        self.node = node
        self.host = host
        self.port = port
        self.peers: set[WebSocketPeer] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    async def run(self) -> None:
        self.loop = asyncio.get_running_loop()
        self.node.set_emitter(self.emit_from_ros)
        server = await asyncio.start_server(self.handle_peer, self.host, self.port)
        self.node.get_logger().info(f"UI WebSocket listening on ws://{self.host}:{self.port}")
        async with server:
            await server.serve_forever()

    async def handle_peer(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = WebSocketPeer(reader, writer)
        try:
            if not await peer.handshake():
                await peer.close()
                return
            self.peers.add(peer)
            await peer.send_json({"type": "hello", "state": self.node.snapshot()})
            while True:
                payload = await peer.read_json()
                if payload is None:
                    break
                if payload:
                    self.node.handle_command(payload)
        except Exception as exc:
            self.node.get_logger().warn(f"websocket client closed: {exc}")
        finally:
            self.peers.discard(peer)
            await peer.close()

    def emit_from_ros(self, payload: Dict[str, Any]) -> None:
        if not self.loop:
            return
        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self.broadcast(payload)))

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        if not self.peers:
            return
        stale = []
        for peer in list(self.peers):
            try:
                await peer.send_json(payload)
            except Exception:
                stale.append(peer)
        for peer in stale:
            self.peers.discard(peer)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PalletizerUiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = os.environ.get("PALLETIZER_UI_HOST", "127.0.0.1")
    port = int(os.environ.get("PALLETIZER_UI_PORT", "8765"))
    try:
        asyncio.run(WebSocketServer(node, host, port).run())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

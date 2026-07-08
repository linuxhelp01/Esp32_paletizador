import { FormEvent, useState } from "react";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  lastFeedback: Record<string, unknown> | null;
  lastResult: Record<string, unknown> | null;
  send: (payload: Record<string, unknown>) => void;
};

type InterfaceRow = {
  kind: string;
  name: string;
  type: string;
  state: string;
  detail: string;
};

function ageText(value: unknown): string {
  const age = Number(value);
  if (!Number.isFinite(age)) return "sin datos";
  if (age < 1000) return `${Math.round(age)} ms`;
  return `${(age / 1000).toFixed(1)} s`;
}

function okText(value: unknown): string {
  return value ? "OK" : "N/D";
}

function vectorText(values: unknown, digits = 1): string {
  if (!Array.isArray(values)) return "[]";
  return `[${values.map((item) => Number(item ?? 0).toFixed(digits)).join(", ")}]`;
}

export function NodeInterfacesPanel({ state, lastFeedback, lastResult, send }: Props) {
  const [rawCommand, setRawCommand] = useState("PING");
  const availability = state.availability;
  const ages = state.last_seen_age_ms;
  const status = state.status;

  const rows: InterfaceRow[] = [
    {
      kind: "Publisher",
      name: "/joint_states",
      type: "sensor_msgs/msg/JointState",
      state: okText(state.connections.joint_states),
      detail: `${ageText(ages.joint_states)} | ${state.joint_states.name.length || 5} joints`,
    },
    {
      kind: "Publisher",
      name: "/palletizer/axis_position_mm",
      type: "std_msgs/msg/Float32MultiArray",
      state: okText(state.connections.axis_position_mm),
      detail: `${ageText(ages.axis_position_mm)} | ${vectorText(state.axis_position_mm, 2)} mm`,
    },
    {
      kind: "Publisher",
      name: "/palletizer/motor_rpm",
      type: "std_msgs/msg/Float32MultiArray",
      state: okText(state.connections.motor_rpm),
      detail: `${ageText(ages.motor_rpm)} | ${vectorText(state.motor_rpm, 0)} rpm`,
    },
    {
      kind: "Publisher",
      name: "/palletizer/status",
      type: "std_msgs/msg/String JSON",
      state: okText(state.connections.status),
      detail: `${ageText(ages.status)} | fault=${String(status.fault ?? "N/D")} homing=${String(status.homing ?? "N/D")}`,
    },
    {
      kind: "Publisher",
      name: "/palletizer/fault_state",
      type: "std_msgs/msg/String",
      state: okText(state.connections.fault_state),
      detail: `${ageText(ages.fault_state)} | ${state.fault_state || "sin estado"}`,
    },
    {
      kind: "Subscriber",
      name: "/palletizer/emergency_stop",
      type: "std_msgs/msg/Bool",
      state: "accionador",
      detail: "Boton Parada emergencia en Estado de maquina",
    },
    {
      kind: "Subscriber",
      name: "/palletizer/jog_xyz_delta",
      type: "std_msgs/msg/Float32MultiArray",
      state: okText(availability.jog_delta_topic),
      detail: "Control manual XY/Z envia deltas relativos al ESP32",
    },
    {
      kind: "Subscriber",
      name: "/palletizer/fast_move_xyz",
      type: "std_msgs/msg/Float32MultiArray",
      state: okText(availability.fast_move_topic),
      detail: "Fallback de movimiento absoluto rapido",
    },
    {
      kind: "Subscriber",
      name: "/palletizer/command",
      type: "std_msgs/msg/String",
      state: okText(availability.command_topic),
      detail: "Comando textual de diagnostico/fallback",
    },
    {
      kind: "Service",
      name: "/palletizer/set_gripper",
      type: "palletizer_msgs/srv/SetGripper",
      state: okText(availability.set_gripper),
      detail: "Botones Abrir/Cerrar en Controles manuales",
    },
    {
      kind: "Action",
      name: "/palletizer/move_xyz",
      type: "palletizer_msgs/action/MoveXYZ",
      state: okText(availability.move_xyz),
      detail: lastFeedback?.action === "move_xyz"
        ? `feedback=${String(lastFeedback.state ?? "activo")}`
        : lastResult?.action === "move_xyz"
          ? `resultado=${String(lastResult.message ?? "recibido")}`
          : "Panel Movimiento XYZ + Eje A",
    },
  ];

  const submitRaw = (event: FormEvent) => {
    event.preventDefault();
    send({ type: "raw_command", command: rawCommand });
  };

  return (
    <section className="section node-interfaces">
      <div className="section-heading">
        <h2>Nodo ESP32</h2>
        <span>{state.connections.ros_communication ? "mensajes ROS activos" : "sin mensajes ROS"}</span>
      </div>

      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Tipo</th>
              <th>Interfaz</th>
              <th>Mensaje</th>
              <th>Estado</th>
              <th>Dato / accion</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((row) => (
              <tr key={`${row.kind}:${row.name}`}>
                <td>{row.kind}</td>
                <td>{row.name}</td>
                <td>{row.type}</td>
                <td>{row.state}</td>
                <td>{row.detail}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <form className="raw-command-form" onSubmit={submitRaw}>
        <label>
          <span>/palletizer/command</span>
          <input value={rawCommand} onChange={(event) => setRawCommand(event.target.value)} />
        </label>
        <button className="secondary" type="submit" disabled={!availability.command_topic}>Enviar comando</button>
      </form>
    </section>
  );
}

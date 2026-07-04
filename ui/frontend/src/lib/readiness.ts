import { PalletizerState } from "./types";

export const MOTOR_COUNT = 5;

function boolArray(value: unknown): boolean[] {
  return Array.isArray(value) ? value.map(Boolean) : [];
}

function allTrue(values: boolean[], expectedLength: number) {
  return values.length >= expectedLength && values.slice(0, expectedLength).every(Boolean);
}

function noneTrue(values: boolean[], expectedLength: number) {
  return values.length >= expectedLength && values.slice(0, expectedLength).every((value) => !value);
}

function countTrue(values: boolean[]) {
  return values.filter(Boolean).length;
}

export function machineReadiness(state: PalletizerState, uiConnected = true) {
  const online = boolArray(state.status?.online);
  const enabled = boolArray(state.status?.enabled);
  const enabledOk = boolArray(state.status?.enabledOk);
  const stalled = boolArray(state.status?.stalled);
  const moveStatus = boolArray(state.status?.moveStatus);
  const faultText = String(state.fault_state || "");
  const hasFault = faultText.startsWith("FAULT") || state.status?.fault === 1;

  const esp32Physical = Boolean(state.connections.esp32_physical ?? state.connections.palletizer_node);
  const rosCommunication = Boolean(state.connections.ros_communication ?? state.connections.ros_messages);
  const statusFresh = Boolean(state.connections.status);
  const jointStatesFresh = Boolean(state.connections.joint_states);
  const motorRpmFresh = Boolean(state.connections.motor_rpm);
  const axisPositionFresh = Boolean(state.connections.axis_position_mm);
  const motorStateTopicsFresh = statusFresh && jointStatesFresh && motorRpmFresh;
  const telemetryFresh = motorStateTopicsFresh && axisPositionFresh;

  const motorsOnlineKnown = online.length >= MOTOR_COUNT;
  const enabledKnown = enabled.length >= MOTOR_COUNT;
  const enabledOkKnown = enabledOk.length >= MOTOR_COUNT;
  const stalledKnown = stalled.length >= MOTOR_COUNT;

  const onlineCount = countTrue(online.slice(0, MOTOR_COUNT));
  const enabledCount = countTrue(enabled.slice(0, MOTOR_COUNT));
  const enableValidCount = countTrue(enabledOk.slice(0, MOTOR_COUNT));
  const stalledCount = countTrue(stalled.slice(0, MOTOR_COUNT));

  const allMotorsOnline = allTrue(online, MOTOR_COUNT);
  const allMotorsEnabled = allTrue(enabled, MOTOR_COUNT);
  const allEnableValid = allTrue(enabledOk, MOTOR_COUNT);
  const noMotorStalled = stalledKnown && noneTrue(stalled, MOTOR_COUNT);
  const moving = moveStatus.some(Boolean);

  const operative = Boolean(
    uiConnected &&
    esp32Physical &&
    rosCommunication &&
    telemetryFresh &&
    motorsOnlineKnown &&
    enabledKnown &&
    enabledOkKnown &&
    stalledKnown &&
    allMotorsOnline &&
    allMotorsEnabled &&
    allEnableValid &&
    noMotorStalled &&
    !hasFault
  );

  return {
    uiConnected,
    esp32Physical,
    rosCommunication,
    statusFresh,
    jointStatesFresh,
    motorRpmFresh,
    axisPositionFresh,
    motorStateTopicsFresh,
    telemetryFresh,
    motorsOnlineKnown,
    enabledKnown,
    enabledOkKnown,
    stalledKnown,
    onlineCount,
    enabledCount,
    enableValidCount,
    stalledCount,
    allMotorsOnline,
    allMotorsEnabled,
    allEnableValid,
    noMotorStalled,
    hasFault,
    moving,
    operative,
    ready: operative && !moving
  };
}

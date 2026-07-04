import { PalletizerState } from "../lib/types";
import { MOTOR_COUNT, machineReadiness } from "../lib/readiness";

type Props = {
  connected: boolean;
  state: PalletizerState;
  lastPing: Record<string, unknown> | null;
  send: (payload: Record<string, unknown>) => void;
};

function pingClass(lastPing: Record<string, unknown> | null) {
  if (!lastPing) return "pill";
  if (Boolean(lastPing.driver_status_pending)) return "pill warn";
  return Boolean(lastPing.ok) ? "pill ok" : "pill bad";
}

function pingText(lastPing: Record<string, unknown> | null) {
  if (!lastPing) return "Ping sin ejecutar";
  if (Boolean(lastPing.driver_status_pending)) return "Ping pendiente";
  return Boolean(lastPing.ok) ? "Ping OK" : "Ping fallo";
}

export function StatusBar({ connected, state, lastPing, send }: Props) {
  const readiness = machineReadiness(state, connected);
  const statusAge = state.last_seen_age_ms.status;
  const statusAgeText = Number.isFinite(statusAge) ? `${Math.round(statusAge)} ms` : "sin datos";
  const pingMessage = String(lastPing?.message ?? "Ejecuta ping para verificar ESP32 y ROS");

  return (
    <header className="status-bar">
      <div>
        <h1>Paletizador</h1>
        <p>{state.ros.palletizer_node} · ROS_DOMAIN_ID {state.ros.domain_id || "sin definir"} · status {statusAgeText}</p>
      </div>
      <div className="status-items">
        <span className={connected ? "pill ok" : "pill bad"}>{connected ? "UI conectada" : "UI desconectada"}</span>
        <span className={readiness.esp32Physical ? "pill ok" : "pill bad"}>{readiness.esp32Physical ? "Conexion fisica ESP32" : "ESP32 sin conexion fisica"}</span>
        <span className={readiness.rosCommunication ? "pill ok" : "pill bad"}>{readiness.rosCommunication ? "Comunicacion ROS" : "Sin comunicacion ROS"}</span>
        <span className={readiness.motorStateTopicsFresh && readiness.allMotorsOnline ? "pill ok" : "pill bad"}>Motores {readiness.onlineCount}/{MOTOR_COUNT}</span>
        <span className={readiness.operative ? "pill ok" : "pill bad"}>{readiness.operative ? "Paletizador operativo" : "Paletizador no operativo"}</span>
        <span className={pingClass(lastPing)} title={pingMessage}>{pingText(lastPing)}</span>
        <button className="secondary ping-button" type="button" disabled={!connected} onClick={() => send({ type: "ping" })}>Ping</button>
      </div>
    </header>
  );
}

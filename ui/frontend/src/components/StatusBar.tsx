import { PalletizerState } from "../lib/types";
import { MOTOR_COUNT, machineReadiness } from "../lib/readiness";

type Props = {
  connected: boolean;
  state: PalletizerState;
};

export function StatusBar({ connected, state }: Props) {
  const readiness = machineReadiness(state, connected);
  const statusAge = state.last_seen_age_ms.status;
  const statusAgeText = Number.isFinite(statusAge) ? `${Math.round(statusAge)} ms` : "sin datos";

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
      </div>
    </header>
  );
}

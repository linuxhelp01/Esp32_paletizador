import { PalletizerState } from "../lib/types";

type Props = {
  connected: boolean;
  state: PalletizerState;
};

export function StatusBar({ connected, state }: Props) {
  const fault = String(state.fault_state || "");
  const hasFault = fault.startsWith("FAULT") || state.status?.fault === 1;

  return (
    <header className="status-bar">
      <div>
        <h1>Paletizador</h1>
        <p>{state.ros.palletizer_node} · ROS_DOMAIN_ID {state.ros.domain_id || "sin definir"}</p>
      </div>
      <div className="status-items">
        <span className={connected ? "pill ok" : "pill bad"}>{connected ? "UI conectada" : "UI desconectada"}</span>
        <span className={hasFault ? "pill bad" : "pill ok"}>{hasFault ? fault || "FAULT" : "Sin falla"}</span>
      </div>
    </header>
  );
}

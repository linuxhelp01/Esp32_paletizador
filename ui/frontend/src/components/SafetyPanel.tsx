import { PalletizerState } from "../lib/types";
import { machineReadiness } from "../lib/readiness";

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

export function SafetyPanel({ state, send }: Props) {
  const fault = String(state.fault_state || "OK");
  const readiness = machineReadiness(state);

  return (
    <section className="section safety machine-state">
      <div className="section-heading">
        <h2>Estado de maquina</h2>
        <span>{fault}</span>
      </div>
      <div className="state-stack">
        <div className={readiness.ready ? "state-card ok active" : "state-card ok"}><strong>▶</strong><span>Listo</span></div>
        <div className={readiness.moving ? "state-card info active" : "state-card info"}><strong>↻</strong><span>En movimiento</span></div>
        <div className={!readiness.moving && readiness.operative ? "state-card warn" : "state-card warn active"}><strong>Ⅱ</strong><span>Pausado</span></div>
        <div className={readiness.hasFault ? "state-card danger active" : "state-card danger"}><strong>■</strong><span>E-Stop</span></div>
      </div>
      <button className="danger" onClick={() => send({ type: "emergency_stop", data: true })}>Parada emergencia</button>
      <div className="button-row compact-actions">
        <button className="secondary" disabled={!state.availability.clear_fault} onClick={() => send({ type: "clear_fault" })}>Limpiar falla</button>
        <button className="secondary" disabled={!state.availability.release_stall} onClick={() => send({ type: "release_stall", request: { axis: 3 } })}>Liberar stall</button>
      </div>
    </section>
  );
}

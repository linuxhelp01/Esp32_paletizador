import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

export function SafetyPanel({ state, send }: Props) {
  const fault = String(state.fault_state || "OK");
  return (
    <section className="section safety">
      <div className="section-heading">
        <h2>Seguridad</h2>
        <span>{fault}</span>
      </div>
      <button className="danger" onClick={() => send({ type: "emergency_stop", data: true })}>Parada emergencia</button>
      <button className="secondary" disabled={!state.availability.clear_fault} onClick={() => send({ type: "clear_fault" })}>Limpiar falla</button>
      <button className="secondary" disabled={!state.availability.release_stall} onClick={() => send({ type: "release_stall", request: { axis: 3 } })}>Liberar stall</button>
    </section>
  );
}

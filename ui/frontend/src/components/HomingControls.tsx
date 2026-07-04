import { useState } from "react";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

export function HomingControls({ state, send }: Props) {
  const [axis, setAxis] = useState(0);
  const canHome = Boolean(state.availability.home_axis || state.availability.command_topic);
  const canOrigin = Boolean(state.availability.go_origin || state.availability.command_topic);
  return (
    <section className="section">
      <div className="section-heading">
        <h2>Homing</h2>
        <span>{String(state.status?.homing ?? "idle")}</span>
      </div>
      <div className="inline-controls">
        <label>
          <span>Eje</span>
          <select value={axis} onChange={(event) => setAxis(Number(event.target.value))}>
            <option value={0}>X</option>
            <option value={1}>Y</option>
            <option value={2}>Z</option>
            <option value={3}>ALL</option>
            <option value={4}>A rotatorio</option>
          </select>
        </label>
      </div>
      <div className="button-row">
        <button className="secondary" disabled={!canHome} onClick={() => send({ type: "home_axis", goal: { axis, set_limits: false, min_mm: 0, max_mm: 0, fast_rpm: 300, slow_rpm: 80, timeout_ms: 120000 } })}>Home</button>
        <button className="secondary" disabled={!canOrigin} onClick={() => send({ type: "go_origin", goal: { axis, tolerance_mm: 1, timeout_ms: 30000 } })}>Ir a origen</button>
      </div>
    </section>
  );
}

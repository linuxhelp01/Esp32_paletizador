import { useState } from "react";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

export function ServiceControls({ state, send }: Props) {
  const [axis, setAxis] = useState(0);
  const [minMm, setMinMm] = useState(-5);
  const [maxMm, setMaxMm] = useState(300);

  const canEnable = Boolean(state.availability.enable_axis || state.availability.command_topic);
  const linearAxis = axis >= 0 && axis <= 2;

  return (
    <section className="section">
      <div className="section-heading">
        <h2>Configuracion</h2>
        <span>servicios opcionales</span>
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
            <option value={5}>X1 motor</option>
            <option value={6}>X2 motor</option>
          </select>
        </label>
        <label>
          <span>Min mm</span>
          <input type="number" value={minMm} onChange={(event) => setMinMm(Number(event.target.value))} />
        </label>
        <label>
          <span>Max mm</span>
          <input type="number" value={maxMm} onChange={(event) => setMaxMm(Number(event.target.value))} />
        </label>
      </div>
      <div className="button-row">
        <button className="secondary" disabled={!canEnable} onClick={() => send({ type: "enable_axis", request: { axis, enable: true } })}>Enable</button>
        <button className="secondary" disabled={!canEnable} onClick={() => send({ type: "enable_axis", request: { axis, enable: false } })}>Disable</button>
        <button className="secondary" disabled={!state.availability.set_axis_limits || !linearAxis} onClick={() => send({ type: "set_axis_limits", request: { axis, min_mm: minMm, max_mm: maxMm } })}>Set limits</button>
        <button className="secondary" disabled={!state.availability.set_zero || !linearAxis} onClick={() => send({ type: "set_zero", request: { axis, min_mm: minMm, max_mm: maxMm } })}>Set zero</button>
        <button className="secondary" disabled={!state.availability.get_driver_status} onClick={() => send({ type: "get_driver_status" })}>Driver status</button>
      </div>
    </section>
  );
}

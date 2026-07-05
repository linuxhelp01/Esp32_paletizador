import { useState } from "react";
import { mm, percent } from "../lib/format";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  lastFeedback: Record<string, unknown> | null;
  lastResult: Record<string, unknown> | null;
  send: (payload: Record<string, unknown>) => void;
};

function numberValue(value: unknown, fallback = 0) {
  const numeric = Number(value ?? fallback);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function clamp01(value: unknown) {
  return Math.max(0, Math.min(1, numberValue(value, 0)));
}

export function HomingControls({ state, lastFeedback, lastResult, send }: Props) {
  const [axis, setAxis] = useState(0);
  const canHome = Boolean(state.availability.home_axis || state.availability.command_topic);
  const canOrigin = Boolean(state.availability.go_origin || state.availability.command_topic);
  const activityFeedback = lastFeedback?.action === "home_axis" || lastFeedback?.action === "go_origin" ? lastFeedback : null;
  const activityResult = lastResult?.action === "home_axis" || lastResult?.action === "go_origin" ? lastResult : null;
  const progress = clamp01(activityFeedback?.progress);
  const currentPosition = numberValue(activityFeedback?.current_position_mm);
  const error = numberValue(activityFeedback?.error_mm);
  const resultSuccess = activityResult ? Boolean(activityResult.success) : null;

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
      <div className="feedback compact-feedback">
        <div>
          <span>Action</span>
          <strong>{activityFeedback ? String(activityFeedback.action) : "sin actividad"}</strong>
          <small>{activityFeedback ? String(activityFeedback.state ?? "feedback") : "home_axis / go_origin"}</small>
        </div>
        <div>
          <span>Progreso</span>
          <strong>{percent(progress)}</strong>
          <div className="progress-track"><span style={{ width: `${progress * 100}%` }} /></div>
        </div>
        <div>
          <span>Posicion</span>
          <strong>{mm(currentPosition)}</strong>
          <small>feedback.current_position_mm</small>
        </div>
        <div>
          <span>Error origen</span>
          <strong>{mm(error)}</strong>
          <small>{activityFeedback?.action === "go_origin" ? "feedback.error_mm" : "no aplica"}</small>
        </div>
        <div>
          <span>Resultado</span>
          <strong className={resultSuccess === true ? "result-ok" : resultSuccess === false ? "result-bad" : ""}>{activityResult ? String(activityResult.message ?? "sin mensaje") : "pendiente"}</strong>
          <small>{activityResult ? "ultimo resultado de action" : "esperando respuesta"}</small>
        </div>
      </div>
    </section>
  );
}

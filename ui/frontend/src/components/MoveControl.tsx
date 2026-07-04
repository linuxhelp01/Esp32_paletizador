import { FormEvent, useMemo, useState } from "react";
import { deg, mm, percent } from "../lib/format";
import { MoveGoal, PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  lastFeedback: Record<string, unknown> | null;
  lastResult: Record<string, unknown> | null;
  send: (payload: Record<string, unknown>) => void;
};

type NumericFieldProps = {
  label: string;
  unit: string;
  value: number;
  step?: number;
  min?: number;
  max?: number;
  disabled?: boolean;
  help: string;
  onChange: (value: string) => void;
};

const initialGoal: MoveGoal = {
  x_mm: 50,
  y_mm: 50,
  z_mm: 20,
  use_a: false,
  a_deg: 0,
  speed_mm_s: 25,
  accel_mm_s2: 50,
  angular_speed_deg_s: 90,
  angular_accel_deg_s2: 180,
  tolerance_mm: 1,
  angular_tolerance_deg: 1,
  timeout_ms: 30000
};

function numberValue(value: unknown, fallback = 0) {
  const numeric = Number(value ?? fallback);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function clamp01(value: unknown) {
  return Math.max(0, Math.min(1, numberValue(value, 0)));
}

function currentAFromState(state: PalletizerState): number {
  const statusA = Number(state.status?.a_deg ?? NaN);
  if (Number.isFinite(statusA)) return statusA;
  const names = state.joint_states.name ?? [];
  const index = names.findIndex((name) => name === "A");
  if (index < 0) return 0;
  const radians = Number(state.joint_states.position[index] ?? 0);
  return Number.isFinite(radians) ? radians * 180 / Math.PI : 0;
}

function NumericField({ label, unit, value, step = 0.1, min, max, disabled, help, onChange }: NumericFieldProps) {
  return (
    <label className="motion-field">
      <span className="motion-label">
        {label}
        <button className="info-dot" type="button" tabIndex={-1} title={help}>i</button>
      </span>
      <span className="motion-input-shell">
        <input
          type="number"
          min={min}
          max={max}
          step={step}
          value={value}
          disabled={disabled}
          onChange={(event) => onChange(event.target.value)}
        />
        <strong>{unit}</strong>
      </span>
    </label>
  );
}

export function MoveControl({ state, lastFeedback, lastResult, send }: Props) {
  const [goal, setGoal] = useState<MoveGoal>(initialGoal);
  const [previewActive, setPreviewActive] = useState(false);
  const available = Boolean(state.availability.move_xyz);
  const moveFeedback = lastFeedback?.action === "move_xyz" ? lastFeedback : null;
  const moveResult = lastResult?.action === "move_xyz" ? lastResult : null;
  const progress = clamp01(moveFeedback?.progress);
  const currentX = numberValue(moveFeedback?.current_x_mm, numberValue(state.axis_position_mm[0]));
  const currentY = numberValue(moveFeedback?.current_y_mm, numberValue(state.axis_position_mm[1]));
  const currentZ = numberValue(moveFeedback?.current_z_mm, numberValue(state.axis_position_mm[2]));
  const currentA = numberValue(moveFeedback?.current_a_deg, currentAFromState(state));

  const preview = useMemo(() => {
    const dx = goal.x_mm - numberValue(state.axis_position_mm[0]);
    const dy = goal.y_mm - numberValue(state.axis_position_mm[1]);
    const dz = goal.z_mm - numberValue(state.axis_position_mm[2]);
    const da = goal.use_a ? goal.a_deg - currentAFromState(state) : 0;
    const linearDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const linearTime = goal.speed_mm_s > 0 ? linearDistance / goal.speed_mm_s : 0;
    const angularTime = goal.use_a && goal.angular_speed_deg_s > 0 ? Math.abs(da) / goal.angular_speed_deg_s : 0;
    return {
      dx,
      dy,
      dz,
      da,
      linearDistance,
      estimatedSeconds: Math.max(linearTime, angularTime)
    };
  }, [goal, state.axis_position_mm, state.status, state.joint_states]);

  const update = (field: keyof MoveGoal, value: string) => {
    setGoal((current) => ({ ...current, [field]: field === "timeout_ms" ? Number.parseInt(value || "0", 10) : Number(value) }));
  };

  const setUseA = (checked: boolean) => {
    setGoal((current) => ({ ...current, use_a: checked }));
  };

  const submit = (event: FormEvent) => {
    event.preventDefault();
    send({ type: "move_xyz", goal });
  };

  const resultSuccess = moveResult ? Boolean(moveResult.success) : null;
  const resultText = moveResult ? String(moveResult.message ?? "sin mensaje") : "En espera de envio";
  const stateText = String(moveFeedback?.state ?? (available ? "Pendiente" : "Action no disponible"));

  return (
    <section className="section command-surface motion-command">
      <div className="motion-header">
        <div>
          <span className="motion-kicker">Action /palletizer/move_xyz</span>
          <h2>Movimiento XYZ + Eje A</h2>
        </div>
        <span className={available ? "action-badge ok" : "action-badge bad"}>{available ? "Accion disponible" : "Accion no detectada"}</span>
      </div>

      <form className="motion-form" onSubmit={submit}>
        <section className="motion-step">
          <div className="motion-step-title"><strong>1</strong><span>Posicion objetivo XYZ</span></div>
          <div className="motion-field-grid xyz-grid">
            <NumericField label="X" unit="mm" value={goal.x_mm} help="Campo x_mm del goal MoveXYZ." onChange={(value) => update("x_mm", value)} />
            <NumericField label="Y" unit="mm" value={goal.y_mm} help="Campo y_mm del goal MoveXYZ." onChange={(value) => update("y_mm", value)} />
            <NumericField label="Z" unit="mm" value={goal.z_mm} help="Campo z_mm del goal MoveXYZ." onChange={(value) => update("z_mm", value)} />
          </div>
        </section>

        <section className="motion-step">
          <div className="motion-step-title"><strong>2</strong><span>Movimiento lineal</span></div>
          <div className="motion-field-grid linear-grid">
            <NumericField label="Velocidad" unit="mm/s" value={goal.speed_mm_s} min={0} max={2000} help="Campo speed_mm_s del goal. Valores <= 0 usan default del firmware." onChange={(value) => update("speed_mm_s", value)} />
            <NumericField label="Aceleracion" unit="mm/s2" value={goal.accel_mm_s2} min={0} max={1000} help="Campo accel_mm_s2 del goal. Valores <= 0 usan default del firmware." onChange={(value) => update("accel_mm_s2", value)} />
            <NumericField label="Tolerancia" unit="mm" value={goal.tolerance_mm} min={0} help="Campo tolerance_mm usado para confirmar llegada en el firmware." onChange={(value) => update("tolerance_mm", value)} />
            <NumericField label="Timeout" unit="ms" value={goal.timeout_ms} step={1000} min={0} help="Campo timeout_ms del goal MoveXYZ." onChange={(value) => update("timeout_ms", value)} />
          </div>
          <div className="motion-slider-row">
            <label><span>Velocidad</span><input type="range" min="0" max="2000" step="1" value={goal.speed_mm_s} onChange={(event) => update("speed_mm_s", event.target.value)} /></label>
            <label><span>Aceleracion</span><input type="range" min="0" max="1000" step="1" value={goal.accel_mm_s2} onChange={(event) => update("accel_mm_s2", event.target.value)} /></label>
          </div>
        </section>

        <section className="motion-step">
          <div className="motion-step-title"><strong>3</strong><span>Eje A rotatorio</span></div>
          <label className="motion-toggle">
            <input type="checkbox" checked={goal.use_a} onChange={(event) => setUseA(event.target.checked)} />
            <span>Incluir eje A</span>
            <small>Activa los campos use_a, a_deg y tolerancia angular del action.</small>
          </label>
          <div className="motion-field-grid angular-grid">
            <NumericField label="Angulo objetivo" unit="deg" value={goal.a_deg} disabled={!goal.use_a} help="Campo a_deg del goal MoveXYZ." onChange={(value) => update("a_deg", value)} />
            <NumericField label="Velocidad angular" unit="deg/s" value={goal.angular_speed_deg_s} min={0} disabled={!goal.use_a} help="Campo angular_speed_deg_s del goal." onChange={(value) => update("angular_speed_deg_s", value)} />
            <NumericField label="Aceleracion angular" unit="deg/s2" value={goal.angular_accel_deg_s2} min={0} disabled={!goal.use_a} help="Campo angular_accel_deg_s2 del goal." onChange={(value) => update("angular_accel_deg_s2", value)} />
            <NumericField label="Tolerancia angular" unit="deg" value={goal.angular_tolerance_deg} min={0} disabled={!goal.use_a} help="Campo angular_tolerance_deg usado para confirmar llegada del eje A." onChange={(value) => update("angular_tolerance_deg", value)} />
          </div>
        </section>

        <div className="motion-actions">
          <button className="primary" type="submit" disabled={!available}>Enviar setpoint</button>
          <button className="secondary" type="button" onClick={() => setPreviewActive((current) => !current)}>Simular</button>
        </div>
      </form>

      {previewActive && (
        <div className="motion-preview">
          <div><span>Delta XYZ</span><strong>{mm(preview.linearDistance)}</strong></div>
          <div><span>Delta A</span><strong>{deg(preview.da)}</strong></div>
          <div><span>Tiempo estimado</span><strong>{preview.estimatedSeconds.toFixed(2)} s</strong></div>
          <p>Estimacion local basada en objetivo, /palletizer/axis_position_mm y velocidad configurada. No envia comandos ROS.</p>
        </div>
      )}

      <div className="feedback motion-feedback">
        <div>
          <span>Estado</span>
          <strong>{stateText}</strong>
          <small>{moveFeedback ? "feedback.state" : "sin feedback del action"}</small>
        </div>
        <div>
          <span>Progreso</span>
          <strong>{percent(progress)}</strong>
          <div className="progress-track"><span style={{ width: `${progress * 100}%` }} /></div>
        </div>
        <div>
          <span>Error XYZ</span>
          <strong>{mm(Math.max(Math.abs(numberValue(moveFeedback?.error_x_mm)), Math.abs(numberValue(moveFeedback?.error_y_mm)), Math.abs(numberValue(moveFeedback?.error_z_mm))))}</strong>
          <small>max(error_x/y/z)</small>
        </div>
        <div>
          <span>Error eje A</span>
          <strong>{deg(numberValue(moveFeedback?.error_a_deg))}</strong>
          <small>feedback.error_a_deg</small>
        </div>
        <div>
          <span>Posicion actual</span>
          <strong>{mm(currentX)} / {mm(currentY)} / {mm(currentZ)}</strong>
          <small>feedback actual o telemetria</small>
        </div>
        <div>
          <span>Resultado</span>
          <strong className={resultSuccess === true ? "result-ok" : resultSuccess === false ? "result-bad" : ""}>{resultText}</strong>
          <small>{moveResult ? `final A ${deg(numberValue(moveResult.final_a_deg))}` : "ultima respuesta del action"}</small>
        </div>
        <div>
          <span>A actual</span>
          <strong>{deg(currentA)}</strong>
          <small>feedback.current_a_deg o joint_states/status</small>
        </div>
      </div>

      <footer className="motion-footnote">
        <span>Unidades: mm, mm/s, mm/s2, ms, deg, deg/s, deg/s2</span>
        <span>Feedback actualizado desde palletizer_msgs/action/MoveXYZ</span>
      </footer>
    </section>
  );
}

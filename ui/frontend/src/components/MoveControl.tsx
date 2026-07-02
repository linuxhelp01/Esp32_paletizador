import { FormEvent, useState } from "react";
import { percent } from "../lib/format";
import { MoveGoal, PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  lastFeedback: Record<string, unknown> | null;
  lastResult: Record<string, unknown> | null;
  send: (payload: Record<string, unknown>) => void;
};

const initialGoal: MoveGoal = {
  x_mm: 50,
  y_mm: 50,
  z_mm: 20,
  speed_mm_s: 25,
  accel_mm_s2: 50,
  tolerance_mm: 1,
  timeout_ms: 30000
};

export function MoveControl({ state, lastFeedback, lastResult, send }: Props) {
  const [goal, setGoal] = useState<MoveGoal>(initialGoal);
  const available = Boolean(state.availability.move_xyz);

  const update = (field: keyof MoveGoal, value: string) => {
    setGoal((current) => ({ ...current, [field]: field === "timeout_ms" ? Number.parseInt(value || "0", 10) : Number(value) }));
  };

  const submit = (event: FormEvent) => {
    event.preventDefault();
    send({ type: "move_xyz", goal });
  };

  return (
    <section className="section command-surface">
      <div className="section-heading">
        <h2>Movimiento XYZ</h2>
        <span>{available ? "action disponible" : "action no detectada"}</span>
      </div>

      <form className="form-grid" onSubmit={submit}>
        {(["x_mm", "y_mm", "z_mm", "speed_mm_s", "accel_mm_s2", "tolerance_mm", "timeout_ms"] as const).map((field) => (
          <label key={field}>
            <span>{field}</span>
            <input type="number" step={field === "timeout_ms" ? 1000 : 0.1} value={goal[field]} onChange={(event) => update(field, event.target.value)} />
          </label>
        ))}
        <button className="primary" type="submit" disabled={!available}>Enviar setpoint</button>
      </form>

      <div className="feedback">
        <div>
          <span>Estado</span>
          <strong>{String(lastFeedback?.state ?? "sin feedback")}</strong>
        </div>
        <div>
          <span>Progreso</span>
          <strong>{percent(lastFeedback?.progress)}</strong>
        </div>
        <div>
          <span>Resultado</span>
          <strong>{String(lastResult?.message ?? "pendiente")}</strong>
        </div>
      </div>
    </section>
  );
}

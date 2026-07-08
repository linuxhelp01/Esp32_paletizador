import { useState } from "react";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

type AxisLimit = {
  configured: boolean;
  min: number;
  max: number;
};

const FALLBACK_LIMITS: AxisLimit[] = [
  { configured: false, min: -1000, max: 1000 },
  { configured: false, min: -1000, max: 1000 },
  { configured: false, min: -1000, max: 1000 }
];

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

function axisLimitsFromState(state: PalletizerState): AxisLimit[] {
  const limits = state.status?.limits;
  if (!Array.isArray(limits)) return FALLBACK_LIMITS;
  return FALLBACK_LIMITS.map((fallback, index) => {
    const item = limits[index];
    if (!Array.isArray(item) || item.length < 3) return fallback;
    const configured = Boolean(Number(item[0]));
    const min = Number(item[1]);
    const max = Number(item[2]);
    if (!configured || !Number.isFinite(min) || !Number.isFinite(max) || min >= max) return fallback;
    return { configured, min, max };
  });
}

function currentAxis(state: PalletizerState, index: number) {
  const value = Number(state.axis_position_mm[index] ?? 0);
  return Number.isFinite(value) ? value : 0;
}

export function JogControl({ state, send }: Props) {
  const [stepMm, setStepMm] = useState(5);
  const [speedMmS, setSpeedMmS] = useState(25);
  const [accelMmS2, setAccelMmS2] = useState(50);
  const fastJogAvailable = Boolean(state.availability.fast_jog);
  const relativeJogAvailable = Boolean(state.availability.jog_delta_topic);
  const available = Boolean(fastJogAvailable || state.availability.move_xyz);
  const gripperAvailable = Boolean(state.availability.set_gripper || state.availability.aux_servo || state.availability.command_topic);
  const limits = axisLimitsFromState(state);

  const jog = (dxMm: number, dyMm: number, dzMm: number) => {
    if (fastJogAvailable) {
      send({
        type: "jog_xyz_delta",
        goal: {
          dx_mm: dxMm,
          dy_mm: dyMm,
          dz_mm: dzMm,
          speed_mm_s: speedMmS,
          accel_mm_s2: accelMmS2
        }
      });
      return;
    }

    const nextX = clamp(currentAxis(state, 0) + dxMm, limits[0].min, limits[0].max);
    const nextY = clamp(currentAxis(state, 1) + dyMm, limits[1].min, limits[1].max);
    const nextZ = clamp(currentAxis(state, 2) + dzMm, limits[2].min, limits[2].max);

    send({
      type: "move_xyz",
      goal: {
        x_mm: nextX,
        y_mm: nextY,
        z_mm: nextZ,
        use_a: false,
        speed_mm_s: speedMmS,
        accel_mm_s2: accelMmS2,
        tolerance_mm: 1,
        timeout_ms: 30000
      }
    });
  };

  const move = (dx: number, dy: number, dz: number) => {
    jog(dx * stepMm, dy * stepMm, dz * stepMm);
  };

  const setGripper = (closed: boolean) => {
    send({ type: "set_gripper", request: { closed } });
  };

  return (
    <section className="section jog-control">
      <div className="section-heading">
        <h2>Controles manuales</h2>
        <span>{relativeJogAvailable ? "jog relativo ESP32" : fastJogAvailable ? "jog rapido" : available ? "jog por action" : "sin jog"}</span>
      </div>

      <div className="jog-layout">
        <div className="xy-pad" aria-label="Control discreto del plano XY">
          <button className="jog-button y-plus" type="button" disabled={!available} aria-label="Mover Y positivo" onClick={() => move(0, 1, 0)}>&uarr;</button>
          <button className="jog-button x-minus" type="button" disabled={!available} aria-label="Mover X negativo" onClick={() => move(-1, 0, 0)}>&larr;</button>
          <div className="jog-center">XY</div>
          <button className="jog-button x-plus" type="button" disabled={!available} aria-label="Mover X positivo" onClick={() => move(1, 0, 0)}>&rarr;</button>
          <button className="jog-button y-minus" type="button" disabled={!available} aria-label="Mover Y negativo" onClick={() => move(0, -1, 0)}>&darr;</button>
        </div>

        <div className="z-pad" aria-label="Control discreto del eje Z">
          <button className="jog-button" type="button" disabled={!available} aria-label="Mover Z arriba" onClick={() => move(0, 0, 1)}>Z+ &uarr;</button>
          <button className="jog-button" type="button" disabled={!available} aria-label="Mover Z abajo" onClick={() => move(0, 0, -1)}>Z- &darr;</button>
        </div>
      </div>

      <div className="jog-speed">
        <div className="range-row"><span>Velocidad de jog</span><strong>{speedMmS.toFixed(0)} mm/s</strong></div>
        <input type="range" min="1" max="2000" step="1" value={speedMmS} onChange={(event) => setSpeedMmS(Number(event.target.value))} />
        <div className="range-scale"><span>1</span><span>2000</span></div>
      </div>
      <div className="inline-controls jog-settings">
        <label>
          <span>Paso mm</span>
          <input type="number" min="0.1" step="0.1" value={stepMm} onChange={(event) => setStepMm(Number(event.target.value))} />
        </label>
        <label>
          <span>Acc mm/s2</span>
          <input type="number" min="0.1" step="0.1" value={accelMmS2} onChange={(event) => setAccelMmS2(Number(event.target.value))} />
        </label>
      </div>

      <div className="manual-gripper">
        <div className="range-row"><span>Pinza servo</span><strong>{gripperAvailable ? "lista" : "sin servicio"}</strong></div>
        <div className="button-row gripper-actions">
          <button className="secondary" type="button" disabled={!gripperAvailable} onClick={() => setGripper(false)}>Abrir</button>
          <button className="primary" type="button" disabled={!gripperAvailable} onClick={() => setGripper(true)}>Cerrar</button>
        </div>
      </div>
    </section>
  );
}

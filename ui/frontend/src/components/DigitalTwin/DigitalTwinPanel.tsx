import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import { FormEvent, useEffect, useMemo, useState } from "react";
import { MoveGoal, PalletizerState } from "../../lib/types";
import { mm } from "../../lib/format";

type Target = Pick<MoveGoal, "x_mm" | "y_mm" | "z_mm">;

type AxisLimit = {
  configured: boolean;
  min: number;
  max: number;
};

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

const FALLBACK_LIMITS: AxisLimit[] = [
  { configured: false, min: 0, max: 300 },
  { configured: false, min: 0, max: 300 },
  { configured: false, min: 0, max: 250 }
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
    if (!Number.isFinite(min) || !Number.isFinite(max) || min >= max) return fallback;
    return { configured, min, max };
  });
}

function targetFromState(state: PalletizerState): Target {
  return {
    x_mm: Number(state.axis_position_mm[0] ?? 0),
    y_mm: Number(state.axis_position_mm[1] ?? 0),
    z_mm: Number(state.axis_position_mm[2] ?? 0)
  };
}

function RobotTemplate({
  current,
  target,
  limits,
  targetEditing,
  onTargetSelect
}: {
  current: Target;
  target: Target;
  limits: AxisLimit[];
  targetEditing: boolean;
  onTargetSelect: () => void;
}) {
  const xRange = limits[0].max - limits[0].min;
  const yRange = limits[1].max - limits[1].min;
  const zRange = limits[2].max - limits[2].min;
  const railColor = "#66717f";

  return (
    <group>
      <mesh position={[limits[0].min + xRange / 2, -4, limits[1].min + yRange / 2]}>
        <boxGeometry args={[xRange, 8, 8]} />
        <meshStandardMaterial color={railColor} />
      </mesh>
      <mesh position={[limits[0].min + xRange / 2, -4, limits[1].max]}>
        <boxGeometry args={[xRange, 8, 8]} />
        <meshStandardMaterial color={railColor} />
      </mesh>
      <mesh position={[current.x_mm, 8, limits[1].min + yRange / 2]}>
        <boxGeometry args={[18, 16, yRange]} />
        <meshStandardMaterial color="#2d7d86" />
      </mesh>
      <mesh position={[current.x_mm, current.z_mm / 2, current.y_mm]}>
        <boxGeometry args={[20, Math.max(current.z_mm, 1), 20]} />
        <meshStandardMaterial color="#394756" />
      </mesh>
      <mesh position={[current.x_mm, current.z_mm, current.y_mm]}>
        <sphereGeometry args={[10, 24, 24]} />
        <meshStandardMaterial color="#187f45" />
      </mesh>
      <mesh
        position={[target.x_mm, target.z_mm, target.y_mm]}
        scale={targetEditing ? 1.35 : 1}
        onPointerDown={(event: ThreeEvent<PointerEvent>) => {
          event.stopPropagation();
          onTargetSelect();
        }}
      >
        <sphereGeometry args={[9, 24, 24]} />
        <meshStandardMaterial color="#d59a1a" emissive={targetEditing ? "#d59a1a" : "#6d4600"} emissiveIntensity={targetEditing ? 0.55 : 0.25} />
      </mesh>
      <mesh position={[target.x_mm, target.z_mm / 2, target.y_mm]}>
        <cylinderGeometry args={[2, 2, Math.max(target.z_mm, 1), 12]} />
        <meshStandardMaterial color="#d59a1a" transparent opacity={0.65} />
      </mesh>
      <mesh position={[limits[0].min + xRange / 2, limits[2].min + zRange / 2, limits[1].min + yRange / 2]}>
        <boxGeometry args={[xRange, zRange, yRange]} />
        <meshBasicMaterial color="#3b86a0" wireframe transparent opacity={0.28} />
      </mesh>
    </group>
  );
}

function DragPlane({ target, limits, onTargetChange }: { target: Target; limits: AxisLimit[]; onTargetChange: (next: Target) => void }) {
  const [dragging, setDragging] = useState(false);
  const xRange = limits[0].max - limits[0].min;
  const yRange = limits[1].max - limits[1].min;
  const centerX = limits[0].min + xRange / 2;
  const centerY = limits[1].min + yRange / 2;

  const update = (event: ThreeEvent<PointerEvent>) => {
    event.stopPropagation();
    onTargetChange({
      ...target,
      x_mm: clamp(event.point.x, limits[0].min, limits[0].max),
      y_mm: clamp(event.point.z, limits[1].min, limits[1].max)
    });
  };

  return (
    <mesh
      position={[centerX, 0, centerY]}
      rotation={[-Math.PI / 2, 0, 0]}
      onPointerDown={(event: ThreeEvent<PointerEvent>) => {
        setDragging(true);
        update(event);
      }}
      onPointerMove={(event: ThreeEvent<PointerEvent>) => {
        if (dragging) update(event);
      }}
      onPointerUp={() => setDragging(false)}
      onPointerLeave={() => setDragging(false)}
    >
      <planeGeometry args={[xRange, yRange]} />
      <meshBasicMaterial color="#e9eef3" transparent opacity={0.3} />
    </mesh>
  );
}

function Scene({
  current,
  target,
  limits,
  targetEditing,
  onTargetSelect,
  onTargetChange
}: {
  current: Target;
  target: Target;
  limits: AxisLimit[];
  targetEditing: boolean;
  onTargetSelect: () => void;
  onTargetChange: (next: Target) => void;
}) {
  const xRange = limits[0].max - limits[0].min;
  const yRange = limits[1].max - limits[1].min;
  const maxRange = Math.max(xRange, yRange, 250);
  const centerX = limits[0].min + xRange / 2;
  const centerY = limits[1].min + yRange / 2;

  return (
    <Canvas camera={{ position: [centerX + maxRange * 0.9, maxRange * 0.85, centerY + maxRange * 0.95], fov: 42 }}>
      <color attach="background" args={["#f6f8fa"]} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[250, 400, 200]} intensity={1.0} />
      <gridHelper args={[maxRange * 1.3, 24, "#9aa5b1", "#d4dae1"]} position={[centerX, -0.5, centerY]} />
      <axesHelper args={[80]} position={[limits[0].min, 0, limits[1].min]} />
      {targetEditing && <DragPlane target={target} limits={limits} onTargetChange={onTargetChange} />}
      <RobotTemplate current={current} target={target} limits={limits} targetEditing={targetEditing} onTargetSelect={onTargetSelect} />
      <OrbitControls makeDefault enabled={!targetEditing} target={[centerX, 60, centerY]} />
    </Canvas>
  );
}

export function DigitalTwinPanel({ state, send }: Props) {
  const available = Boolean(state.availability.move_xyz);
  const limits = useMemo(() => axisLimitsFromState(state), [state]);
  const current = targetFromState(state);
  const [target, setTarget] = useState<Target>(current);
  const [targetEditing, setTargetEditing] = useState(false);
  const [speedMmS, setSpeedMmS] = useState(25);
  const [accelMmS2, setAccelMmS2] = useState(50);
  const [toleranceMm, setToleranceMm] = useState(1);
  const [timeoutMs, setTimeoutMs] = useState(30000);

  useEffect(() => {
    if (state.stamp_ms && target.x_mm === 0 && target.y_mm === 0 && target.z_mm === 0) {
      setTarget(targetFromState(state));
    }
  }, [state.stamp_ms]);

  const updateTargetField = (field: keyof Target, value: string) => {
    const index = field === "x_mm" ? 0 : field === "y_mm" ? 1 : 2;
    const numeric = Number(value);
    setTarget((currentTarget) => ({
      ...currentTarget,
      [field]: clamp(Number.isFinite(numeric) ? numeric : 0, limits[index].min, limits[index].max)
    }));
  };

  const submit = (event: FormEvent) => {
    event.preventDefault();
    send({
      type: "move_xyz",
      goal: {
        ...target,
        speed_mm_s: speedMmS,
        accel_mm_s2: accelMmS2,
        tolerance_mm: toleranceMm,
        timeout_ms: timeoutMs
      }
    });
    setTargetEditing(false);
  };

  return (
    <section className="section twin-section">
      <div className="section-heading">
        <h2>Gemelo digital</h2>
        <span>{available ? "control 3D disponible" : "esperando action"}</span>
      </div>
      <div className="twin-layout">
        <div className="twin-canvas" aria-label="Visualizacion 3D del paletizador">
          <Scene
            current={current}
            target={target}
            limits={limits}
            targetEditing={targetEditing}
            onTargetSelect={() => setTargetEditing(true)}
            onTargetChange={setTarget}
          />
        </div>
        <form className="twin-controls" onSubmit={submit}>
          <div className="twin-mode-row">
            <span className={targetEditing ? "edit-badge active" : "edit-badge"}>{targetEditing ? "Editando setpoint" : "Orbita camara"}</span>
            <button className="secondary" type="button" onClick={() => setTargetEditing((currentMode) => !currentMode)}>
              {targetEditing ? "Bloquear setpoint" : "Editar setpoint"}
            </button>
          </div>
          <p className="twin-hint">Click en la esfera amarilla habilita el arrastre del setpoint. Fuera de ese modo, el mouse orienta la vista 3D.</p>
          <div className="twin-readout">
            <div><span>Actual X</span><strong>{mm(current.x_mm)}</strong></div>
            <div><span>Actual Y</span><strong>{mm(current.y_mm)}</strong></div>
            <div><span>Actual Z</span><strong>{mm(current.z_mm)}</strong></div>
          </div>
          <div className="form-grid compact">
            <label><span>Target X</span><input type="number" step="0.1" value={target.x_mm} onChange={(event) => updateTargetField("x_mm", event.target.value)} /></label>
            <label><span>Target Y</span><input type="number" step="0.1" value={target.y_mm} onChange={(event) => updateTargetField("y_mm", event.target.value)} /></label>
            <label><span>Target Z</span><input type="number" step="0.1" value={target.z_mm} onChange={(event) => updateTargetField("z_mm", event.target.value)} /></label>
            <label><span>Velocidad</span><input type="number" step="0.1" value={speedMmS} onChange={(event) => setSpeedMmS(Number(event.target.value))} /></label>
            <label><span>Aceleracion</span><input type="number" step="0.1" value={accelMmS2} onChange={(event) => setAccelMmS2(Number(event.target.value))} /></label>
            <label><span>Tolerancia</span><input type="number" step="0.1" value={toleranceMm} onChange={(event) => setToleranceMm(Number(event.target.value))} /></label>
            <label><span>Timeout ms</span><input type="number" step="1000" value={timeoutMs} onChange={(event) => setTimeoutMs(Number.parseInt(event.target.value || "0", 10))} /></label>
          </div>
          <button className="primary" type="submit" disabled={!available}>Enviar target 3D</button>
        </form>
      </div>
    </section>
  );
}

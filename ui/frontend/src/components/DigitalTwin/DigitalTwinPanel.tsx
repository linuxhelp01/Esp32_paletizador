import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { useMemo } from "react";
import { PalletizerState } from "../../lib/types";

type Target = {
  x_mm: number;
  y_mm: number;
  z_mm: number;
};

type AxisLimit = {
  configured: boolean;
  min: number;
  max: number;
};

type Props = {
  state: PalletizerState;
};

const FALLBACK_LIMITS: AxisLimit[] = [
  { configured: false, min: 0, max: 300 },
  { configured: false, min: 0, max: 300 },
  { configured: false, min: 0, max: 250 }
];

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
  limits
}: {
  current: Target;
  limits: AxisLimit[];
}) {
  const xRange = limits[0].max - limits[0].min;
  const yRange = limits[1].max - limits[1].min;
  const zRange = limits[2].max - limits[2].min;
  const railColor = "#7f8996";

  return (
    <group>
      <mesh position={[limits[0].min + xRange / 2, -4, limits[1].min + yRange / 2]}>
        <boxGeometry args={[xRange, 8, 8]} />
        <meshStandardMaterial color={railColor} metalness={0.35} roughness={0.38} />
      </mesh>
      <mesh position={[limits[0].min + xRange / 2, -4, limits[1].max]}>
        <boxGeometry args={[xRange, 8, 8]} />
        <meshStandardMaterial color={railColor} metalness={0.35} roughness={0.38} />
      </mesh>
      <mesh position={[current.x_mm, 8, limits[1].min + yRange / 2]}>
        <boxGeometry args={[18, 16, yRange]} />
        <meshStandardMaterial color="#6f7f8f" metalness={0.25} roughness={0.4} />
      </mesh>
      <mesh position={[current.x_mm, current.z_mm / 2, current.y_mm]}>
        <boxGeometry args={[20, Math.max(current.z_mm, 1), 20]} />
        <meshStandardMaterial color="#c8d0da" metalness={0.35} roughness={0.35} />
      </mesh>
      <mesh position={[current.x_mm, current.z_mm, current.y_mm]}>
        <sphereGeometry args={[10, 24, 24]} />
        <meshStandardMaterial color="#2fe36e" emissive="#0a5c24" emissiveIntensity={0.35} />
      </mesh>
      <mesh position={[limits[0].min + xRange / 2, limits[2].min + zRange / 2, limits[1].min + yRange / 2]}>
        <boxGeometry args={[xRange, zRange, yRange]} />
        <meshBasicMaterial color="#2f9bff" wireframe transparent opacity={0.22} />
      </mesh>
    </group>
  );
}

function Scene({
  current,
  limits
}: {
  current: Target;
  limits: AxisLimit[];
}) {
  const xRange = limits[0].max - limits[0].min;
  const yRange = limits[1].max - limits[1].min;
  const maxRange = Math.max(xRange, yRange, 250);
  const centerX = limits[0].min + xRange / 2;
  const centerY = limits[1].min + yRange / 2;

  return (
    <Canvas camera={{ position: [centerX + maxRange * 0.9, maxRange * 0.85, centerY + maxRange * 0.95], fov: 42 }}>
      <color attach="background" args={["#07111c"]} />
      <ambientLight intensity={0.55} />
      <directionalLight position={[250, 400, 200]} intensity={1.25} />
      <gridHelper args={[maxRange * 1.3, 24, "#25699c", "#182a3a"]} position={[centerX, -0.5, centerY]} />
      <axesHelper args={[80]} position={[limits[0].min, 0, limits[1].min]} />
      <RobotTemplate current={current} limits={limits} />
      <OrbitControls makeDefault target={[centerX, 60, centerY]} />
    </Canvas>
  );
}

export function DigitalTwinPanel({ state }: Props) {
  const limits = useMemo(() => axisLimitsFromState(state), [state]);
  const current = targetFromState(state);

  return (
    <section className="section twin-section">
      <div className="section-heading">
        <h2>Gemelo digital</h2>
        <span>visualizacion por telemetria</span>
      </div>
      <div className="twin-layout">
        <div className="twin-canvas" aria-label="Visualizacion 3D del paletizador">
          <Scene
            current={current}
            limits={limits}
          />
        </div>
      </div>
    </section>
  );
}

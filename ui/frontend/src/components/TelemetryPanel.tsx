import { deg, mm, rpm } from "../lib/format";
import { PalletizerState } from "../lib/types";

const axisLabels = ["X", "Y", "Z"];
const motorLabels = ["X1", "X2", "Y", "Z", "A"];

function numberArray(value: unknown): number[] {
  return Array.isArray(value) ? value.map((item) => Number(item ?? 0)) : [];
}

function textArray(value: unknown): string[] {
  return Array.isArray(value) ? value.map((item) => String(item ?? "")) : [];
}

type Props = {
  state: PalletizerState;
};

export function TelemetryPanel({ state }: Props) {
  const status = state.status;
  const positionMm = numberArray(status.mm);
  const velocityMmS = numberArray(status.vel_mm_s);
  const axisVelocityMmS = numberArray(status.axis_velocity_mm_s);
  const axisAccelMmS2 = numberArray(status.axis_accel_mm_s2);
  const derivedAccel = numberArray(status.derived_accel);
  const lastAcc = numberArray(status.lastAcc);
  const online = numberArray(status.online);
  const enabled = numberArray(status.enabled);
  const units = textArray(status.units);
  const accelUnits = textArray(status.derived_accel_units);

  return (
    <section className="section">
      <div className="section-heading">
        <h2>Telemetria</h2>
        <span>{state.connections.axis_position_mm ? "posicion activa" : "sin posicion"}</span>
      </div>

      <div className="metric-grid three">
        {axisLabels.map((axis, index) => (
          <div className="metric" key={axis}>
            <span>{axis}</span>
            <strong>{mm(state.axis_position_mm[index])}</strong>
            <small>Vel {mm(axisVelocityMmS[index])}/s</small>
            <small>Acc {Number(axisAccelMmS2[index] ?? 0).toFixed(1)} mm/s2</small>
          </div>
        ))}
      </div>

      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Motor</th>
              <th>Posicion</th>
              <th>Velocidad</th>
              <th>Acel. derivada</th>
              <th>Acc cmd</th>
              <th>RPM</th>
              <th>Online</th>
              <th>Enable</th>
            </tr>
          </thead>
          <tbody>
            {motorLabels.map((motor, index) => (
              <tr key={motor}>
                <td>{motor}</td>
                <td>{units[index] === "deg" ? deg(positionMm[index]) : mm(positionMm[index])}</td>
                <td>{units[index] === "deg" ? `${deg(velocityMmS[index])}/s` : `${mm(velocityMmS[index])}/s`}</td>
                <td>{Number(derivedAccel[index] ?? 0).toFixed(1)} {accelUnits[index] || (units[index] === "deg" ? "deg/s2" : "mm/s2")}</td>
                <td>{Number(lastAcc[index] ?? 0).toFixed(0)}</td>
                <td>{rpm(state.motor_rpm[index])}</td>
                <td>{online[index] ? "OK" : "N/D"}</td>
                <td>{enabled[index] ? "ON" : "OFF"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
}

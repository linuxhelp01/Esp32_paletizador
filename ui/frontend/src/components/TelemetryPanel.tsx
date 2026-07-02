import { mm, rpm } from "../lib/format";
import { PalletizerState } from "../lib/types";

const axisLabels = ["X", "Y", "Z"];
const motorLabels = ["X1", "X2", "Y", "Z"];

type Props = {
  state: PalletizerState;
};

export function TelemetryPanel({ state }: Props) {
  const status = state.status;
  const positionMm = Array.isArray(status.mm) ? status.mm as number[] : [];
  const velocityMmS = Array.isArray(status.vel_mm_s) ? status.vel_mm_s as number[] : [];
  const online = Array.isArray(status.online) ? status.online as number[] : [];
  const enabled = Array.isArray(status.enabled) ? status.enabled as number[] : [];

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
              <th>RPM</th>
              <th>Online</th>
              <th>Enable</th>
            </tr>
          </thead>
          <tbody>
            {motorLabels.map((motor, index) => (
              <tr key={motor}>
                <td>{motor}</td>
                <td>{mm(positionMm[index])}</td>
                <td>{mm(velocityMmS[index])}/s</td>
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

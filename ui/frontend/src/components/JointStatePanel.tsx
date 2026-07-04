import { meters } from "../lib/format";
import { PalletizerState } from "../lib/types";

type Props = {
  state: PalletizerState;
};

export function JointStatePanel({ state }: Props) {
  const names = state.joint_states.name.length ? state.joint_states.name : ["X1", "X2", "Y", "Z", "A"];
  return (
    <section className="section">
      <div className="section-heading">
        <h2>Joint states</h2>
        <span>{state.connections.joint_states ? "ROS activo" : "sin datos"}</span>
      </div>
      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Joint</th>
              <th>Posicion</th>
              <th>Velocidad</th>
            </tr>
          </thead>
          <tbody>
            {names.map((name, index) => (
              <tr key={name}>
                <td>{name}</td>
                <td>{meters(state.joint_states.position[index])}</td>
                <td>{meters(state.joint_states.velocity[index])}/s</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
}

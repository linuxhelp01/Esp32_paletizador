import { PalletizerState } from "../lib/types";
import { MOTOR_COUNT, machineReadiness } from "../lib/readiness";

type Props = {
  state: PalletizerState;
};

export function SystemSummary({ state }: Props) {
  const readiness = machineReadiness(state);

  return (
    <section className="section system-summary">
      <div className="section-heading">
        <h2>Resumen del sistema</h2>
        <span>{readiness.operative ? "operativo" : "no operativo"}</span>
      </div>
      <div className="summary-list">
        <div><span>Topico status</span><strong>{readiness.statusFresh ? "OK" : "N/D"}</strong></div>
        <div><span>Topicos motor</span><strong>{readiness.motorStateTopicsFresh ? "OK" : "N/D"}</strong></div>
        <div><span>Servos conectados</span><strong>{readiness.onlineCount}/{MOTOR_COUNT}</strong></div>
        <div><span>Enable valido</span><strong>{readiness.enableValidCount}/{MOTOR_COUNT}</strong></div>
        <div><span>Servos habilitados</span><strong>{readiness.enabledCount}/{MOTOR_COUNT}</strong></div>
        <div><span>Alarmas activas</span><strong>{readiness.hasFault || readiness.stalledCount ? readiness.stalledCount || 1 : 0}</strong></div>
        <div><span>Modo de operacion</span><strong>Manual</strong></div>
        <div><span>Comunicacion</span><strong>{readiness.rosCommunication ? "OK" : "N/D"}</strong></div>
      </div>
    </section>
  );
}

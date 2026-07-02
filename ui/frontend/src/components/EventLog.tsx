import { BackendEvent } from "../lib/types";

type Props = {
  events: BackendEvent[];
};

export function EventLog({ events }: Props) {
  return (
    <section className="section log-section">
      <div className="section-heading">
        <h2>Eventos</h2>
        <span>{events.length}</span>
      </div>
      <div className="event-log">
        {events.map((event, index) => (
          <pre key={`${event.type}-${index}`}>{JSON.stringify(event, null, 2)}</pre>
        ))}
      </div>
    </section>
  );
}

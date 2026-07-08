import { EventLog } from "./components/EventLog";
import { DigitalTwinPanel } from "./components/DigitalTwin/DigitalTwinPanel";
import { HomingControls } from "./components/HomingControls";
import { JointStatePanel } from "./components/JointStatePanel";
import { JogControl } from "./components/JogControl";
import { MotorStateDrawer } from "./components/MotorStateDrawer";
import { MoveControl } from "./components/MoveControl";
import { NodeInterfacesPanel } from "./components/NodeInterfacesPanel";
import { SafetyPanel } from "./components/SafetyPanel";
import { ServiceControls } from "./components/ServiceControls";
import { StatusBar } from "./components/StatusBar";
import { SystemSummary } from "./components/SystemSummary";
import { TelemetryPanel } from "./components/TelemetryPanel";
import { usePalletizerSocket } from "./hooks/usePalletizerSocket";

export function App() {
  const { connected, state, events, lastFeedback, lastResult, lastPing, send } = usePalletizerSocket();

  return (
    <main className="app">
      <StatusBar connected={connected} state={state} lastPing={lastPing} send={send} />
      <div className="operator-grid">
        <aside className="left-rail">
          <SafetyPanel state={state} send={send} />
          <SystemSummary state={state} />
        </aside>
        <section className="center-stage">
          <DigitalTwinPanel state={state} />
          <TelemetryPanel state={state} />
        </section>
        <aside className="right-rail">
          <JogControl state={state} send={send} />
          <MoveControl state={state} lastFeedback={lastFeedback} lastResult={lastResult} send={send} />
          <HomingControls state={state} lastFeedback={lastFeedback} lastResult={lastResult} send={send} />
          <ServiceControls state={state} send={send} />
        </aside>
        <section className="bottom-rail">
          <EventLog events={events} />
          <NodeInterfacesPanel state={state} lastFeedback={lastFeedback} lastResult={lastResult} send={send} />
          <JointStatePanel state={state} />
        </section>
      </div>
      <MotorStateDrawer state={state} send={send} />
    </main>
  );
}

import { EventLog } from "./components/EventLog";
import { DigitalTwinPanel } from "./components/DigitalTwin/DigitalTwinPanel";
import { HomingControls } from "./components/HomingControls";
import { JointStatePanel } from "./components/JointStatePanel";
import { MotorStateDrawer } from "./components/MotorStateDrawer";
import { MoveControl } from "./components/MoveControl";
import { SafetyPanel } from "./components/SafetyPanel";
import { ServiceControls } from "./components/ServiceControls";
import { StatusBar } from "./components/StatusBar";
import { TelemetryPanel } from "./components/TelemetryPanel";
import { usePalletizerSocket } from "./hooks/usePalletizerSocket";

export function App() {
  const { connected, state, events, lastFeedback, lastResult, send } = usePalletizerSocket();

  return (
    <main className="app">
      <StatusBar connected={connected} state={state} />
      <div className="layout">
        <div className="main-column">
          <TelemetryPanel state={state} />
          <DigitalTwinPanel state={state} send={send} />
          <MoveControl state={state} lastFeedback={lastFeedback} lastResult={lastResult} send={send} />
          <JointStatePanel state={state} />
        </div>
        <aside className="side-column">
          <SafetyPanel state={state} send={send} />
          <ServiceControls state={state} send={send} />
          <HomingControls state={state} send={send} />
          <EventLog events={events} />
        </aside>
      </div>
      <MotorStateDrawer state={state} />
    </main>
  );
}

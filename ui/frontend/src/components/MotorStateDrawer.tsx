import { useState } from "react";
import { asBoolText, deg, mm, rpm } from "../lib/format";
import { PalletizerState } from "../lib/types";

const motorLabels = ["X1", "X2", "Y", "Z", "A"];
const motorEnableAxes = [5, 6, 1, 2, 4];

function numberArray(value: unknown): number[] {
  return Array.isArray(value) ? value.map((item) => Number(item ?? 0)) : [];
}

function valueArray(value: unknown): unknown[] {
  return Array.isArray(value) ? value : [];
}

function statusText(value: unknown, okText = "OK", badText = "N/D") {
  return value ? okText : badText;
}

type Props = {
  state: PalletizerState;
  send: (payload: Record<string, unknown>) => void;
};

export function MotorStateDrawer({ state, send }: Props) {
  const [open, setOpen] = useState(false);
  const status = state.status;
  const online = valueArray(status.online);
  const enabledOk = valueArray(status.enabledOk);
  const enabled = valueArray(status.enabled);
  const stalled = valueArray(status.stalled);
  const positionMm = numberArray(status.mm);
  const velocityMmS = numberArray(status.vel_mm_s);
  const statusRpm = numberArray(status.rpm);
  const enc31 = numberArray(status.enc31);
  const raw35 = numberArray(status.raw35);
  const angleError = numberArray(status.angleError);
  const moveStatus = valueArray(status.moveStatus);
  const home91 = valueArray(status.home91);
  const home3B = valueArray(status.home3B);
  const units = valueArray(status.units);
  const canEnable = Boolean(state.availability.enable_axis || state.availability.command_topic);

  const setMotorEnable = (axis: number, enable: boolean) => {
    send({ type: "enable_axis", request: { axis, enable } });
  };

  return (
    <>
      <button className="motor-drawer-toggle" type="button" onClick={() => setOpen(true)}>
        Estados motores
      </button>
      {open && <button className="drawer-backdrop" type="button" aria-label="Cerrar estados de motores" onClick={() => setOpen(false)} />}
      <aside className={`motor-drawer ${open ? "open" : ""}`} aria-hidden={!open}>
        <div className="drawer-header">
          <div>
            <h2>Motores</h2>
            <p>Telemetria y estado por eje desde el ESP32-S3</p>
          </div>
          <button type="button" className="secondary" onClick={() => setOpen(false)}>Cerrar</button>
        </div>

        <div className="motor-state-grid">
          {motorLabels.map((motor, index) => {
            const rpmValue = Number.isFinite(statusRpm[index]) ? statusRpm[index] : state.motor_rpm[index];
            return (
              <section className="motor-state-row" key={motor}>
                <div className="motor-state-title">
                  <strong>{motor}</strong>
                  <span className={enabled[index] ? "state-dot on" : "state-dot"}>{enabled[index] ? "ON" : "OFF"}</span>
                </div>
                <div className="motor-action-row">
                  <button
                    className="secondary"
                    type="button"
                    disabled={!canEnable || Boolean(enabled[index])}
                    onClick={() => setMotorEnable(motorEnableAxes[index], true)}
                  >
                    Habilitar
                  </button>
                  <button
                    className="secondary"
                    type="button"
                    disabled={!canEnable || !Boolean(enabled[index])}
                    onClick={() => setMotorEnable(motorEnableAxes[index], false)}
                  >
                    Eje libre
                  </button>
                </div>
                <dl>
                  <div><dt>Online</dt><dd>{statusText(online[index])}</dd></div>
                  <div><dt>Enable valido</dt><dd>{asBoolText(enabledOk[index])}</dd></div>
                  <div><dt>Stall</dt><dd>{stalled[index] ? "SI" : "No"}</dd></div>
                  <div><dt>Posicion</dt><dd>{units[index] === "deg" ? deg(positionMm[index]) : mm(positionMm[index])}</dd></div>
                  <div><dt>Velocidad</dt><dd>{units[index] === "deg" ? `${deg(velocityMmS[index])}/s` : `${mm(velocityMmS[index])}/s`}</dd></div>
                  <div><dt>RPM</dt><dd>{rpm(rpmValue)}</dd></div>
                  <div><dt>Encoder 0x31</dt><dd>{Number(enc31[index] ?? 0).toFixed(0)}</dd></div>
                  <div><dt>Encoder 0x35</dt><dd>{Number(raw35[index] ?? 0).toFixed(0)}</dd></div>
                  <div><dt>Error angular</dt><dd>{Number(angleError[index] ?? 0).toFixed(3)}</dd></div>
                  <div><dt>Movimiento</dt><dd>{String(moveStatus[index] ?? "N/D")}</dd></div>
                  <div><dt>Home 0x91</dt><dd>{String(home91[index] ?? "N/D")}</dd></div>
                  <div><dt>Home 0x3B</dt><dd>{String(home3B[index] ?? "N/D")}</dd></div>
                </dl>
              </section>
            );
          })}
        </div>
      </aside>
    </>
  );
}

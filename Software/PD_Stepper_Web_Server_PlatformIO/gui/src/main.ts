import { SerialConnection } from './serial';
import { moveCommand, setCurrentCommand, type Packet } from './protocol';

const conn = new SerialConnection();

// ── DOM references ──────────────────────────────────────────────────────────

const btnConnect     = document.getElementById('btn-connect')       as HTMLButtonElement;
const connDot        = document.getElementById('conn-dot')          as HTMLSpanElement;
const connStatus     = document.getElementById('conn-status')       as HTMLSpanElement;
const moveForm       = document.getElementById('move-form')         as HTMLFormElement;
const btnMove        = document.getElementById('btn-move')          as HTMLButtonElement;
const inpDistance    = document.getElementById('inp-distance')      as HTMLInputElement;
const inpSpeed       = document.getElementById('inp-speed')         as HTMLInputElement;
const inpAccel       = document.getElementById('inp-accel')         as HTMLInputElement;
const chkAbs         = document.getElementById('chk-abs')           as HTMLInputElement;
const sliderCurrent  = document.getElementById('slider-current')    as HTMLInputElement;
const currentVal     = document.getElementById('current-val')       as HTMLSpanElement;
const btnApplyCurrent = document.getElementById('btn-apply-current') as HTMLButtonElement;
const teleMeas       = document.getElementById('tele-meas')         as HTMLSpanElement;
const teleVel        = document.getElementById('tele-vel')          as HTMLSpanElement;
const teleLag        = document.getElementById('tele-lag')          as HTMLSpanElement;
const teleStatus     = document.getElementById('tele-status')       as HTMLDivElement;
const stopInfo       = document.getElementById('stop-info')         as HTMLDivElement;

// ── Initial state ────────────────────────────────────────────────────────────

setControlsEnabled(false);

function setControlsEnabled(on: boolean): void {
  btnMove.disabled          = !on;
  btnApplyCurrent.disabled  = !on;
  sliderCurrent.disabled    = !on;
  inpDistance.disabled      = !on;
  inpSpeed.disabled         = !on;
  inpAccel.disabled         = !on;
  chkAbs.disabled           = !on;
}

function setMotionStatus(moving: boolean): void {
  teleStatus.textContent = moving ? '● MOVING' : '● STOPPED';
  teleStatus.className   = `status ${moving ? 'moving' : 'stopped'}`;
}

// ── Connection ───────────────────────────────────────────────────────────────

conn.onConnectionChange = (connected: boolean): void => {
  connDot.className     = `dot ${connected ? 'connected' : 'disconnected'}`;
  connStatus.textContent = connected ? 'Connected' : 'Disconnected';
  btnConnect.textContent = connected ? 'Disconnect' : 'Connect';
  btnConnect.classList.toggle('connected', connected);
  setControlsEnabled(connected);
  if (!connected) setMotionStatus(false);
};

btnConnect.addEventListener('click', () => {
  btnConnect.disabled = true;
  const action = conn.isConnected ? conn.disconnect() : conn.connect();
  action
    .catch((err: unknown) => {
      alert(`Connection error: ${err instanceof Error ? err.message : String(err)}`);
    })
    .finally(() => { btnConnect.disabled = false; });
});

// ── Move command ─────────────────────────────────────────────────────────────

moveForm.addEventListener('submit', (e: Event) => {
  e.preventDefault();
  stopInfo.classList.add('hidden');
  setMotionStatus(true);
  const cmd = moveCommand(
    parseInt(inpDistance.value, 10),
    parseFloat(inpSpeed.value),
    parseFloat(inpAccel.value),
    chkAbs.checked,
  );
  conn.write(cmd).catch((err: unknown) => {
    setMotionStatus(false);
    alert(`Send error: ${err instanceof Error ? err.message : String(err)}`);
  });
});

// ── Current setting ──────────────────────────────────────────────────────────

sliderCurrent.addEventListener('input', () => {
  currentVal.textContent = sliderCurrent.value;
});

btnApplyCurrent.addEventListener('click', () => {
  conn.write(setCurrentCommand(parseInt(sliderCurrent.value, 10))).catch(
    (err: unknown) => {
      alert(`Send error: ${err instanceof Error ? err.message : String(err)}`);
    },
  );
});

// ── Telemetry ────────────────────────────────────────────────────────────────

conn.onPacket = (packet: Packet): void => {
  if (packet.type === 'update') {
    teleMeas.textContent = packet.meas.toString();
    teleVel.textContent  = packet.vel.toString();
    teleLag.textContent  = packet.lag.toString();
  } else {
    setMotionStatus(false);
    teleMeas.textContent = packet.pos.toString();
    stopInfo.textContent = `Stopped at ${packet.pos} steps — ${packet.reason}`;
    stopInfo.classList.remove('hidden');
  }
};

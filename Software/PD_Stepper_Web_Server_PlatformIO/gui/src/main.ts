import 'uplot/dist/uPlot.min.css';
import { SerialConnection } from './serial';
import { moveCommand, type Packet, type SettingsPacket, type StatusPacket } from './protocol';
import { TelemetryStore } from './telemetry-store';
import { TelemetryChart } from './chart';
import { encToDeg, encToRev, encPerSecToRPM, degToMicrosteps, rpmToStepsPerSec, degPerSec2ToStepsPerSec2, degToEnc, stepsPerSecToRPM, rpmToMicrostepsPerSec } from './units';

const conn  = new SerialConnection();
const store = new TelemetryStore();
const chart = new TelemetryChart();

// ── DOM references ──────────────────────────────────────────────────────────

const btnConnect      = document.getElementById('btn-connect')        as HTMLButtonElement;
const btnEstop        = document.getElementById('btn-estop')          as HTMLButtonElement;
const connDot         = document.getElementById('conn-dot')           as HTMLSpanElement;
const connStatus      = document.getElementById('conn-status')        as HTMLSpanElement;
const moveForm        = document.getElementById('move-form')          as HTMLFormElement;
const btnMove         = document.getElementById('btn-move')           as HTMLButtonElement;
const inpDistance     = document.getElementById('inp-distance')       as HTMLInputElement;
const inpSpeed        = document.getElementById('inp-speed')          as HTMLInputElement;
const inpAccel        = document.getElementById('inp-accel')          as HTMLInputElement;
const chkAbs          = document.getElementById('chk-abs')            as HTMLInputElement;
const teleMeas        = document.getElementById('tele-meas')          as HTMLSpanElement;
const teleTarget      = document.getElementById('tele-target')        as HTMLSpanElement;
const teleVel         = document.getElementById('tele-vel')           as HTMLSpanElement;
const teleLag         = document.getElementById('tele-lag')           as HTMLSpanElement;
const teleMaxLag      = document.getElementById('tele-max-lag')       as HTMLSpanElement;
const teleSkipped     = document.getElementById('tele-skipped')       as HTMLSpanElement;
const teleStatus      = document.getElementById('tele-status')        as HTMLDivElement;
const perfMaxLag      = document.getElementById('perf-max-lag')       as HTMLSpanElement;
const perfLagJitter   = document.getElementById('perf-lag-jitter')    as HTMLSpanElement;
const perfEffort      = document.getElementById('perf-effort')        as HTMLSpanElement;
const chartContainer  = document.getElementById('chart-container')    as HTMLDivElement;
const btnResetChart   = document.getElementById('btn-reset-chart')    as HTMLButtonElement;

// Hold accuracy section
const devNeedle       = document.getElementById('dev-needle')         as HTMLDivElement;
const holdDev         = document.getElementById('hold-dev')           as HTMLSpanElement;
const holdPeakDev     = document.getElementById('hold-peak-dev')      as HTMLSpanElement;
const holdSettleTime  = document.getElementById('hold-settle-time')   as HTMLSpanElement;

// USB link card
const linkUpdates     = document.getElementById('link-updates')       as HTMLSpanElement;
const linkStopsRatio  = document.getElementById('link-stops-ratio')   as HTMLSpanElement;
const linkCsumErr     = document.getElementById('link-csum-err')      as HTMLSpanElement;
const linkResyncs     = document.getElementById('link-resyncs')       as HTMLSpanElement;
const linkBytes       = document.getElementById('link-bytes')         as HTMLSpanElement;
const linkWriteErr    = document.getElementById('link-write-err')     as HTMLSpanElement;
const linkDrain       = document.getElementById('link-drain')         as HTMLSpanElement;

// PD settings card
const setKp           = document.getElementById('set-kp')             as HTMLInputElement;
const setKd           = document.getElementById('set-kd')             as HTMLInputElement;
const setKv           = document.getElementById('set-kv')             as HTMLInputElement;
const btnApplyPd      = document.getElementById('btn-apply-pd')       as HTMLButtonElement;
const btnSave         = document.getElementById('btn-save')           as HTMLButtonElement;
const syncIndicator   = document.getElementById('settings-sync-indicator') as HTMLDivElement;

// TMC settings card
const setVoltage      = document.getElementById('set-voltage')        as HTMLSelectElement;
const setMicrosteps   = document.getElementById('set-microsteps')     as HTMLSelectElement;
const setCurrent      = document.getElementById('set-current')        as HTMLInputElement;
const setHoldCurrent  = document.getElementById('set-hold-current')   as HTMLInputElement;
const setHoldDelay    = document.getElementById('set-hold-delay')     as HTMLInputElement;
const setStall        = document.getElementById('set-stall')          as HTMLInputElement;
const setStandstill   = document.getElementById('set-standstill')     as HTMLSelectElement;
const setStealthchop  = document.getElementById('set-stealthchop')    as HTMLInputElement;
const setSpreadEnable = document.getElementById('set-spread-enable')  as HTMLInputElement;
const setSpreadSpeed  = document.getElementById('set-spread-speed')   as HTMLInputElement;
const setCoolstep     = document.getElementById('set-coolstep')       as HTMLInputElement;
const btnApplyTmc     = document.getElementById('btn-apply-tmc')      as HTMLButtonElement;

// Driver status card
const dsVbus        = document.getElementById('ds-vbus')        as HTMLSpanElement;
const dsPg          = document.getElementById('ds-pg')          as HTMLSpanElement;
const dsCs          = document.getElementById('ds-cs')          as HTMLSpanElement;
const dsCsBar       = document.getElementById('ds-cs-bar')      as HTMLDivElement;
const dsPwm         = document.getElementById('ds-pwm')         as HTMLSpanElement;
const dsPwmBar      = document.getElementById('ds-pwm-bar')     as HTMLDivElement;
const dsSg          = document.getElementById('ds-sg')          as HTMLSpanElement;
const dsSgBar       = document.getElementById('ds-sg-bar')      as HTMLDivElement;
const dsStealth     = document.getElementById('ds-stealth')     as HTMLSpanElement;
const dsStandstill2 = document.getElementById('ds-standstill')  as HTMLSpanElement;
const dsBoot        = document.getElementById('ds-boot')        as HTMLSpanElement;
const dsReset       = document.getElementById('ds-reset')       as HTMLSpanElement;

// Fault badges (driver status)
const fbOtWarn    = document.getElementById('fb-ot-warn')    as HTMLSpanElement;
const fbOtShut    = document.getElementById('fb-ot-shut')    as HTMLSpanElement;
const fbShortA    = document.getElementById('fb-short-a')    as HTMLSpanElement;
const fbShortB    = document.getElementById('fb-short-b')    as HTMLSpanElement;
const fbOpenA     = document.getElementById('fb-open-a')     as HTMLSpanElement;
const fbOpenB     = document.getElementById('fb-open-b')     as HTMLSpanElement;
const fbLag       = document.getElementById('fb-lag')        as HTMLSpanElement;
const fbBrownout  = document.getElementById('fb-brownout')   as HTMLSpanElement;
const fbHoldActive = document.getElementById('fb-hold-active') as HTMLSpanElement;
const fbHoldSettled = document.getElementById('fb-hold-settled') as HTMLSpanElement;

// ── Session state ─────────────────────────────────────────────────────────────

let movesSent     = 0;
let stopsReceived = 0;
let settingsReceived = false;
let currentMicrosteps = 32;  // from SETTINGS packet, used for deg→µstep command conversion
let linkInterval: ReturnType<typeof setInterval> | null = null;

// Chart rendering is decoupled from packet arrival via requestAnimationFrame.
// Packets arrive at 100 Hz; the browser renders at ~60 Hz. Setting this flag
// on each packet and consuming it in the rAF loop means chart.update() is
// called at most once per animation frame, keeping the main thread responsive.
let _chartDirty = false;

function _rafLoop(): void {
  if (_chartDirty) {
    chart.update(store);
    _chartDirty = false;
  }
  requestAnimationFrame(_rafLoop);
}
requestAnimationFrame(_rafLoop);

// Keep kp/kd together since set_pd requires both
let currentKp = 3.0;
let currentKd = 0.1;

// ── Initial state ────────────────────────────────────────────────────────────

setControlsEnabled(false);
chart.init(chartContainer);

function setMotionEnabled(on: boolean): void {
  btnMove.disabled     = !on;
  inpDistance.disabled = !on;
  inpSpeed.disabled    = !on;
  inpAccel.disabled    = !on;
  chkAbs.disabled      = !on;
}

function setSettingsEnabled(on: boolean): void {
  setKp.disabled          = !on;
  setKd.disabled          = !on;
  setKv.disabled          = !on;
  btnApplyPd.disabled     = !on;
  btnSave.disabled        = !on;
  setVoltage.disabled     = !on;
  setMicrosteps.disabled  = !on;
  setCurrent.disabled     = !on;
  setHoldCurrent.disabled = !on;
  setHoldDelay.disabled   = !on;
  setStall.disabled       = !on;
  setStandstill.disabled  = !on;
  setStealthchop.disabled = !on;
  setSpreadEnable.disabled = !on;
  setSpreadSpeed.disabled = !on || !setSpreadEnable.checked;
  setCoolstep.disabled    = !on;
  btnApplyTmc.disabled    = !on;
}

// Toggle enables/disables the SpreadCycle threshold speed input
setSpreadEnable.addEventListener('change', () => {
  setSpreadSpeed.disabled = !setSpreadEnable.checked;
});

function setControlsEnabled(on: boolean): void {
  setMotionEnabled(on);
  setSettingsEnabled(on);
}

// ── Motion state machine ──────────────────────────────────────────────────────

type MotionState = 'idle' | 'moving' | 'correcting' | 'holding';

let motionState: MotionState = 'idle';
let stopReceivedAt = 0;  // timestamp for settle time computation

const STATE_LABELS: Record<MotionState, string> = {
  idle:       '● IDLE',
  moving:     '● MOVING',
  correcting: '● CORRECTING',
  holding:    '● HOLDING',
};

function setMotionState(state: MotionState): void {
  if (state === motionState) return;
  motionState = state;
  teleStatus.textContent = STATE_LABELS[state];
  teleStatus.className   = `status ${state}`;
}

// Gauge range: ±DEV_RANGE degrees maps to full width
const DEV_RANGE = encToDeg(20);  // ~1.76°

/** Reset Move Performance and Hold Accuracy to dimmed placeholders */
function resetPerfAndHold(): void {
  perfMaxLag.textContent    = '–';
  perfLagJitter.textContent = '–';
  perfEffort.textContent    = '–';
  perfMaxLag.classList.add('dimmed');
  perfLagJitter.classList.add('dimmed');
  perfEffort.classList.add('dimmed');

  holdDev.textContent       = '–';
  holdPeakDev.textContent   = '–';
  holdSettleTime.textContent = '–';
  holdDev.className         = 'metric-value dev-value dimmed';
  holdPeakDev.classList.add('dimmed');
  holdSettleTime.classList.add('dimmed');
  devNeedle.style.left      = '50%';
}
resetPerfAndHold();

function updateDeviationGauge(lag: number): void {
  const lagDeg = encToDeg(lag);
  // Needle position: 50% = center, clamp to [2%, 98%]
  const pct = Math.max(2, Math.min(98, 50 + (lagDeg / DEV_RANGE) * 50));
  devNeedle.style.left = `${pct}%`;

  // Colour class based on magnitude (in encoder counts)
  const absLag = Math.abs(lag);
  let cls: string;
  if (absLag <= 6)       cls = 'dev-green';
  else if (absLag <= 12) cls = 'dev-amber';
  else                   cls = 'dev-red';

  const sign = lagDeg >= 0 ? '+' : '';
  holdDev.textContent = `${sign}${lagDeg.toFixed(2)}°`;
  holdDev.className   = `metric-value dev-value ${cls}`;
}

// ── Health colour helpers ─────────────────────────────────────────────────────

/** Returns CSS class name for a counter: ok / warn / error */
function health(value: number, warnAt: number, errorAt: number): string {
  if (value >= errorAt) return 'error';
  if (value >= warnAt)  return 'warn';
  return 'ok';
}

function fmtBytes(n: number): string {
  if (n >= 1_048_576) return `${(n / 1_048_576).toFixed(1)} MB`;
  if (n >= 1024)      return `${(n / 1024).toFixed(1)} KB`;
  return `${n} B`;
}

// ── Reset reason mapping ──────────────────────────────────────────────────────

function resetReasonStr(n: number): string {
  const reasons: Record<number, string> = {
    1: 'Power On', 3: 'Software', 4: 'Panic',
    5: 'Int WDT', 6: 'Task WDT', 7: 'WDT',
    8: 'Deep Sleep', 9: 'Brownout', 10: 'SDIO',
  };
  return reasons[n] ?? `Unknown (${n})`;
}

// ── USB link stats ────────────────────────────────────────────────────────────

function updateLinkStats(): void {
  if (!conn.isConnected) return;
  const ls = conn.linkStats;

  linkUpdates.textContent    = ls.updateCount.toString();
  linkBytes.textContent      = `${fmtBytes(ls.bytesIn)} / ${fmtBytes(ls.bytesOut)}`;
  linkDrain.textContent      = fmtBytes(ls.drainBytes);

  // Colour-coded counters
  const stopsMissed = movesSent - stopsReceived;
  linkStopsRatio.textContent = `${stopsReceived} / ${movesSent}`;
  linkStopsRatio.className   = `metric-value ${health(stopsMissed, 1, 2)}`;

  linkCsumErr.textContent  = ls.checksumErrors.toString();
  linkCsumErr.className    = `metric-value ${health(ls.checksumErrors, 1, 5)}`;

  linkResyncs.textContent  = ls.resyncEvents.toString();
  linkResyncs.className    = `metric-value ${health(ls.resyncEvents, 1, 5)}`;

  linkWriteErr.textContent = ls.writeErrors.toString();
  linkWriteErr.className   = `metric-value ${health(ls.writeErrors, 1, 3)}`;
}

// ── Connection ───────────────────────────────────────────────────────────────

conn.onConnectionChange = (connected: boolean): void => {
  connDot.className      = `dot ${connected ? 'connected' : 'disconnected'}`;
  connStatus.textContent = connected ? 'Connected' : 'Disconnected';
  btnConnect.textContent = connected ? 'Disconnect' : 'Connect';
  btnConnect.classList.toggle('connected', connected);
  if (!connected) {
    settingsReceived = false;
    setControlsEnabled(false);
    setMotionState('idle');
    btnEstop.disabled = true;
    if (linkInterval) { clearInterval(linkInterval); linkInterval = null; }
  } else {
    setMotionEnabled(true);
    setSettingsEnabled(false);   // gated — unlocked by onSettings
    btnEstop.disabled = false;
    movesSent = stopsReceived = 0;
    syncIndicator.classList.remove('hidden');
    linkInterval = setInterval(updateLinkStats, 1000);
    updateLinkStats();
    // Delay past the 150 ms drain window in serial.ts — the firmware responds
    // to get_settings in <10 ms, so sending immediately would land inside the
    // drain window and get silently discarded.
    setTimeout(() => {
      conn.write('{"cmd":"get_settings"}\n').catch(() => undefined);
    }, 200);
  }
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

btnEstop.addEventListener('click', () => {
  sendCmd({ cmd: 'estop' });
});

btnResetChart.addEventListener('click', () => {
  store.clear();
  chart.resetZoom();   // clear zoom + Y seed → full auto
  chart.update(store);
});

// ── Move command ─────────────────────────────────────────────────────────────

moveForm.addEventListener('submit', (e: Event) => {
  e.preventDefault();
  resetPerfAndHold();
  setMotionState('moving');

  // Capture last known position (encoder counts) BEFORE clearing the store
  const currentMeas = store.lastMeas;
  store.clear();

  // User inputs are in degrees / RPM / °/s²
  const distDeg  = parseFloat(inpDistance.value);
  const speedRPM = parseFloat(inpSpeed.value);
  const accelDPS = parseFloat(inpAccel.value);

  // Convert to encoder counts for chart seeding
  const distEnc  = degToEnc(distDeg);
  const finalPos = chkAbs.checked ? distEnc : (currentMeas + distEnc);
  const speedEnc = rpmToStepsPerSec(speedRPM, currentMicrosteps) / (200 * currentMicrosteps / 4096);
  const padding  = Math.abs(finalPos - currentMeas) * 0.1 + 10;
  chart.seedYRange(
    Math.min(currentMeas, finalPos) - padding,
    Math.max(currentMeas, finalPos) + padding,
    speedEnc,
  );

  chart.update(store);
  movesSent++;

  // Convert to microsteps for the firmware JSON command
  const cmd = moveCommand(
    degToMicrosteps(distDeg, currentMicrosteps),
    rpmToStepsPerSec(speedRPM, currentMicrosteps),
    degPerSec2ToStepsPerSec2(accelDPS, currentMicrosteps),
    chkAbs.checked,
  );
  conn.write(cmd).catch((err: unknown) => {
    movesSent--;
    setMotionState('idle');
    alert(`Send error: ${err instanceof Error ? err.message : String(err)}`);
  });
});

// ── Settings: apply helpers ───────────────────────────────────────────────────

function sendCmd(obj: Record<string, unknown>): void {
  conn.write(JSON.stringify(obj) + '\n').catch(() => undefined);
}

btnApplyPd.addEventListener('click', () => {
  currentKp = parseFloat(setKp.value);
  currentKd = parseFloat(setKd.value);
  sendCmd({ cmd: 'set_pd', kp: currentKp, kd: currentKd });
  sendCmd({ cmd: 'set_phase_lead', kv: parseFloat(setKv.value) });
});
btnSave.addEventListener('click', () => {
  sendCmd({ cmd: 'save' });
});
btnApplyTmc.addEventListener('click', () => {
  sendCmd({ cmd: 'set_voltage',        value: parseInt(setVoltage.value, 10) });
  sendCmd({ cmd: 'set_microsteps',     value: parseInt(setMicrosteps.value, 10) });
  sendCmd({ cmd: 'set_current',        value: parseInt(setCurrent.value, 10) });
  sendCmd({ cmd: 'set_hold_current',   value: parseInt(setHoldCurrent.value, 10) });
  sendCmd({ cmd: 'set_hold_delay',     value: parseInt(setHoldDelay.value, 10) });
  sendCmd({ cmd: 'set_stall_threshold',value: parseInt(setStall.value, 10) });
  sendCmd({ cmd: 'set_standstill_mode',value: setStandstill.value });
  sendCmd({ cmd: 'set_stealthchop',    value: setStealthchop.checked ? 1 : 0 });
  sendCmd({ cmd: 'set_spread_cycle_speed', value: setSpreadEnable.checked ? Math.round(rpmToMicrostepsPerSec(parseFloat(setSpreadSpeed.value), currentMicrosteps)) : 0 });
  sendCmd({ cmd: 'set_coolstep',       value: setCoolstep.checked ? 1 : 0 });
});

// ── Settings packet handler ────────────────────────────────────────────────────

const standstillModes = ['NORMAL', 'FREEWHEELING', 'BRAKING', 'STRONG_BRAKING'];

conn.onSettings = (pkt: SettingsPacket): void => {
  syncIndicator.classList.add('hidden');
  settingsReceived = true;
  currentMicrosteps = pkt.microsteps || 32;
  setSettingsEnabled(true);   // unlock now that values are known

  setVoltage.value     = pkt.voltage.toString();
  setMicrosteps.value  = pkt.microsteps.toString();
  setCurrent.value     = pkt.current.toString();
  setHoldCurrent.value = pkt.holdCurrent.toString();
  setHoldDelay.value   = pkt.holdDelay.toString();
  setStall.value       = pkt.stallThreshold.toString();
  setStandstill.value  = standstillModes[pkt.standstillMode] ?? 'NORMAL';
  setStealthchop.checked = pkt.stealthchop;
  setSpreadEnable.checked = pkt.spreadCycleSpeed > 0;
  // SpreadCycle threshold: firmware stores in steps/s, display as RPM
  const spreadRPM = pkt.spreadCycleSpeed > 0
    ? stepsPerSecToRPM(pkt.spreadCycleSpeed, currentMicrosteps)
    : 50;
  setSpreadSpeed.value   = spreadRPM.toFixed(0);
  setSpreadSpeed.disabled = !pkt.spreadCycleSpeed;
  setCoolstep.checked    = pkt.coolstep;

  currentKp = pkt.kp;
  currentKd = pkt.kd;
  setKp.value = pkt.kp.toFixed(3);
  setKd.value = pkt.kd.toFixed(4);
  setKv.value = pkt.kv.toFixed(5);
};

// ── Status packet handler ────────────────────────────────────────────────────

function setBadge(el: HTMLSpanElement, active: boolean, className = 'active'): void {
  el.className = `fault-badge${active ? ' ' + className : ''}`;
}

conn.onStatus = (pkt: StatusPacket): void => {
  dsVbus.textContent     = `${(pkt.vbusMv / 1000).toFixed(2)} V`;
  dsPg.textContent       = pkt.pgOk ? 'OK' : 'FAIL';
  dsPg.className         = `metric-value ${pkt.pgOk ? 'ok' : 'error'}`;

  const csPct = Math.round((pkt.csActual / 31) * 100);
  dsCs.textContent       = `${csPct}%`;
  dsCsBar.style.width    = `${csPct}%`;
  dsPwm.textContent      = `${Math.round((pkt.pwmScale / 255) * 100)}%`;
  dsPwmBar.style.width   = `${Math.round((pkt.pwmScale / 255) * 100)}%`;
  dsSg.textContent       = `${Math.round((pkt.sgResult / 1023) * 100)}%`;
  dsSgBar.style.width    = `${Math.round((pkt.sgResult / 1023) * 100)}%`;
  dsStealth.textContent  = pkt.stealthchopActive ? 'StealthChop' : 'SpreadCycle';
  dsStealth.className    = `metric-value ${pkt.stealthchopActive ? 'ok' : ''}`;
  dsStandstill2.textContent = pkt.standstill ? 'YES' : 'no';

  setBadge(fbOtWarn,   pkt.otWarn,      'warn');
  setBadge(fbOtShut,   pkt.otShutdown,  'active');
  setBadge(fbShortA,   pkt.shortGndA,   'active');
  setBadge(fbShortB,   pkt.shortGndB,   'active');
  setBadge(fbOpenA,    pkt.openLoadA,   'warn');
  setBadge(fbOpenB,    pkt.openLoadB,   'warn');
  setBadge(fbLag,      pkt.faultLag,    'active');
  setBadge(fbBrownout, pkt.faultBrownout, 'active');

  dsBoot.textContent  = pkt.bootCount.toString();
  dsReset.textContent = resetReasonStr(pkt.resetReason);

  setBadge(fbHoldActive,  pkt.holdActive,  'ok');
  setBadge(fbHoldSettled, pkt.holdSettled, 'ok');

  // Derive motion state from authoritative STATUS flags
  if (pkt.isRunning) {
    setMotionState('moving');
  } else if (pkt.holdActive && !pkt.holdSettled) {
    setMotionState('correcting');
  } else if (pkt.holdActive && pkt.holdSettled) {
    // Record settle time on first transition to HOLDING
    if (motionState === 'correcting' && stopReceivedAt > 0) {
      holdSettleTime.textContent = `${Date.now() - stopReceivedAt} ms`;
    }
    setMotionState('holding');
  } else {
    setMotionState('idle');
  }

  // Gate TMC settings apply button during motion — firmware rejects those commands anyway
  if (settingsReceived) {
    btnApplyTmc.disabled = pkt.isRunning;
  }
};

// ── Telemetry ────────────────────────────────────────────────────────────────

conn.onPacket = (packet: Packet): void => {
  if (packet.type === 'update') {
    // Display in degrees / RPM (encoder-count-independent units)
    const measDeg = encToDeg(packet.meas);
    const measRev = encToRev(packet.meas);
    teleMeas.textContent    = `${measDeg.toFixed(1)}° (${measRev.toFixed(2)} rev)`;
    teleTarget.textContent  = `${encToDeg(packet.target).toFixed(1)}°`;
    teleVel.textContent     = `${encPerSecToRPM(packet.vel).toFixed(1)} RPM`;
    teleLag.textContent     = `${encToDeg(packet.lag).toFixed(2)}°`;

    // Chart accumulates during moving/correcting, freezes when settled
    const shouldChart = motionState === 'moving' || motionState === 'correcting';
    store.push(packet, shouldChart);
    if (shouldChart) _chartDirty = true;

    const ms = store.moveStats;
    teleMaxLag.textContent  = `${ms.peakLagDeg.toFixed(2)}°`;
    teleSkipped.textContent = `${ms.skippedDeg.toFixed(2)}°`;
    _chartDirty = true;

    // Update hold accuracy gauge if in hold phase
    if (motionState === 'correcting' || motionState === 'holding') {
      updateDeviationGauge(packet.lag);
      holdPeakDev.textContent = `${ms.holdPeakDevDeg.toFixed(2)}°`;
    }
  } else {
    stopsReceived++;
    teleMeas.textContent = `${encToDeg(packet.pos).toFixed(1)}°`;

    // Enter hold phase for deviation tracking
    const isNormalStop = !packet.reason.includes('Fault') && !packet.reason.includes('E-STOP');
    if (isNormalStop) {
      store.enterHoldPhase();
      stopReceivedAt = Date.now();
      setMotionState('correcting');
      holdSettleTime.textContent = '…';
      holdSettleTime.classList.remove('dimmed');
      holdPeakDev.textContent = '—';
      holdPeakDev.classList.remove('dimmed');
      holdDev.classList.remove('dimmed');
      updateDeviationGauge(0);
    } else {
      setMotionState('idle');
    }

    // Populate per-move performance panel (undim values)
    const ms = store.moveStats;
    perfMaxLag.textContent    = `${ms.peakLagDeg.toFixed(2)}°`;
    perfLagJitter.textContent = `${ms.lagJitterDeg.toFixed(2)}°`;
    perfEffort.textContent    = `${ms.effortPct.toFixed(1)}%`;
    perfMaxLag.classList.remove('dimmed');
    perfLagJitter.classList.remove('dimmed');
    perfEffort.classList.remove('dimmed');

    updateLinkStats();
  }
};

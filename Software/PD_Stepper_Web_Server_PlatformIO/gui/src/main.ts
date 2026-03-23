import 'uplot/dist/uPlot.min.css';
import { SerialConnection } from './serial';
import { moveCommand, type Packet, type SettingsPacket, type StatusPacket } from './protocol';
import { TelemetryStore } from './telemetry-store';
import { TelemetryChart } from './chart';

const conn  = new SerialConnection();
const store = new TelemetryStore();
const chart = new TelemetryChart();

// ── DOM references ──────────────────────────────────────────────────────────

const btnConnect      = document.getElementById('btn-connect')        as HTMLButtonElement;
const connDot         = document.getElementById('conn-dot')           as HTMLSpanElement;
const connStatus      = document.getElementById('conn-status')        as HTMLSpanElement;
const moveForm        = document.getElementById('move-form')          as HTMLFormElement;
const btnMove         = document.getElementById('btn-move')           as HTMLButtonElement;
const inpDistance     = document.getElementById('inp-distance')       as HTMLInputElement;
const inpSpeed        = document.getElementById('inp-speed')          as HTMLInputElement;
const inpAccel        = document.getElementById('inp-accel')          as HTMLInputElement;
const chkAbs          = document.getElementById('chk-abs')            as HTMLInputElement;
const teleMeas        = document.getElementById('tele-meas')          as HTMLSpanElement;
const teleVel         = document.getElementById('tele-vel')           as HTMLSpanElement;
const teleLag         = document.getElementById('tele-lag')           as HTMLSpanElement;
const teleSkipped     = document.getElementById('tele-skipped')       as HTMLSpanElement;
const teleStatus      = document.getElementById('tele-status')        as HTMLDivElement;
const stopInfo        = document.getElementById('stop-info')          as HTMLDivElement;
const perfMetrics     = document.getElementById('perf-metrics')       as HTMLDivElement;
const perfSkipped     = document.getElementById('perf-skipped')       as HTMLSpanElement;
const perfMaxLag      = document.getElementById('perf-max-lag')       as HTMLSpanElement;
const perfLagJitter   = document.getElementById('perf-lag-jitter')    as HTMLSpanElement;
const perfEffort      = document.getElementById('perf-effort')        as HTMLSpanElement;
const chartContainer  = document.getElementById('chart-container')    as HTMLDivElement;

// USB link card
const linkUpdates     = document.getElementById('link-updates')       as HTMLSpanElement;
const linkStopsRatio  = document.getElementById('link-stops-ratio')   as HTMLSpanElement;
const linkCsumErr     = document.getElementById('link-csum-err')      as HTMLSpanElement;
const linkResyncs     = document.getElementById('link-resyncs')       as HTMLSpanElement;
const linkGaps        = document.getElementById('link-gaps')          as HTMLSpanElement;
const linkMaxGap      = document.getElementById('link-max-gap')       as HTMLSpanElement;
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
const setCoolstep     = document.getElementById('set-coolstep')       as HTMLInputElement;
const btnApplyTmc     = document.getElementById('btn-apply-tmc')      as HTMLButtonElement;

// Driver status card
const dsVbus        = document.getElementById('ds-vbus')        as HTMLSpanElement;
const dsPg          = document.getElementById('ds-pg')          as HTMLSpanElement;
const dsCs          = document.getElementById('ds-cs')          as HTMLSpanElement;
const dsCsBar       = document.getElementById('ds-cs-bar')      as HTMLDivElement;
const dsPwm         = document.getElementById('ds-pwm')         as HTMLSpanElement;
const dsTstep       = document.getElementById('ds-tstep')       as HTMLSpanElement;
const dsSg          = document.getElementById('ds-sg')          as HTMLSpanElement;
const dsStealth     = document.getElementById('ds-stealth')     as HTMLSpanElement;
const dsStandstill2 = document.getElementById('ds-standstill')  as HTMLSpanElement;
const dsHeap        = document.getElementById('ds-heap')        as HTMLSpanElement;
const dsHwm         = document.getElementById('ds-hwm')         as HTMLSpanElement;
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
let linkInterval: ReturnType<typeof setInterval> | null = null;

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
  setCoolstep.disabled    = !on;
  btnApplyTmc.disabled    = !on;
}

function setControlsEnabled(on: boolean): void {
  setMotionEnabled(on);
  setSettingsEnabled(on);
}

function setMotionStatus(moving: boolean): void {
  teleStatus.textContent = moving ? '● MOVING' : '● STOPPED';
  teleStatus.className   = `status ${moving ? 'moving' : 'stopped'}`;
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
  linkMaxGap.textContent     = `${ls.maxGapMs} ms`;
  linkDrain.textContent      = fmtBytes(ls.drainBytes);

  // Colour-coded counters
  const stopsMissed = movesSent - stopsReceived;
  linkStopsRatio.textContent = `${stopsReceived} / ${movesSent}`;
  linkStopsRatio.className   = `metric-value ${health(stopsMissed, 1, 2)}`;

  linkCsumErr.textContent  = ls.checksumErrors.toString();
  linkCsumErr.className    = `metric-value ${health(ls.checksumErrors, 1, 5)}`;

  linkResyncs.textContent  = ls.resyncEvents.toString();
  linkResyncs.className    = `metric-value ${health(ls.resyncEvents, 1, 5)}`;

  linkGaps.textContent     = ls.gapsOver150ms.toString();
  linkGaps.className       = `metric-value ${health(ls.gapsOver150ms, 1, 5)}`;

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
    setControlsEnabled(false);
    setMotionStatus(false);
    if (linkInterval) { clearInterval(linkInterval); linkInterval = null; }
  } else {
    setMotionEnabled(true);
    setSettingsEnabled(false);   // gated — unlocked by onSettings
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

// ── Move command ─────────────────────────────────────────────────────────────

moveForm.addEventListener('submit', (e: Event) => {
  e.preventDefault();
  stopInfo.classList.add('hidden');
  perfMetrics.classList.add('hidden');
  setMotionStatus(true);
  store.clear();
  chart.update(store);
  movesSent++;
  const cmd = moveCommand(
    parseInt(inpDistance.value, 10),
    parseFloat(inpSpeed.value),
    parseFloat(inpAccel.value),
    chkAbs.checked,
  );
  conn.write(cmd).catch((err: unknown) => {
    movesSent--;
    setMotionStatus(false);
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
  sendCmd({ cmd: 'set_coolstep',       value: setCoolstep.checked ? 1 : 0 });
});

// ── Settings packet handler ────────────────────────────────────────────────────

const standstillModes = ['NORMAL', 'FREEWHEELING', 'BRAKING', 'STRONG_BRAKING'];

conn.onSettings = (pkt: SettingsPacket): void => {
  syncIndicator.classList.add('hidden');
  setSettingsEnabled(true);   // unlock now that values are known

  setVoltage.value     = pkt.voltage.toString();
  setMicrosteps.value  = pkt.microsteps.toString();
  setCurrent.value     = pkt.current.toString();
  setHoldCurrent.value = pkt.holdCurrent.toString();
  setHoldDelay.value   = pkt.holdDelay.toString();
  setStall.value       = pkt.stallThreshold.toString();
  setStandstill.value  = standstillModes[pkt.standstillMode] ?? 'NORMAL';
  setStealthchop.checked = pkt.stealthchop;
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

  dsCs.textContent       = `${pkt.csActual} / 31`;
  dsCsBar.style.width    = `${Math.round((pkt.csActual / 31) * 100)}%`;
  dsPwm.textContent      = pkt.pwmScale.toString();
  dsTstep.textContent    = pkt.tstep.toString();
  dsSg.textContent       = pkt.sgResult.toString();
  dsStealth.textContent  = pkt.stealthchopActive ? 'ON' : 'off';
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

  dsHeap.textContent  = `${pkt.freeHeapKb} kB`;
  dsHwm.textContent   = `${pkt.ctrlHwm} B`;
  dsBoot.textContent  = pkt.bootCount.toString();
  dsReset.textContent = resetReasonStr(pkt.resetReason);

  setBadge(fbHoldActive,  pkt.holdActive,  'ok');
  setBadge(fbHoldSettled, pkt.holdSettled, 'ok');
};

// ── Telemetry ────────────────────────────────────────────────────────────────

conn.onPacket = (packet: Packet): void => {
  if (packet.type === 'update') {
    teleMeas.textContent    = packet.meas.toString();
    teleVel.textContent     = packet.vel.toString();
    teleLag.textContent     = packet.lag.toString();
    store.push(packet);
    teleSkipped.textContent = store.moveStats.skippedSteps.toString();
    chart.update(store);
  } else {
    stopsReceived++;
    setMotionStatus(false);
    teleMeas.textContent = packet.pos.toString();
    stopInfo.textContent = `Stopped at ${packet.pos} steps — ${packet.reason}`;
    stopInfo.classList.remove('hidden');

    // Populate per-move performance panel
    const ms = store.moveStats;
    perfSkipped.textContent   = `${ms.peakSkippedSteps} steps`;
    perfMaxLag.textContent    = `${ms.peakLag} steps`;
    perfLagJitter.textContent = `${ms.lagJitter.toFixed(1)} steps`;
    perfEffort.textContent    = `${ms.effortPct.toFixed(1)}%`;
    perfSkipped.className     = `metric-value ${health(ms.peakSkippedSteps, 1, 10)}`;
    perfMetrics.classList.remove('hidden');

    updateLinkStats(); // immediate refresh after STOP
  }
};

import 'uplot/dist/uPlot.min.css';
import { SerialConnection } from './serial';
import { moveCommand, type Packet, type SettingsPacket, type StatusPacket, type BlockDonePacket, type HomingDonePacket } from './protocol';
import { TelemetryStore } from './telemetry-store';
import { TelemetryChart } from './chart';
import { encToDeg, encPerSecToRPM, degToMicrosteps, rpmToStepsPerSec, degPerSec2ToStepsPerSec2, degToEnc, stepsPerSecToRPM, rpmToMicrostepsPerSec } from './units';
import { AutoTuner, type TuneResult, type TunerProgress } from './tuner';
import { QueueStore, type QueueEntry } from './queue-store';
import { QueueRunner } from './queue-runner';

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
const teleStatus      = document.getElementById('tele-status')        as HTMLDivElement;
const perfMaxLag      = document.getElementById('perf-max-lag')       as HTMLSpanElement;
const perfMeanLag     = document.getElementById('perf-mean-lag')      as HTMLSpanElement;
const perfStepLoss    = document.getElementById('perf-step-loss')     as HTMLSpanElement;
const perfLagJitter   = document.getElementById('perf-lag-jitter')    as HTMLSpanElement;
const perfEffort      = document.getElementById('perf-effort')        as HTMLSpanElement;
const chartContainer  = document.getElementById('chart-container')    as HTMLDivElement;
const btnResetChart   = document.getElementById('btn-reset-chart')    as HTMLButtonElement;

// Hold accuracy section
const devNeedle       = document.getElementById('dev-needle')         as HTMLDivElement;
const holdDev         = document.getElementById('hold-dev')           as HTMLSpanElement;
const holdPeakDev     = document.getElementById('hold-peak-dev')      as HTMLSpanElement;
const holdSettleTime  = document.getElementById('hold-settle-time')   as HTMLSpanElement;

// Queue pane
const tabMove         = document.getElementById('tab-move')           as HTMLButtonElement;
const tabDwell        = document.getElementById('tab-dwell')          as HTMLButtonElement;
const moveFields      = document.getElementById('move-fields')        as HTMLDivElement;
const dwellFields     = document.getElementById('dwell-fields')       as HTMLDivElement;
const inpDwell        = document.getElementById('inp-dwell')          as HTMLInputElement;
const btnAddMove      = document.getElementById('btn-add-move')       as HTMLButtonElement;
const btnAddDwell     = document.getElementById('btn-add-dwell')      as HTMLButtonElement;
const queueList       = document.getElementById('queue-list')         as HTMLDivElement;
const queueCount      = document.getElementById('queue-count')        as HTMLSpanElement;
const btnRunQueue     = document.getElementById('btn-run-queue')      as HTMLButtonElement;
const btnLoopQueue    = document.getElementById('btn-loop-queue')     as HTMLButtonElement;
const btnClearQueue   = document.getElementById('btn-clear-queue')    as HTMLButtonElement;
const blockStatsSection = document.getElementById('block-stats-section') as HTMLElement;
const blockStatsList  = document.getElementById('block-stats-list')   as HTMLDivElement;

// Jog controls
const btnJogNeg       = document.getElementById('btn-jog-neg')         as HTMLButtonElement;
const btnJogPos       = document.getElementById('btn-jog-pos')         as HTMLButtonElement;
const inpJogSpeed     = document.getElementById('inp-jog-speed')       as HTMLInputElement;
const jogStepBtns     = document.querySelectorAll<HTMLButtonElement>('.jog-step');

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
const setDAlpha       = document.getElementById('set-d-alpha')        as HTMLInputElement;
const setJerkInput    = document.getElementById('set-jerk')           as HTMLInputElement;
const setKv           = document.getElementById('set-kv')             as HTMLInputElement;
const btnApplyPd      = document.getElementById('btn-apply-pd')       as HTMLButtonElement;
const btnSave         = document.getElementById('btn-save')           as HTMLButtonElement;
const syncIndicator   = document.getElementById('settings-sync-indicator') as HTMLDivElement;
const btnAutotune     = document.getElementById('btn-autotune')       as HTMLButtonElement;
const btnAutotuneStop = document.getElementById('btn-autotune-stop')  as HTMLButtonElement;
const autotuneProgress= document.getElementById('autotune-progress')  as HTMLDivElement;
const autotuneResults = document.getElementById('autotune-results')   as HTMLDivElement;
const atParamLabel    = document.getElementById('at-param-label')     as HTMLSpanElement;
const atIterLabel     = document.getElementById('at-iter-label')      as HTMLSpanElement;
const atProgressFill  = document.getElementById('at-progress-fill')   as HTMLDivElement;
const atLogList       = document.getElementById('at-log-list')        as HTMLUListElement;
const atSummaryList   = document.getElementById('at-summary-list')    as HTMLUListElement;

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

// Homing card
const btnHome        = document.getElementById('btn-home')          as HTMLButtonElement;
const chkHomeInvert  = document.getElementById('chk-home-invert')   as HTMLInputElement;
const setHomeCurrent = document.getElementById('set-home-current')  as HTMLInputElement;
const setHomeSpeed1  = document.getElementById('set-home-speed1')   as HTMLInputElement;
const setHomeSpeed2  = document.getElementById('set-home-speed2')   as HTMLInputElement;
const setHomeSg1     = document.getElementById('set-home-sg1')      as HTMLInputElement;
const setHomeSg2     = document.getElementById('set-home-sg2')      as HTMLInputElement;
const homingResultDiv = document.getElementById('homing-result')    as HTMLDivElement;

// ── Session state ─────────────────────────────────────────────────────────────

let movesSent     = 0;
let stopsReceived = 0;
let settingsReceived = false;
let currentMicrosteps = 32;  // from SETTINGS packet, used for deg→µstep command conversion
let linkInterval: ReturnType<typeof setInterval> | null = null;
let activeTuner: AutoTuner | null = null;


// ── Queue ──────────────────────────────────────────────────────────────────────

const queueStore = new QueueStore();
const queueRunner = new QueueRunner(queueStore, {
  write: (cmd) => conn.write(cmd),
  getMicrosteps: () => currentMicrosteps,
  onStateChange: (state) => {
    const running = state !== 'idle';
    btnRunQueue.disabled = running || queueStore.length === 0;
    btnClearQueue.disabled = running;
    btnAddMove.disabled = running;
    btnAddDwell.disabled = running;
    // Disable direct move and jog during queue execution
    btnMove.disabled = running;
    btnJogNeg.disabled = running;
    btnJogPos.disabled = running;
    if (running && state === 'sending') {
      resetPerfAndHold();
      store.clear();
      chart.update(store);
      setMotionState('moving');
    }
  },
  onComplete: () => {
    renderBlockStats();
  },
  onAbort: (_reason) => {
    renderBlockStats();
  },
});

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
  btnRunQueue.disabled = !on || queueStore.length === 0;
  btnJogNeg.disabled   = !on;
  btnJogPos.disabled   = !on;
}

function setSettingsEnabled(on: boolean): void {
  setKp.disabled          = !on;
  setKd.disabled          = !on;
  setDAlpha.disabled      = !on;
  setJerkInput.disabled   = !on;
  setKv.disabled          = !on;
  btnApplyPd.disabled     = !on;
  btnAutotune.disabled    = !on;
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
  perfMeanLag.textContent   = '–';
  perfStepLoss.textContent  = '–';
  perfLagJitter.textContent = '–';
  perfEffort.textContent    = '–';
  perfMaxLag.classList.add('dimmed');
  perfMeanLag.classList.add('dimmed');
  perfStepLoss.classList.add('dimmed');
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
  if (queueRunner.isRunning) queueRunner.abort('E-STOP (GUI)');
});

// Homing button
function startHoming(): void {
  const directionCW = !chkHomeInvert.checked;
  const speed1 = rpmToMicrostepsPerSec(parseFloat(setHomeSpeed1.value) || 30, currentMicrosteps);
  const speed2 = rpmToMicrostepsPerSec(parseFloat(setHomeSpeed2.value) || 5, currentMicrosteps);
  store.clear();
  chart.resetZoom();
  chart.update(store);
  sendCmd({
    cmd: 'home',
    direction: directionCW ? 'cw' : 'ccw',
    current_pct: parseInt(setHomeCurrent.value, 10) || 40,
    speed1,
    speed2,
    sg_thresh1: parseInt(setHomeSg1.value, 10) || 100,
    sg_thresh2: parseInt(setHomeSg2.value, 10) || 80,
  });
  btnHome.disabled = true;
}
btnHome.addEventListener('click', () => startHoming());

btnResetChart.addEventListener('click', () => {
  store.clear();
  chart.resetZoom();   // clear zoom + Y seed → full auto
  chart.update(store);
});

// ── Queue UI ────────────────────────────────────────────────────────────────

// Tab switching (Move / Dwell)
tabMove.addEventListener('click', () => {
  tabMove.classList.add('active');
  tabDwell.classList.remove('active');
  moveFields.classList.remove('hidden');
  dwellFields.classList.add('hidden');
});
tabDwell.addEventListener('click', () => {
  tabDwell.classList.add('active');
  tabMove.classList.remove('active');
  dwellFields.classList.remove('hidden');
  moveFields.classList.add('hidden');
});

// Add entry buttons
btnAddMove.addEventListener('click', () => {
  queueStore.addMove({
    distanceDeg: parseFloat(inpDistance.value) || 0,
    speedRPM:    parseFloat(inpSpeed.value) || 112,
    accelDPS2:   parseFloat(inpAccel.value) || 500,
    absolute:    chkAbs.checked,
  });
});
btnAddDwell.addEventListener('click', () => {
  queueStore.addDwell({ durationMs: parseFloat(inpDwell.value) || 500 });
});

// Run queue
btnRunQueue.addEventListener('click', () => {
  if (queueRunner.isRunning) return;
  movesSent++;
  blockStatsSection.classList.add('hidden');
  queueRunner.start();
});

// Loop toggle
btnLoopQueue.addEventListener('click', () => {
  queueRunner.loop = !queueRunner.loop;
  btnLoopQueue.classList.toggle('active', queueRunner.loop);
});

// Clear queue
btnClearQueue.addEventListener('click', () => {
  queueStore.clear();
  blockStatsSection.classList.add('hidden');
});

// Queue rendering
function renderQueue(): void {
  queueCount.textContent = `${queueStore.length} entries`;
  btnRunQueue.disabled = queueStore.length === 0 || queueRunner.isRunning || !conn.isConnected;

  queueList.innerHTML = '';
  let moveNum = 0, dwellNum = 0;

  for (let i = 0; i < queueStore.entries.length; i++) {
    const entry = queueStore.entries[i];
    const div = document.createElement('div');
    div.className = 'queue-entry';
    div.dataset.id = entry.id;
    div.dataset.status = entry.status;
    div.draggable = true;

    let label: string;
    if (entry.type === 'move') {
      moveNum++;
      const p = entry.params;
      const sign = p.absolute ? 'abs ' : (p.distanceDeg >= 0 ? '+' : '');
      label = `M${moveNum}: ${sign}${p.distanceDeg.toFixed(1)}° @ ${p.speedRPM} RPM, ${p.accelDPS2}°/s²`;
    } else {
      dwellNum++;
      label = `D${dwellNum}: Wait ${entry.params.durationMs} ms`;
    }

    div.innerHTML = `
      <span class="queue-label">${label}</span>
      <span class="queue-status-dot"></span>
      <button class="queue-edit" title="Edit">&#9998;</button>
      <button class="queue-delete" title="Delete">&times;</button>
    `;

    // Delete handler
    div.querySelector('.queue-delete')!.addEventListener('click', (e) => {
      e.stopPropagation();
      queueStore.remove(entry.id);
    });

    // Edit handler
    div.querySelector('.queue-edit')!.addEventListener('click', (e) => {
      e.stopPropagation();
      startInlineEdit(div, entry);
    });

    // Drag handlers
    div.addEventListener('dragstart', (e) => {
      e.dataTransfer!.setData('text/plain', String(i));
      e.dataTransfer!.effectAllowed = 'move';
    });
    div.addEventListener('dragover', (e) => {
      e.preventDefault();
      e.dataTransfer!.dropEffect = 'move';
      div.classList.add('drag-over');
    });
    div.addEventListener('dragleave', () => div.classList.remove('drag-over'));
    div.addEventListener('drop', (e) => {
      e.preventDefault();
      div.classList.remove('drag-over');
      const fromIdx = parseInt(e.dataTransfer!.getData('text/plain'), 10);
      queueStore.move(fromIdx, i);
    });

    queueList.appendChild(div);
  }
}

function startInlineEdit(div: HTMLDivElement, entry: QueueEntry): void {
  if (queueRunner.isRunning) return;
  div.classList.add('editing');

  const fields = document.createElement('div');
  fields.className = 'queue-edit-fields';

  if (entry.type === 'move') {
    const p = entry.params;
    fields.innerHTML = `
      <input type="number" value="${p.distanceDeg}" step="0.1" title="Distance (°)" />
      <input type="number" value="${p.speedRPM}" step="1" min="1" title="Speed (RPM)" />
      <input type="number" value="${p.accelDPS2}" step="10" min="10" title="Accel (°/s²)" />
      <button>OK</button>
    `;
    const inputs = fields.querySelectorAll('input');
    fields.querySelector('button')!.addEventListener('click', () => {
      queueStore.updateEntry(entry.id, {
        distanceDeg: parseFloat(inputs[0].value) || 0,
        speedRPM:    parseFloat(inputs[1].value) || 112,
        accelDPS2:   parseFloat(inputs[2].value) || 500,
      });
    });
  } else {
    fields.innerHTML = `
      <input type="number" value="${entry.params.durationMs}" step="100" min="0" title="Duration (ms)" />
      <button>OK</button>
    `;
    const input = fields.querySelector('input')!;
    fields.querySelector('button')!.addEventListener('click', () => {
      queueStore.updateEntry(entry.id, { durationMs: parseFloat(input.value) || 500 });
    });
  }

  // Escape to cancel
  fields.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') renderQueue();
  });

  div.appendChild(fields);
}

function renderBlockStats(): void {
  const stats = store.blockStats;
  if (stats.length === 0) { blockStatsSection.classList.add('hidden'); return; }

  // Also snapshot the final block (STOP captures it)
  // The last block's stats are the current moveStats (not yet snapshotted by BLOCK_DONE)
  const finalStats = store.moveStats;

  blockStatsSection.classList.remove('hidden');
  blockStatsList.innerHTML = '';

  // Combine snapshotted blocks + final block
  const allBlocks = [...stats];
  // Add final block only if BLOCK_DONE didn't already capture it
  if (allBlocks.length === 0 || allBlocks[allBlocks.length - 1].blockIndex < allBlocks.length) {
    allBlocks.push({
      blockIndex: allBlocks.length,
      peakLagCounts: finalStats.peakLagCounts,
      meanLagCounts: finalStats.meanLagCounts,
      skippedCounts: finalStats.skippedCounts,
      lagJitter: finalStats.lagJitter,
      effortPct: finalStats.effortPct,
      peakLagDeg: finalStats.peakLagDeg,
      meanLagDeg: finalStats.meanLagDeg,
      skippedDeg: finalStats.skippedDeg,
      lagJitterDeg: finalStats.lagJitterDeg,
    });
  }

  for (const bs of allBlocks) {
    const row = document.createElement('div');
    row.className = 'block-stat-row';
    row.innerHTML = `
      <span class="block-label">M${bs.blockIndex + 1}</span>
      <span class="block-values">
        lag ${bs.peakLagDeg.toFixed(2)}°pk / ${bs.meanLagDeg.toFixed(2)}°avg
        &nbsp; loss ${bs.skippedDeg.toFixed(2)}°
        &nbsp; effort ${bs.effortPct.toFixed(1)}%
      </span>
    `;
    blockStatsList.appendChild(row);
  }
}

queueStore.onChange = renderQueue;

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

// ── Jog controls ─────────────────────────────────────────────────────────────

const DEG_PER_FULL_STEP = 1.8;
let selectedJogSteps = 10;

jogStepBtns.forEach(btn => {
  btn.addEventListener('click', () => {
    jogStepBtns.forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    selectedJogSteps = parseInt(btn.dataset.steps!, 10);
  });
});

function sendJog(sign: 1 | -1): void {
  const distDeg  = sign * selectedJogSteps * DEG_PER_FULL_STEP;
  const speedRPM = parseFloat(inpJogSpeed.value);
  const accelDPS = parseFloat(inpAccel.value);

  const cmd = moveCommand(
    degToMicrosteps(distDeg, currentMicrosteps),
    rpmToStepsPerSec(speedRPM, currentMicrosteps),
    degPerSec2ToStepsPerSec2(accelDPS, currentMicrosteps),
    false,
  );
  conn.write(cmd).catch((err: unknown) => {
    alert(`Jog error: ${err instanceof Error ? err.message : String(err)}`);
  });
}

btnJogNeg.addEventListener('click', () => sendJog(-1));
btnJogPos.addEventListener('click', () => sendJog(1));

// ── Settings: apply helpers ───────────────────────────────────────────────────

function sendCmd(obj: Record<string, unknown>): void {
  conn.write(JSON.stringify(obj) + '\n').catch(() => undefined);
}

btnApplyPd.addEventListener('click', () => {
  currentKp = parseFloat(setKp.value);
  currentKd = parseFloat(setKd.value);
  sendCmd({ cmd: 'set_pd', kp: currentKp, kd: currentKd, d_alpha: parseFloat(setDAlpha.value) });
  sendCmd({ cmd: 'set_jerk', value: parseFloat(setJerkInput.value) });
  sendCmd({ cmd: 'set_phase_lead', kv: parseFloat(setKv.value) });
});
btnSave.addEventListener('click', () => {
  sendCmd({ cmd: 'save' });
});

// ── Auto-tuner ───────────────────────────────────────────────────────────────

btnAutotuneStop.addEventListener('click', () => {
  activeTuner?.abort();
});

btnAutotune.addEventListener('click', () => {
  const config = {
    distanceDeg:  parseFloat(inpDistance.value),
    speedRpm:     parseFloat(inpSpeed.value),
    accelDps2:    parseFloat(inpAccel.value),
    microsteps:   currentMicrosteps,
    currentKp:    parseFloat(setKp.value),
    currentKd:    parseFloat(setKd.value),
    currentKv:    parseFloat(setKv.value),
    currentJerk:  parseFloat(setJerkInput.value),
    dAlpha:       parseFloat(setDAlpha.value),
  };

  // Reset UI
  atLogList.innerHTML    = '';
  atSummaryList.innerHTML = '';
  atProgressFill.style.width = '0%';
  atParamLabel.textContent = 'Starting…';
  atIterLabel.textContent  = `Step 0/${16}`;
  autotuneProgress.classList.remove('hidden');
  autotuneResults.classList.add('hidden');

  // Lock controls during tuning
  btnAutotune.disabled = true;
  setSettingsEnabled(false);
  setMotionEnabled(false);

  const onProgress = (p: TunerProgress) => {
    atParamLabel.textContent = p.paramLabel;
    atIterLabel.textContent  = `Step ${p.step}/${p.totalSteps}`;
    atProgressFill.style.width = `${(p.step / p.totalSteps) * 100}%`;
    const li = document.createElement('li');
    li.textContent = p.logLine;
    atLogList.prepend(li);
    // Keep last 8 lines
    while (atLogList.children.length > 8) atLogList.removeChild(atLogList.lastChild!);
  };

  activeTuner = new AutoTuner(conn, sendCmd, onProgress);

  activeTuner.tune(config).then((results: TuneResult[]) => {
    // Update GUI inputs with best values
    for (const r of results) {
      if (r.param === 'Kp')   setKp.value        = r.to.toFixed(2);
      if (r.param === 'Kv')   setKv.value        = r.to.toFixed(3);
      if (r.param === 'Jerk') setJerkInput.value = r.to.toFixed(0);
    }

    // Show summary
    autotuneResults.classList.remove('hidden');
    atSummaryList.innerHTML = '';
    if (results.length === 0) {
      const li = document.createElement('li');
      li.textContent = 'Aborted — no results';
      li.className = 'unchanged';
      atSummaryList.appendChild(li);
    } else {
      for (const r of results) {
        const improved = r.metricAfter < r.metricBefore * 0.99;
        const li = document.createElement('li');
        li.className = improved ? 'improved' : 'unchanged';
        li.textContent =
          `${r.param}: ${r.fromDisplay} → ${r.toDisplay}` +
          `  (${r.metricLabel}: ${r.metricBefore.toFixed(1)} → ${r.metricAfter.toFixed(1)})`;
        atSummaryList.appendChild(li);
      }
    }

    atParamLabel.textContent = 'Done';
    atProgressFill.style.width = '100%';
  }).catch((err: unknown) => {
    autotuneResults.classList.remove('hidden');
    atSummaryList.innerHTML = '';
    const li = document.createElement('li');
    li.className = 'unchanged';
    li.textContent = `Error: ${err instanceof Error ? err.message : String(err)}`;
    atSummaryList.appendChild(li);
    atParamLabel.textContent = 'Failed';
  }).finally(() => {
    activeTuner = null;
    setSettingsEnabled(true);
    setMotionEnabled(true);
    btnAutotune.disabled = false;
  });
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
  setKp.value     = pkt.kp.toFixed(3);
  setKd.value     = pkt.kd.toFixed(4);
  setDAlpha.value     = pkt.d_alpha.toFixed(2);
  setJerkInput.value  = pkt.jerk.toFixed(0);
  setKv.value         = pkt.kv.toFixed(5);
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

  btnHome.disabled = !settingsReceived || pkt.isRunning || pkt.isHoming;

  // Derive motion state from authoritative STATUS flags.
  // Guard: a STATUS packet sent just before the planner finished can arrive
  // after the STOP packet (different send tasks, 100ms vs 1kHz). Suppress
  // isRunning=true for 500ms after a STOP so it cannot flip 'correcting' back
  // to 'moving'.
  const recentStop = stopReceivedAt > 0 && (Date.now() - stopReceivedAt) < 500;
  if (pkt.isRunning && !recentStop) {
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
    teleMeas.textContent    = `${encToDeg(packet.meas).toFixed(1)}°`;
    teleTarget.textContent  = `${encToDeg(packet.target).toFixed(1)}°`;
    teleVel.textContent     = `${encPerSecToRPM(packet.vel).toFixed(1)} RPM`;
    teleLag.textContent     = `${encToDeg(packet.lag).toFixed(2)}°`;

    // Chart accumulates during moving/correcting, freezes when settled
    const shouldChart = motionState === 'moving' || motionState === 'correcting';
    store.push(packet, shouldChart);
    if (shouldChart) _chartDirty = true;

    _chartDirty = true;

    // Update hold accuracy gauge if in hold phase
    if (motionState === 'correcting' || motionState === 'holding') {
      updateDeviationGauge(packet.lag);
      holdPeakDev.textContent = `${store.moveStats.holdPeakDevDeg.toFixed(2)}°`;
    }

  } else if (packet.type === 'stop') {
    stopsReceived++;
    teleMeas.textContent = `${encToDeg(packet.pos).toFixed(1)}°`;

    // Enter hold phase for deviation tracking
    const isNormalStop = !packet.reason.includes('Fault') && !packet.reason.includes('E-STOP');
    stopReceivedAt = Date.now();  // always stamp — guards STATUS race for both normal and fault stops
    if (isNormalStop) {
      store.enterHoldPhase();
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
    perfMeanLag.textContent   = `${ms.meanLagDeg.toFixed(2)}°`;
    perfStepLoss.textContent  = `${ms.skippedDeg.toFixed(2)}°`;
    perfLagJitter.textContent = `${ms.lagJitterDeg.toFixed(2)}°`;
    perfEffort.textContent    = `${ms.effortPct.toFixed(1)}%`;
    perfMaxLag.classList.remove('dimmed');
    perfMeanLag.classList.remove('dimmed');
    perfStepLoss.classList.remove('dimmed');
    perfLagJitter.classList.remove('dimmed');
    perfEffort.classList.remove('dimmed');

    // Notify queue runner (no-op if not running a queue)
    queueRunner.onStop(packet.reason);

    updateLinkStats();
  }
};

// ── Block-done handler (chain boundary marker) ──────────────────────────────

conn.onBlockDone = (pkt: BlockDonePacket): void => {
  store.snapshotBlock(pkt.blockIndex);
  queueRunner.onBlockDone(pkt.blockIndex);
};

// ── Homing result handler ────────────────────────────────────────────────────

conn.onHomingDone = (pkt: HomingDonePacket): void => {
  // All SG values are 0–1023 raw; halve to convert to SGTHRS (0–255) space.
  // GUI trigger inputs are already in SGTHRS space.
  const thresh1 = parseInt(setHomeSg1.value, 10) || 65;
  const thresh2 = parseInt(setHomeSg2.value, 10) || 10;

  const sgF  = pkt.sgMinFast  === 0xFFFF ? null : Math.round(pkt.sgMinFast  / 2);
  const sgS  = pkt.sgMinSlow  === 0xFFFF ? null : Math.round(pkt.sgMinSlow  / 2);
  const sgBF = pkt.sgBaseFast === 0xFFFF ? null : Math.round(pkt.sgBaseFast / 2);
  const sgBS = pkt.sgBaseSlow === 0xFFFF ? null : Math.round(pkt.sgBaseSlow / 2);

  // "Fast: approach 98 → trigger 65 → min 67"
  const statsLine = (label: string, base: number | null, thresh: number, min: number | null): string | null => {
    if (min === null) return null;
    const parts: string[] = [];
    if (base !== null) parts.push(`approach ${base}`);
    parts.push(`trigger ${thresh}`);
    parts.push(`~min ${min}`);
    return `${label}: ${parts.join(' \u2192 ')}`;
  };

  const fastLine = statsLine('Fast', sgBF, thresh1, sgF);
  const slowLine = statsLine('Slow', sgBS, thresh2, sgS);

  let cls: string;
  const lines: string[] = [];

  switch (pkt.result) {
    case 0:  // success
      cls = 'ok';
      lines.push('Homed');
      teleMeas.textContent   = '0.0°';
      teleTarget.textContent = '0.0°';
      teleVel.textContent    = '0.0 RPM';
      teleLag.textContent    = '0.00°';
      break;
    case 1:  // timeout — motor never reached endstop
      cls = 'error';
      lines.push('Timeout \u2014 no contact. Is endstop reachable?');
      break;
    case 2:  // instant stall — fired before minimum travel
      cls = 'warn';
      lines.push(sgS !== null
        ? 'Premature stall (slow) \u2014 lower Slow Home Trigger'
        : 'Premature stall (fast) \u2014 lower Fast Home Trigger or check start position');
      break;
    case 3:  // grinding — encoder stalled, SG never triggered
      cls = 'error';
      lines.push(sgS !== null
        ? 'Grinding (slow) \u2014 raise Slow Home Trigger'
        : 'Grinding (fast) \u2014 raise Fast Home Trigger');
      break;
    case 4:  // e-stop
      cls = 'error';
      lines.push('E-Stop during homing');
      break;
    default:
      cls = 'error';
      lines.push(pkt.errorMsg || 'Unknown homing error');
  }

  if (fastLine) lines.push(fastLine);
  if (slowLine) lines.push(slowLine);

  homingResultDiv.innerHTML = lines.join('<br>');
  homingResultDiv.className = `homing-result ${cls}`;
};

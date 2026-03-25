/**
 * Three-phase auto-tuner for Kp, Kv, and Jerk Ramp Time.
 *
 * Each Kp and Jerk candidate is scored across three move profiles derived
 * from the user's configured move:
 *   gentle     — 0.3× speed, 0.3× accel  (slow/smooth tracking)
 *   normal     — 1.0× speed, 1.0× accel  (user's baseline)
 *   aggressive — 1.0× speed, 3.0× accel  (hard start/stop)
 *
 * The aggregate score is the mean of all profiles that complete successfully.
 * A timeout on any profile triggers E-STOP and that profile is skipped.
 *
 * Phase 1 — Kp sweep        7 candidates × 2 profiles  score = mean |lag| + settle
 * Phase 2 — Kv analytic     1 coasting move             score = analytical
 * Phase 3 — Ramp-time sweep 7 candidates × 3 profiles  score = vel-residual σ
 *
 * Total: 15 candidate steps (progress bar), ~43 individual moves internally.
 *
 * Jerk ramp time (ms): Jerk = Accel / T_ramp, so the same ms value is correct
 * regardless of the accel setting. Candidates are expressed in ms.
 */

import { moveCommand, type Packet, type TelemetryUpdate } from './protocol';
import {
  rpmToStepsPerSec,
  degPerSec2ToStepsPerSec2,
  degToMicrosteps,
  COUNTS_PER_REV,
} from './units';

export interface TuneConfig {
  distanceDeg:  number;
  speedRpm:     number;
  accelDps2:    number;
  microsteps:   number;
  currentKp:    number;
  currentKd:    number;
  currentKv:    number;
  currentJerk:  number;
  dAlpha:       number;
}

export interface TuneResult {
  param:        string;
  from:         number;
  to:           number;
  fromDisplay:  string;
  toDisplay:    string;
  metricLabel:  string;
  metricBefore: number;
  metricAfter:  number;
}

export interface TunerProgress {
  paramLabel:  string;
  step:        number;
  totalSteps:  number;
  logLine:     string;
}

type ProgressCb = (p: TunerProgress) => void;
type SendFn     = (obj: Record<string, unknown>) => void;

interface ConnLike {
  onPacket: ((pkt: Packet) => void) | null;
  write:    (data: string) => Promise<void>;
}

// Kp profiles: gentle + normal only.
// Aggressive accel is intentionally excluded — it biases Kp toward higher
// values that track hard moves well but are underdamped in hold mode.
const KP_PROFILE_DEFS = [
  { label: 'gentle', speedMult: 0.3, accelMult: 0.3 },
  { label: 'normal', speedMult: 1.0, accelMult: 1.0 },
] as const;

// Jerk profiles: full range including aggressive, because resonance excitation
// depends strongly on accel magnitude.
const JERK_PROFILE_DEFS = [
  { label: 'gentle',     speedMult: 0.3, accelMult: 0.3 },
  { label: 'normal',     speedMult: 1.0, accelMult: 1.0 },
  { label: 'aggressive', speedMult: 1.0, accelMult: 3.0 },
] as const;

// Post-move window to measure settling quality for Kp scoring
const KP_SETTLE_MS = 400;

// Absolute Kp candidates — not relative to currentKp.
// Using relative multipliers caused compounding across multiple tuner runs
// (each run could 3× the previous value, reaching dangerously high gains).
const KP_CANDIDATES = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0];
// Ramp time candidates in ms. Jerk = Accel / T_ramp, so these auto-scale with accel.
const RAMP_TIME_CANDIDATES = [0, 50, 100, 150, 200, 300, 500]; // ms (0 = auto ~10 ms)
// Progress bar steps: one per candidate + one for Kv analytical
const TOTAL_STEPS = KP_CANDIDATES.length + 1 + RAMP_TIME_CANDIDATES.length; // 15

const MOVE_TIMEOUT_MS  = 12_000;
const FAIL_RECOVERY_MS = 1_000;

function sleep(ms: number): Promise<void> {
  return new Promise(r => setTimeout(r, ms));
}

export class AutoTuner {
  private _aborted = false;
  private _step    = 0;

  constructor(
    private readonly conn:       ConnLike,
    private readonly send:       SendFn,
    private readonly onProgress: ProgressCb,
  ) {}

  abort(): void { this._aborted = true; }

  async tune(config: TuneConfig): Promise<TuneResult[]> {
    this._aborted = false;
    this._step    = 0;
    const results: TuneResult[] = [];

    const kpResult = await this._tuneKp(config);
    if (kpResult) {
      results.push(kpResult);
      config.currentKp = kpResult.to;
    }
    if (this._aborted) return results;

    const kvResult = await this._tuneKv(config);
    if (kvResult) {
      results.push(kvResult);
      config.currentKv = kvResult.to;
    }
    if (this._aborted) return results;

    const jerkResult = await this._tuneJerk(config);
    if (jerkResult) results.push(jerkResult);

    return results;
  }

  // ── Phase 1: Kp sweep ──────────────────────────────────────────────────────

  private async _tuneKp(config: TuneConfig): Promise<TuneResult | null> {
    const label = 'Kp (position gain)';
    let bestKp    = config.currentKp;
    let bestScore = Infinity;
    let scoreBefore = Infinity;

    for (const kp of KP_CANDIDATES) {
      if (this._aborted) break;
      this._step++;

      this.send({ cmd: 'set_pd', kp, kd: config.currentKd, d_alpha: config.dAlpha });
      await sleep(50);

      const { avg, scores } = await this._scoreProfiles(
        config, this._scoreKpWithSettle.bind(this), KP_PROFILE_DEFS);
      const scoreStr = scores.map(s => isFinite(s) ? s.toFixed(1) : '×').join('/');

      this.onProgress({
        paramLabel: label, step: this._step, totalSteps: TOTAL_STEPS,
        logLine: `Kp ${kp.toFixed(2)} → lag+settle ${isFinite(avg) ? avg.toFixed(1) : '—'} [${scoreStr}]${avg < bestScore ? ' ↑' : ''}`,
      });

      if (kp === config.currentKp) scoreBefore = avg;
      if (avg < bestScore) { bestScore = avg; bestKp = kp; }
    }

    this.send({ cmd: 'set_pd', kp: bestKp, kd: config.currentKd, d_alpha: config.dAlpha });

    return {
      param: 'Kp', from: config.currentKp, to: bestKp,
      fromDisplay: config.currentKp.toFixed(2), toDisplay: bestKp.toFixed(2),
      metricLabel: 'mean |lag|',
      metricBefore: isFinite(scoreBefore) ? scoreBefore : bestScore,
      metricAfter:  bestScore,
    };
  }

  // ── Phase 2: Kv analytical ─────────────────────────────────────────────────

  private async _tuneKv(config: TuneConfig): Promise<TuneResult | null> {
    const label = 'Kv (velocity feedforward)';
    this._step++;

    this.send({ cmd: 'set_phase_lead', kv: 0 });
    await sleep(50);

    const coastCfg = this._withGuaranteedCoast(config);

    let packets: TelemetryUpdate[];
    try {
      packets = await this._collectMove(coastCfg);
    } catch {
      this.onProgress({
        paramLabel: label, step: this._step, totalSteps: TOTAL_STEPS,
        logLine: 'Kv → timeout, keeping previous value',
      });
      await sleep(FAIL_RECOVERY_MS);
      this.send({ cmd: 'set_phase_lead', kv: config.currentKv });
      return null;
    }

    const coast = packets.filter(p => Math.abs(p.accel) < 5 && Math.abs(p.vel) > 10);

    let kvOpt     = config.currentKv;
    let lagBefore = NaN;

    if (coast.length >= 5) {
      const meanLag = coast.reduce((s, p) => s + p.lag, 0) / coast.length;
      const meanVel = coast.reduce((s, p) => s + Math.abs(p.vel), 0) / coast.length;
      lagBefore = meanLag;
      if (meanVel > 1) {
        // At constant velocity, lag = vel/Kp - Kv*vel → Kv_opt = lag/vel
        kvOpt = Math.max(0, Math.min(0.02, meanLag / meanVel));
        kvOpt = Math.round(kvOpt * 1000) / 1000;
      }
    }

    this.onProgress({
      paramLabel: label, step: this._step, totalSteps: TOTAL_STEPS,
      logLine: `Kv: coast lag ${isFinite(lagBefore) ? lagBefore.toFixed(1) : '—'} enc (${coast.length} samples) → ${kvOpt.toFixed(3)}`,
    });

    this.send({ cmd: 'set_phase_lead', kv: kvOpt });

    return {
      param: 'Kv', from: config.currentKv, to: kvOpt,
      fromDisplay: config.currentKv.toFixed(3), toDisplay: kvOpt.toFixed(3),
      metricLabel: 'coast lag',
      metricBefore: isFinite(lagBefore) ? Math.abs(lagBefore) : 0,
      metricAfter:  0,
    };
  }

  // ── Phase 3: Jerk ramp-time sweep ─────────────────────────────────────────

  private async _tuneJerk(config: TuneConfig): Promise<TuneResult | null> {
    const label = 'Jerk Ramp (ms)';
    let bestMs    = config.currentJerk;  // currentJerk is now ramp time in ms
    let bestScore = Infinity;
    let scoreBefore = Infinity;

    for (const ms of RAMP_TIME_CANDIDATES) {
      if (this._aborted) break;
      this._step++;

      this.send({ cmd: 'set_jerk', value: ms });  // firmware receives ms directly
      await sleep(50);

      const { avg, scores } = await this._scoreProfiles(
        config, this._scoreJerk.bind(this), JERK_PROFILE_DEFS);
      const scoreStr = scores.map(s => isFinite(s) ? s.toFixed(1) : '×').join('/');

      this.onProgress({
        paramLabel: label, step: this._step, totalSteps: TOTAL_STEPS,
        logLine: `Ramp ${ms === 0 ? 'auto' : ms + 'ms'} → vel-res σ ${isFinite(avg) ? avg.toFixed(1) : '—'} [${scoreStr}]${avg < bestScore ? ' ↑' : ''}`,
      });

      if (ms === config.currentJerk) scoreBefore = avg;
      if (avg < bestScore) { bestScore = avg; bestMs = ms; }
    }

    this.send({ cmd: 'set_jerk', value: bestMs });

    return {
      param: 'Jerk', from: config.currentJerk, to: bestMs,
      fromDisplay: config.currentJerk === 0 ? 'auto' : `${config.currentJerk} ms`,
      toDisplay:   bestMs === 0 ? 'auto' : `${bestMs} ms`,
      metricLabel: 'vel-res σ',
      metricBefore: isFinite(scoreBefore) ? scoreBefore : bestScore,
      metricAfter:  bestScore,
    };
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  /**
   * Run the given set of profiles for a candidate and return per-profile scores
   * plus their mean. A failed/timeout profile records NaN and is excluded.
   */
  private async _scoreProfiles(
    config: TuneConfig,
    scoreFn: (pkts: TelemetryUpdate[], postPkts: TelemetryUpdate[]) => number,
    profileDefs: ReadonlyArray<{ label: string; speedMult: number; accelMult: number }>,
  ): Promise<{ scores: number[]; avg: number }> {
    const scores: number[] = [];

    for (const def of profileDefs) {
      if (this._aborted) break;
      const profile: TuneConfig = {
        ...config,
        speedRpm:  Math.max(30, config.speedRpm * def.speedMult),
        accelDps2: config.accelDps2 * def.accelMult,
      };
      try {
        const { movePkts, postPkts } = await this._collectMoveWithSettle(profile, KP_SETTLE_MS);
        scores.push(scoreFn(movePkts, postPkts));
      } catch {
        scores.push(NaN);
        await sleep(FAIL_RECOVERY_MS);
      }
    }

    const finite = scores.filter(s => isFinite(s));
    const avg = finite.length > 0
      ? finite.reduce((a, b) => a + b, 0) / finite.length
      : Infinity;
    return { scores, avg };
  }

  /** Scale distanceDeg up so there is at least 0.5 rev of coasting. */
  private _withGuaranteedCoast(config: TuneConfig): TuneConfig {
    const velEnc = config.speedRpm * COUNTS_PER_REV / 60;
    const accEnc = config.accelDps2 * COUNTS_PER_REV / 360;
    const tRamp  = velEnc / accEnc;
    const dRamp  = 0.5 * accEnc * tRamp * tRamp;
    const minEnc = 2 * dRamp + COUNTS_PER_REV * 0.5;
    const minDeg = Math.min(minEnc * 360 / COUNTS_PER_REV, 1800);
    return { ...config, distanceDeg: Math.max(minDeg, 360) };
  }

  /**
   * Run a move and optionally collect postStopMs of UPDATE packets after the
   * STOP packet (for settling analysis). Returns { movePkts, postPkts }.
   */
  private _collectMoveWithSettle(
    config: TuneConfig,
    postStopMs = 0,
  ): Promise<{ movePkts: TelemetryUpdate[]; postPkts: TelemetryUpdate[] }> {
    const dist  = degToMicrosteps(config.distanceDeg, config.microsteps);
    const speed = rpmToStepsPerSec(config.speedRpm, config.microsteps);
    const accel = degPerSec2ToStepsPerSec2(config.accelDps2, config.microsteps);
    const cmd   = moveCommand(dist, speed, accel);

    return new Promise((resolve, reject) => {
      const movePkts: TelemetryUpdate[] = [];
      const postPkts: TelemetryUpdate[] = [];
      let   inPostStop = false;
      const prevHandler = this.conn.onPacket;

      const cleanup = () => { this.conn.onPacket = prevHandler; };

      const moveTimer = setTimeout(() => {
        cleanup();
        this.send({ cmd: 'estop' });
        reject(new Error('timeout'));
      }, MOVE_TIMEOUT_MS);

      this.conn.onPacket = (pkt: Packet) => {
        prevHandler?.(pkt);

        if (inPostStop) {
          if (pkt.type === 'update') postPkts.push(pkt);
          return;
        }

        if (pkt.type === 'update') {
          movePkts.push(pkt);
        } else if (pkt.type === 'stop') {
          clearTimeout(moveTimer);
          if (pkt.reason !== 'Completed') {
            cleanup();
            reject(new Error(`stopped: ${pkt.reason}`));
            return;
          }
          if (postStopMs <= 0) {
            cleanup();
            resolve({ movePkts, postPkts });
            return;
          }
          // Collect post-stop packets then resolve
          inPostStop = true;
          setTimeout(() => {
            cleanup();
            resolve({ movePkts, postPkts });
          }, postStopMs);
        }
      };

      this.conn.write(cmd).catch((err: unknown) => {
        clearTimeout(moveTimer);
        cleanup();
        reject(err);
      });
    });
  }

  /** Convenience wrapper — used by Kv and Jerk which don't need post-stop data. */
  private async _collectMove(config: TuneConfig): Promise<TelemetryUpdate[]> {
    const { movePkts } = await this._collectMoveWithSettle(config, 0);
    return movePkts;
  }

  // ── Scoring ────────────────────────────────────────────────────────────────

  /**
   * Kp score = mean |lag| during move + mean |lag| during post-stop window.
   * Combining both penalises Kp values that track well during the move but
   * oscillate during settling — which is exactly what too-high Kp does.
   */
  private _scoreKpWithSettle(movePkts: TelemetryUpdate[], postPkts: TelemetryUpdate[]): number {
    if (movePkts.length < 3) return Infinity;
    const moveLag  = movePkts.reduce((s, p) => s + Math.abs(p.lag), 0) / movePkts.length;
    const postLag  = postPkts.length >= 3
      ? postPkts.reduce((s, p) => s + Math.abs(p.lag), 0) / postPkts.length
      : moveLag; // no post data — assume settling equals move lag
    // Weight settling equally with tracking
    return (moveLag + postLag) / 2;
  }

  /**
   * Std-dev of velocity residual (mvel − vel) across the full move.
   * Subtracting the commanded velocity removes the expected ramp shape,
   * leaving only vibration/noise. Lower = smoother velocity tracking.
   */
  private _scoreJerk(movePkts: TelemetryUpdate[], _postPkts: TelemetryUpdate[]): number {
    if (movePkts.length < 3) return Infinity;
    const res  = movePkts.map(p => p.mvel - p.vel);
    const mean = res.reduce((a, b) => a + b, 0) / res.length;
    const variance = res.reduce((a, b) => a + (b - mean) ** 2, 0) / res.length;
    return Math.sqrt(variance);
  }
}

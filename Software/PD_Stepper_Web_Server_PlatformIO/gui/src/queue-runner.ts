/**
 * Queue execution state machine.
 *
 * Splits the queue into chain groups (consecutive moves vs dwells),
 * sends each move group as a seamless chain, and handles dwells
 * between groups.
 */

import { type QueueStore, type ChainGroup } from './queue-store';
import { moveCommand, loopCommand, loopStopCommand } from './protocol';
import { degToMicrosteps, rpmToStepsPerSec, degPerSec2ToStepsPerSec2 } from './units';

export type RunnerState = 'idle' | 'sending' | 'waiting_stop' | 'dwelling';

export interface RunnerCallbacks {
  /** Write a string command to the serial connection. */
  write: (cmd: string) => Promise<void>;
  /** Get current microstep setting for unit conversion. */
  getMicrosteps: () => number;
  /** Current encoder position in degrees — used as base for the first move in a
   *  chain so relative moves are accumulated as a float and sent as absolute,
   *  avoiding per-move microstep rounding accumulation. */
  getStartPosDeg: () => number;
  /** Axis invert factor: 1 (normal) or -1 (inverted). Applied to user-entered
   *  distances before converting to firmware microsteps. */
  getInvertFactor: () => 1 | -1;
  /** Report the accumulated commanded position (degrees) for DRO display. */
  onCommandedPos: (deg: number) => void;
  /** Called when runner state changes. */
  onStateChange: (state: RunnerState) => void;
  /** Called when the entire queue finishes (all groups done). */
  onComplete: () => void;
  /** Called when a fault aborts execution. */
  onAbort: (reason: string) => void;
}

export class QueueRunner {
  private _state: RunnerState = 'idle';
  private _groups: ChainGroup[] = [];
  private _groupIdx = 0;
  /** Index into the flat queue entries where the current chain group starts. */
  private _entryOffset = 0;
  private _dwellTimer: ReturnType<typeof setTimeout> | null = null;

  /** When true, the queue runs as a firmware-side continuous loop. */
  loop = false;

  /** True when running in firmware loop mode (so onStop/stopLoop behave correctly). */
  private _firmwareLoop = false;

  /** Float position accumulator carried across chain groups within one loop iteration.
   *  null = re-sample from encoder at start of first group of each iteration. */
  private _loopAccDeg: number | null = null;

  constructor(
    private _store: QueueStore,
    private _cb: RunnerCallbacks,
  ) {}

  get state(): RunnerState { return this._state; }
  get isRunning(): boolean { return this._state !== 'idle'; }
  get isFirmwareLoop(): boolean { return this._firmwareLoop; }

  /** Swap the backing store. Only call when state === 'idle'. */
  setStore(store: QueueStore): void {
    this._store = store;
  }

  /** Start executing the queue from the beginning. */
  start(): void {
    if (this._state !== 'idle') return;
    if (this._store.length === 0) return;

    this._store.resetAllStatus();

    if (this.loop) {
      this._startFirmwareLoop();
      return;
    }

    this._firmwareLoop = false;
    this._groups = this._store.chainGroups();
    this._groupIdx = 0;
    this._entryOffset = 0;
    this._loopAccDeg = null;  // re-sample encoder at start of first group
    this._sendCurrentGroup();
  }

  /** Send a single firmware-side loop command with all move entries.
   *  Commands are sent as relative distances so the firmware can replay
   *  the pattern cyclically — absolute targets would collapse to zero
   *  displacement on the second iteration for non-returning patterns. */
  private _startFirmwareLoop(): void {
    this._firmwareLoop = true;
    const usteps = this._cb.getMicrosteps();

    // Collect all move entries as firmware commands (relative distances)
    const inv = this._cb.getInvertFactor();
    const commands: Array<{ distance: number; speed: number; accel: number; abs: boolean }> = [];
    for (const entry of this._store.entries) {
      if (entry.type !== 'move') continue;  // skip dwells in firmware loop mode
      commands.push({
        distance: inv * degToMicrosteps(entry.params.distanceDeg, usteps),
        speed:    rpmToStepsPerSec(entry.params.speedRPM, usteps),
        accel:    degPerSec2ToStepsPerSec2(entry.params.accelDPS2, usteps),
        abs:      entry.params.absolute,
      });
      this._store.setStatus(entry.id, 'running');
    }

    if (commands.length === 0) {
      this._firmwareLoop = false;
      return;
    }

    this._setState('sending');
    this._cb.write(loopCommand(commands)).then(() => {
      this._setState('waiting_stop');
    }).catch((err) => {
      this._firmwareLoop = false;
      this.abort(`Send error: ${err instanceof Error ? err.message : String(err)}`);
    });
  }

  /** Stop a running firmware loop. Sends loop_stop; firmware decelerates and sends STOP. */
  stopFirmwareLoop(): void {
    if (!this._firmwareLoop || this._state !== 'waiting_stop') return;
    this._cb.write(loopStopCommand()).catch(() => {});
  }

  /** Abort execution (e.g. E-Stop). Marks remaining entries as error. */
  abort(reason: string): void {
    if (this._state === 'idle') return;
    if (this._dwellTimer !== null) {
      clearTimeout(this._dwellTimer);
      this._dwellTimer = null;
    }
    this._firmwareLoop = false;
    // Mark any non-done entries as error
    for (const entry of this._store.entries) {
      if (entry.status === 'pending' || entry.status === 'running') {
        this._store.setStatus(entry.id, 'error');
      }
    }
    this._setState('idle');
    this._cb.onAbort(reason);
  }

  /** Called by main.ts when a BLOCK_DONE packet arrives during chain execution. */
  onBlockDone(blockIndex: number): void {
    if (this._state !== 'waiting_stop') return;
    if (this._firmwareLoop) return;  // firmware manages per-block progress internally
    const group = this._groups[this._groupIdx];
    if (group.type !== 'moves') return;

    // Mark completed block's entry as done
    const entryIdx = this._entryOffset + blockIndex;
    const entries = this._store.entries;
    if (entryIdx < entries.length) {
      this._store.setStatus(entries[entryIdx].id, 'done');
    }
    // Mark next entry as running
    const nextIdx = entryIdx + 1;
    if (nextIdx < entries.length && entries[nextIdx].status === 'pending') {
      this._store.setStatus(entries[nextIdx].id, 'running');
    }
  }

  /** Called by main.ts when a STOP packet arrives. */
  onStop(reason: string): void {
    if (this._state !== 'waiting_stop') return;

    const isFault = reason !== 'Completed' && reason !== 'Stopped';

    // Firmware loop mode: all entries are managed as a single atomic operation
    if (this._firmwareLoop) {
      const status = isFault ? 'error' : 'done';
      for (const e of this._store.entries) {
        if (e.status === 'running' || e.status === 'pending') {
          this._store.setStatus(e.id, status);
        }
      }
      this._firmwareLoop = false;
      this._setState('idle');
      if (isFault) {
        this._cb.onAbort(reason);
      } else {
        this._cb.onComplete();
      }
      return;
    }

    const group = this._groups[this._groupIdx];

    // Mark last entry in current group as done (or error)
    if (group.type === 'moves') {
      const lastEntry = group.entries[group.entries.length - 1];
      this._store.setStatus(lastEntry.id, isFault ? 'error' : 'done');
    }

    if (isFault) {
      // Mark all remaining entries as error
      const remaining = this._store.entries.filter(
        e => e.status === 'pending' || e.status === 'running'
      );
      for (const e of remaining) this._store.setStatus(e.id, 'error');
      this._setState('idle');
      this._cb.onAbort(reason);
      return;
    }

    this._advanceToNextGroup();
  }

  private _setState(state: RunnerState): void {
    this._state = state;
    this._cb.onStateChange(state);
  }

  private _sendCurrentGroup(): void {
    const group = this._groups[this._groupIdx];
    if (!group) { this._finish(); return; }

    if (group.type === 'moves') {
      this._setState('sending');
      const usteps = this._cb.getMicrosteps();
      const inv = this._cb.getInvertFactor();

      // Mark first entry as running
      this._store.setStatus(group.entries[0].id, 'running');

      // Accumulate commanded position as float so per-move rounding does not
      // stack up. Each move (relative or absolute) is sent as an absolute
      // microstep target derived from the float accumulator, meaning the
      // rounding error across N moves is at most ½ microstep — not N × ½.
      // accDeg is in firmware space (raw encoder degrees); inv converts from user space.
      if (this._loopAccDeg === null) {
        this._loopAccDeg = this._cb.getStartPosDeg();
      }
      let accDeg = this._loopAccDeg;

      // Send all moves in the group
      const promises: Promise<void>[] = [];
      for (let i = 0; i < group.entries.length; i++) {
        const e = group.entries[i];
        const isLast = i === group.entries.length - 1;

        if (e.params.absolute) {
          accDeg = inv * e.params.distanceDeg;  // absolute: invert user target to firmware space
        } else {
          accDeg += inv * e.params.distanceDeg; // relative: accumulate with inversion
        }

        this._cb.onCommandedPos(accDeg);  // update DRO commanded display

        const cmd = moveCommand(
          degToMicrosteps(accDeg, usteps),  // always absolute after accumulation
          rpmToStepsPerSec(e.params.speedRPM, usteps),
          degPerSec2ToStepsPerSec2(e.params.accelDPS2, usteps),
          true,      // absolute — prevents microstep-rounding accumulation
          !isLast,   // chain=true for all except last
        );
        promises.push(this._cb.write(cmd));
      }

      this._loopAccDeg = accDeg;  // carry into next group within same loop iteration

      Promise.all(promises).then(() => {
        this._setState('waiting_stop');
      }).catch((err) => {
        this.abort(`Send error: ${err instanceof Error ? err.message : String(err)}`);
      });

    } else {
      // Dwell group
      this._store.setStatus(group.entry.id, 'running');
      this._setState('dwelling');
      this._dwellTimer = setTimeout(() => {
        this._dwellTimer = null;
        this._store.setStatus(group.entry.id, 'done');
        this._advanceToNextGroup();
      }, group.entry.params.durationMs);
    }
  }

  private _advanceToNextGroup(): void {
    // Compute entry offset for next group
    const group = this._groups[this._groupIdx];
    if (group.type === 'moves') {
      this._entryOffset += group.entries.length;
    } else {
      this._entryOffset += 1;
    }

    this._groupIdx++;
    if (this._groupIdx >= this._groups.length) {
      this._finish();
    } else {
      this._sendCurrentGroup();
    }
  }

  private _finish(): void {
    this._setState('idle');
    this._cb.onComplete();
  }
}

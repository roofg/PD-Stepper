/**
 * Queue execution state machine.
 *
 * Splits the queue into chain groups (consecutive moves vs dwells),
 * sends each move group as a seamless chain, and handles dwells
 * between groups.
 */

import { type QueueStore, type ChainGroup } from './queue-store';
import { moveCommand } from './protocol';
import { degToMicrosteps, rpmToStepsPerSec, degPerSec2ToStepsPerSec2 } from './units';

export type RunnerState = 'idle' | 'sending' | 'waiting_stop' | 'dwelling';

export interface RunnerCallbacks {
  /** Write a string command to the serial connection. */
  write: (cmd: string) => Promise<void>;
  /** Get current microstep setting for unit conversion. */
  getMicrosteps: () => number;
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

  /** When true, the queue restarts from the beginning after the last entry finishes. */
  loop = false;

  constructor(
    private _store: QueueStore,
    private _cb: RunnerCallbacks,
  ) {}

  get state(): RunnerState { return this._state; }
  get isRunning(): boolean { return this._state !== 'idle'; }

  /** Start executing the queue from the beginning. */
  start(): void {
    if (this._state !== 'idle') return;
    if (this._store.length === 0) return;

    this._store.resetAllStatus();
    this._groups = this._store.chainGroups();
    this._groupIdx = 0;
    this._entryOffset = 0;
    this._sendCurrentGroup();
  }

  /** Abort execution (e.g. E-Stop). Marks remaining entries as error. */
  abort(reason: string): void {
    if (this._state === 'idle') return;
    if (this._dwellTimer !== null) {
      clearTimeout(this._dwellTimer);
      this._dwellTimer = null;
    }
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

    const group = this._groups[this._groupIdx];
    const isFault = reason !== 'Completed';

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

      // Mark first entry as running
      this._store.setStatus(group.entries[0].id, 'running');

      // Send all moves in the group
      const promises: Promise<void>[] = [];
      for (let i = 0; i < group.entries.length; i++) {
        const e = group.entries[i];
        const isLast = i === group.entries.length - 1;
        const cmd = moveCommand(
          degToMicrosteps(e.params.distanceDeg, usteps),
          rpmToStepsPerSec(e.params.speedRPM, usteps),
          degPerSec2ToStepsPerSec2(e.params.accelDPS2, usteps),
          e.params.absolute,
          !isLast,  // chain=true for all except last
        );
        promises.push(this._cb.write(cmd));
      }

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
    if (this.loop) {
      this._cb.onComplete();
      this._state = 'idle';  // reset without firing onStateChange (keeps UI in running mode)
      this.start();
      return;
    }
    this._setState('idle');
    this._cb.onComplete();
  }
}

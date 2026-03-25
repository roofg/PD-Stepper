/**
 * Queue data model for move command sequences.
 *
 * Entries are either move commands or dwell (wait) commands.
 * Consecutive moves form chain groups for seamless execution;
 * dwells break the chain into separate groups.
 */

export interface MoveParams {
  distanceDeg: number;
  speedRPM:    number;
  accelDPS2:   number;
  absolute:    boolean;
}

export interface DwellParams {
  durationMs: number;
}

export type EntryStatus = 'pending' | 'running' | 'done' | 'error';

export interface MoveEntry {
  type:   'move';
  id:     string;
  params: MoveParams;
  status: EntryStatus;
}

export interface DwellEntry {
  type:   'dwell';
  id:     string;
  params: DwellParams;
  status: EntryStatus;
}

export type QueueEntry = MoveEntry | DwellEntry;

/** A chain group is either a run of consecutive moves or a single dwell. */
export type ChainGroup =
  | { type: 'moves'; entries: MoveEntry[] }
  | { type: 'dwell'; entry: DwellEntry };

let _nextId = 0;
function genId(): string { return `q${++_nextId}`; }

export class QueueStore {
  private _entries: QueueEntry[] = [];

  /** Callback fired after any mutation. */
  onChange: (() => void) | null = null;

  get entries(): readonly QueueEntry[] { return this._entries; }
  get length(): number { return this._entries.length; }

  addMove(params: MoveParams): void {
    this._entries.push({ type: 'move', id: genId(), params, status: 'pending' });
    this.onChange?.();
  }

  addDwell(params: DwellParams): void {
    this._entries.push({ type: 'dwell', id: genId(), params, status: 'pending' });
    this.onChange?.();
  }

  remove(id: string): void {
    const idx = this._entries.findIndex(e => e.id === id);
    if (idx >= 0) { this._entries.splice(idx, 1); this.onChange?.(); }
  }

  /** Move entry from one index to another (drag-and-drop reorder). */
  move(fromIdx: number, toIdx: number): void {
    if (fromIdx === toIdx) return;
    if (fromIdx < 0 || fromIdx >= this._entries.length) return;
    if (toIdx < 0 || toIdx >= this._entries.length) return;
    const [entry] = this._entries.splice(fromIdx, 1);
    this._entries.splice(toIdx, 0, entry);
    this.onChange?.();
  }

  updateEntry(id: string, partial: Partial<MoveParams & DwellParams>): void {
    const entry = this._entries.find(e => e.id === id);
    if (!entry) return;
    Object.assign(entry.params, partial);
    this.onChange?.();
  }

  setStatus(id: string, status: EntryStatus): void {
    const entry = this._entries.find(e => e.id === id);
    if (entry) { entry.status = status; this.onChange?.(); }
  }

  /** Mark all entries from startIdx onward as the given status. */
  setStatusFrom(startIdx: number, status: EntryStatus): void {
    for (let i = startIdx; i < this._entries.length; i++) {
      this._entries[i].status = status;
    }
    this.onChange?.();
  }

  /** Reset all entry statuses to pending. */
  resetAllStatus(): void {
    for (const e of this._entries) e.status = 'pending';
    this.onChange?.();
  }

  clear(): void {
    this._entries = [];
    this.onChange?.();
  }

  /** Find entry index by ID. Returns -1 if not found. */
  indexOf(id: string): number {
    return this._entries.findIndex(e => e.id === id);
  }

  /**
   * Split entries into chain groups. Consecutive move entries form a single
   * chain group; each dwell entry is its own group. Entries with status
   * 'done' or 'error' are still included (the runner skips completed groups).
   */
  chainGroups(): ChainGroup[] {
    const groups: ChainGroup[] = [];
    let currentMoves: MoveEntry[] = [];

    for (const entry of this._entries) {
      if (entry.type === 'move') {
        currentMoves.push(entry);
      } else {
        if (currentMoves.length > 0) {
          groups.push({ type: 'moves', entries: [...currentMoves] });
          currentMoves = [];
        }
        groups.push({ type: 'dwell', entry });
      }
    }
    if (currentMoves.length > 0) {
      groups.push({ type: 'moves', entries: currentMoves });
    }
    return groups;
  }
}

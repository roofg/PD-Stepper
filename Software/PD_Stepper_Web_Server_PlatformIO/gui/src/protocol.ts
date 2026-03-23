/**
 * Binary protocol parser for PD-Stepper USB CDC telemetry.
 *
 * Wire format (little-endian):
 *   UPDATE  0xAA 0xBB  29 bytes total
 *   STOP    0xAA 0xCC  38 bytes total
 */

const SYNC     = 0xaa;
const UPDATE_T = 0xbb;
const STOP_T   = 0xcc;

const UPDATE_LEN = 29;
const STOP_LEN   = 38;

export interface TelemetryUpdate {
  type: 'update';
  timestamp: number;   // uint32, µs
  pos:       number;   // int32, pulse-counted steps
  meas:      number;   // int32, encoder-derived steps
  target:    number;   // int32, planner reference steps
  lag:       number;   // int16, target - meas
  vel:       number;   // int16, steps/s
  accel:     number;   // int16, steps/s²
  dist:      number;   // int16, remaining steps
  stallguard: number;  // uint16, TMC2209 StallGuard result
}

export interface StopPacket {
  type:   'stop';
  pos:    number;  // int32, final position
  reason: string;  // up to 32-char ASCII
}

export type Packet = TelemetryUpdate | StopPacket;

export class PacketParser {
  private buf: number[] = [];
  onPacket: ((pkt: Packet) => void) | null = null;

  /** Clear buffered bytes — call on each new connection to avoid stale-byte corruption. */
  reset(): void {
    this.buf = [];
  }

  feed(chunk: Uint8Array): void {
    for (let i = 0; i < chunk.length; i++) this.buf.push(chunk[i]);
    this._parse();
  }

  private _parse(): void {
    while (this.buf.length >= 2) {
      if (this.buf[0] !== SYNC) {
        this.buf.shift();
        continue;
      }

      const type = this.buf[1];

      if (type === UPDATE_T) {
        if (this.buf.length < UPDATE_LEN) return;

        // Validate XOR checksum over bytes 2..27 BEFORE consuming.
        // Splicing first would irrecoverably lose any valid 0xAA byte
        // embedded in the discarded window (common in int32 position fields).
        let chk = 0;
        for (let i = 2; i < 28; i++) chk ^= this.buf[i];
        if (chk !== this.buf[28]) {
          this.buf.shift(); // discard false 0xAA sync, rescan from next byte
          continue;
        }

        const pkt = this.buf.splice(0, UPDATE_LEN); // checksum OK — consume

        const dv = new DataView(new Uint8Array(pkt).buffer);
        this.onPacket?.({
          type:       'update',
          timestamp:  dv.getUint32(2, true),
          pos:        dv.getInt32(6, true),
          meas:       dv.getInt32(10, true),
          target:     dv.getInt32(14, true),
          lag:        dv.getInt16(18, true),
          vel:        dv.getInt16(20, true),
          accel:      dv.getInt16(22, true),
          dist:       dv.getInt16(24, true),
          stallguard: dv.getUint16(26, true),
        });

      } else if (type === STOP_T) {
        if (this.buf.length < STOP_LEN) return;
        const pkt = this.buf.splice(0, STOP_LEN);
        const dv  = new DataView(new Uint8Array(pkt).buffer);
        const pos = dv.getInt32(2, true);
        const raw = new Uint8Array(pkt.slice(6, 38));
        const end = raw.indexOf(0);
        const reason = new TextDecoder().decode(raw.slice(0, end === -1 ? 32 : end));
        this.onPacket?.({ type: 'stop', pos, reason });

      } else {
        this.buf.shift(); // unknown second byte — skip sync and rescan
      }
    }
  }
}

export function moveCommand(
  distance: number,
  speed:    number,
  accel:    number,
  abs = false,
): string {
  return JSON.stringify({ cmd: 'move', distance, speed, accel, abs }) + '\n';
}

export function setCurrentCommand(percent: number): string {
  return JSON.stringify({ cmd: 'set_current', value: percent }) + '\n';
}

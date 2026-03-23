/**
 * Web Serial API connection manager for PD-Stepper.
 *
 * Usage:
 *   const conn = new SerialConnection();
 *   conn.onPacket = (pkt) => { ... };
 *   conn.onConnectionChange = (connected) => { ... };
 *   await conn.connect();          // opens Chrome port picker
 *   await conn.write('{"cmd":...}\n');
 *   await conn.disconnect();
 */

import { PacketParser, type Packet } from './protocol';

/** Milliseconds to drain stale bytes after port open (mirrors TriggerMove.py). */
const DRAIN_MS = 150;

export class SerialConnection {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private readLoopDone: Promise<void> = Promise.resolve();
  private readonly parser = new PacketParser();
  private _isConnected = false;
  // Serialize writes to avoid "WritableStream is already locked" errors
  // when UI events fire in rapid succession.
  private _writeChain: Promise<void> = Promise.resolve();

  onPacket: ((pkt: Packet) => void) | null = null;
  onConnectionChange: ((connected: boolean) => void) | null = null;

  get isConnected(): boolean { return this._isConnected; }

  constructor() {
    this.parser.onPacket = (pkt) => this.onPacket?.(pkt);
  }

  async connect(): Promise<void> {
    if (this._isConnected) return; // guard against double-connect
    if (!('serial' in navigator)) {
      throw new Error(
        'Web Serial API not supported. Use Chrome or Edge (or enable the flag in chrome://flags).',
      );
    }
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 921600 });
    this.parser.reset(); // clear any stale bytes from a prior session
    this.port = port;
    this._isConnected = true;
    this.onConnectionChange?.(true);
    this.readLoopDone = this._readLoop(port);
  }

  async disconnect(): Promise<void> {
    if (!this._isConnected) return;
    // Cancel the reader — causes reader.read() to resolve with done=true.
    if (this.reader) await this.reader.cancel().catch(() => undefined);
    // Wait for the read loop's finally block to release the lock.
    await this.readLoopDone;
    if (this.port) {
      await this.port.close().catch(() => undefined);
      this.port = null;
    }
  }

  /**
   * Send a UTF-8 string over the serial port.
   * Calls are automatically serialized — safe to call without awaiting.
   */
  write(data: string): Promise<void> {
    const p = this._writeChain.then(() => this._rawWrite(data));
    // Keep chain alive even if this write fails, so subsequent writes proceed.
    this._writeChain = p.catch(() => undefined);
    return p;
  }

  private async _rawWrite(data: string): Promise<void> {
    if (!this.port?.writable) throw new Error('Not connected');
    const writer = this.port.writable.getWriter();
    try {
      await writer.write(new TextEncoder().encode(data));
    } finally {
      writer.releaseLock();
    }
  }

  private async _readLoop(port: SerialPort): Promise<void> {
    if (!port.readable) return;
    const connectedAt = Date.now();
    const reader = port.readable.getReader();
    this.reader = reader;
    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;
        // Discard stale bytes buffered in the OS CDC TX FIFO before our open.
        if (Date.now() - connectedAt < DRAIN_MS) continue;
        this.parser.feed(value);
      }
    } catch {
      // Port disconnected unexpectedly — fall through to finally.
    } finally {
      reader.releaseLock();
      this.reader = null;
      if (this._isConnected) {
        this._isConnected = false;
        this.onConnectionChange?.(false);
      }
    }
  }
}

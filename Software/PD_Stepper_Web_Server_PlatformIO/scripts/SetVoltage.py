"""
SetVoltage.py — Set USB-PD supply voltage and/or motor run-current on PD-Stepper.

The CH224K USB-PD chip supports fixed voltage profiles: 5, 9, 12, 15, or 20 V.
Motor current is a percentage (0–100%) of the TMC2209 rated RMS current.

Usage:
    python SetVoltage.py --voltage 20
    python SetVoltage.py --current 60
    python SetVoltage.py --voltage 20 --current 60
    python SetVoltage.py --voltage 20 --current 60 --save
    python SetVoltage.py --get

Options:
    --port     Serial port (default: COM3)
    --voltage  USB-PD voltage to request: 5, 9, 12, 15, or 20  [V]
    --current  Run-current percentage: 0–100  [%]
    --save     Persist settings to flash (survives reboot)
    --get      Read and print current settings, then exit
"""

import argparse
import json
import time
import serial

VALID_VOLTAGES = {5, 9, 12, 15, 20}


def send_cmd(ser: serial.Serial, obj: dict) -> None:
    line = json.dumps(obj) + "\n"
    ser.write(line.encode("utf-8"))


def drain_text(ser: serial.Serial, wait: float = 0.3) -> str:
    """Read all pending bytes and return printable ASCII lines."""
    time.sleep(wait)
    raw = ser.read(ser.in_waiting or 0)
    return raw.decode("ascii", errors="replace")


def main():
    parser = argparse.ArgumentParser(
        description="Configure PD-Stepper supply voltage and motor current"
    )
    parser.add_argument("--port",    default="COM3",  help="Serial port (default: COM3)")
    parser.add_argument("--voltage", type=int,        help=f"USB-PD voltage {sorted(VALID_VOLTAGES)} [V]")
    parser.add_argument("--current", type=int,        help="Run-current 0–100 [%%]")
    parser.add_argument("--save",    action="store_true", help="Save settings to flash")
    parser.add_argument("--get",     action="store_true", help="Print current settings and exit")
    args = parser.parse_args()

    if not args.get and args.voltage is None and args.current is None:
        parser.error("Specify at least one of --voltage, --current, or --get")

    if args.voltage is not None and args.voltage not in VALID_VOLTAGES:
        parser.error(f"--voltage must be one of {sorted(VALID_VOLTAGES)}")

    if args.current is not None and not (0 <= args.current <= 100):
        parser.error("--current must be 0–100")

    print(f"Connecting to {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.5, dsrdtr=True)
    except Exception as e:
        print(f"ERROR: {e}")
        return

    time.sleep(0.1)
    ser.reset_input_buffer()

    if args.get:
        send_cmd(ser, {"cmd": "get_settings"})
        output = drain_text(ser, wait=0.5)
        for line in output.splitlines():
            line = line.strip()
            if line:
                print(line)
        ser.close()
        return

    if args.voltage is not None:
        cmd = {"cmd": "set_voltage", "value": args.voltage}
        print(f"Setting voltage → {args.voltage} V")
        send_cmd(ser, cmd)
        output = drain_text(ser)
        for line in output.splitlines():
            line = line.strip()
            if line:
                print(f"  {line}")

    if args.current is not None:
        cmd = {"cmd": "set_current", "value": args.current}
        print(f"Setting run-current → {args.current}%")
        send_cmd(ser, cmd)
        output = drain_text(ser)
        for line in output.splitlines():
            line = line.strip()
            if line:
                print(f"  {line}")

    if args.save:
        print("Saving to flash...")
        send_cmd(ser, {"cmd": "save"})
        output = drain_text(ser)
        for line in output.splitlines():
            line = line.strip()
            if line:
                print(f"  {line}")

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()

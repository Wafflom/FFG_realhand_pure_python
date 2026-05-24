#!/usr/bin/env python3
"""Dump bytes from a USB serial device as hex and best-effort text."""

from __future__ import annotations

import argparse
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from exc


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", default="/dev/ttyUSB2")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--chunk", type=int, default=128)
    args = parser.parse_args()

    if args.scan:
        for port in serial.tools.list_ports.comports():
            print(port.device, port.description, port.hwid)
        return

    deadline = time.monotonic() + args.seconds
    with serial.Serial(args.port, args.baudrate, timeout=0.1) as ser:
        print(f"opened {args.port} @ {args.baudrate}")
        while time.monotonic() < deadline:
            data = ser.read(ser.in_waiting or args.chunk)
            if not data:
                continue
            hex_text = data.hex(" ")
            ascii_text = "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in data)
            print(f"{len(data):04d}  {hex_text}")
            print(f"      {ascii_text}")


if __name__ == "__main__":
    main()

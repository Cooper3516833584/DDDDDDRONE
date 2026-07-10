"""Probe the flight-controller wireless forwarding channel.

This opens the FC serial port and uses the existing 0x0D/0x07 wireless bridge.
It does not arm, take off, change flight modes, or send motion commands.
"""

import argparse
import os
import sys
import threading
import time
from datetime import datetime
from typing import Optional

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Controller  # noqa: E402


def format_bytes(data: bytes, hex_only: bool, encoding: str) -> str:
    hex_text = " ".join(f"{byte:02X}" for byte in data)
    if hex_only:
        return hex_text
    text = data.decode(encoding, errors="replace")
    text = text.replace("\r", "\\r").replace("\n", "\\n")
    return f"{text}    [hex: {hex_text}]"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FC wireless bridge probe")
    parser.add_argument("--fc-port", help="FC serial port, for example COM5 or /dev/ttyACM0")
    parser.add_argument("--fc-baud", type=int, default=500000, help="FC serial baud rate")
    parser.add_argument("--encoding", default="utf-8", help="text encoding for send/display")
    parser.add_argument("--hex", action="store_true", help="display received data as hex only")
    parser.add_argument("--period", type=float, default=1.0, help="seconds between probe messages")
    parser.add_argument("--message", default="AIR_HC14_PING", help="probe message prefix")
    parser.add_argument("--count", type=int, default=0, help="number of probe messages, 0 means forever")
    parser.add_argument(
        "--allow-auto-port",
        action="store_true",
        help="allow SDK VID/PID auto-detection when --fc-port is omitted",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.fc_port and not args.allow_auto_port:
        print("Pass --fc-port explicitly, or add --allow-auto-port to use SDK auto-detection.")
        return 2

    fc = FC_Controller()
    fc.settings.ack_max_retry = 1
    fc.settings.raise_if_no_ack = False
    fc.settings.raise_if_timeout = False

    stop_event = threading.Event()

    def on_wireless(data: bytes) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n[RX wireless {ts} {len(data)}B] {format_bytes(data, args.hex, args.encoding)}")

    fc.register_wireless_callback(on_wireless)

    try:
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
            block_until_connected=False,
        )
        print("FC serial listener started.")
        print("Sending wireless probe frames only; no flight-control commands are sent.")

        seq = 0
        while not stop_event.is_set():
            if args.count and seq >= args.count:
                break
            seq += 1
            ts = datetime.now().strftime("%H:%M:%S")
            payload_text = f"{args.message},{seq},{ts}\n"
            payload = payload_text.encode(args.encoding, errors="replace")
            fc.send_to_wireless(payload)
            print(f"[TX wireless #{seq}] {payload_text.strip()}")
            if stop_event.wait(args.period):
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as exc:
        print(f"Probe failed: {exc}")
        return 1
    finally:
        stop_event.set()
        try:
            fc.close(joined=True)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Continuously display read-only flight-controller telemetry.

This program starts only the SDK serial listener.  It never calls unlock,
take_off, set_flight_mode, or any motion-control API.  The SDK listener sends
its normal communication heartbeat so that the flight controller continues to
publish state frames; that heartbeat is not a flight command.

Example:
    python testcode/fc_state_monitor.py --fc-port COM5
    python testcode/fc_state_monitor.py --allow-auto-port
"""

import argparse
import os
import sys
import time
from datetime import datetime

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Controller  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously display read-only flight-controller telemetry."
    )
    parser.add_argument(
        "--fc-port",
        help="flight-controller serial port, for example COM5 or /dev/ttyACM0",
    )
    parser.add_argument("--fc-baud", type=int, default=500000, help="serial baud rate")
    parser.add_argument(
        "--allow-auto-port",
        action="store_true",
        help="allow SDK VID/PID auto-detection when --fc-port is omitted",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="maximum print rate in Hz (default: 5; use 0 for every received frame)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.fc_port and not args.allow_auto_port:
        print("Pass --fc-port explicitly, or add --allow-auto-port to use SDK auto-detection.")
        return 2
    if args.rate < 0:
        print("--rate must be zero or greater.")
        return 2

    fc = FC_Controller()
    minimum_interval = 1.0 / args.rate if args.rate else 0.0
    last_print_time = 0.0

    def print_state(state) -> None:
        """Called by the serial listener when a state frame arrives."""
        nonlocal last_print_time
        now = time.monotonic()
        if now - last_print_time < minimum_interval:
            return
        last_print_time = now

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(
            f"{timestamp} | battery={state.bat.value:5.2f} V"
            f" | yaw={state.yaw.value:7.2f} deg"
            f" | pitch={state.pit.value:7.2f} deg"
            f" | roll={state.rol.value:7.2f} deg"
            f" | altitude={state.alt_add.value:5d} cm"
            f" | armed={'YES' if state.unlock.value else 'NO'}"
            f" | mode={state.mode.value}"
        )

    try:
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
            callback=print_state,
        )
        print("Telemetry monitor started. No arm, takeoff, mode, or motion command is sent.")
        print("Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nTelemetry monitor stopped.")
    except Exception as exc:
        print(f"Unable to start telemetry monitor: {exc}")
        return 1
    finally:
        try:
            fc.close(joined=True)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

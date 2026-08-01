"""Continuously display read-only flight-controller telemetry.

This program never calls unlock, take_off, set_flight_mode, or any motion-control
API.  The underlying SDK listener sends its normal communication heartbeat so that
the flight controller continues to publish state frames; that heartbeat is not a
flight command.

Two connection modes are supported:

1. Client mode (default) -- connect to an already-running FC_Server
   (e.g. server_ros.py) on the onboard computer via TCP.
   Example:
       python python_sdk/fc_state_monitor.py
       python python_sdk/fc_state_monitor.py --host 192.168.31.176

2. Direct serial mode -- open the flight-controller serial port directly.
   Use this only when server_ros.py is NOT running.
   Example:
       python python_sdk/fc_state_monitor.py --direct --fc-port COM5
       python python_sdk/fc_state_monitor.py --direct --allow-auto-port
"""

import argparse
import os
import sys
import time
from datetime import datetime

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Client, FC_Controller  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously display read-only flight-controller telemetry."
    )

    # ---- 连接模式 ----
    parser.add_argument(
        "--direct",
        action="store_true",
        help="use direct serial mode instead of the default FC_Client (TCP) mode",
    )

    # ---- FC_Client 模式参数 (默认) ----
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="FC_Server address (default: 127.0.0.1; only used in client mode)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5654,
        help="FC_Server port (default: 5654; only used in client mode)",
    )
    parser.add_argument(
        "--authkey",
        default="fc",
        help="FC_Server auth key (default: fc; only used in client mode)",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="connection timeout in seconds (default: 5; only used in client mode)",
    )

    # ---- FC_Controller 直连串口模式参数 ----
    parser.add_argument(
        "--fc-port",
        help="flight-controller serial port, e.g. COM5 or /dev/ttyACM0 (only used in direct mode)",
    )
    parser.add_argument(
        "--fc-baud", type=int, default=500000, help="serial baud rate (only used in direct mode)"
    )
    parser.add_argument(
        "--allow-auto-port",
        action="store_true",
        help="allow SDK VID/PID auto-detection when --fc-port is omitted (only used in direct mode)",
    )

    # ---- 通用参数 ----
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="maximum print rate in Hz (default: 5; use 0 for every received frame)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate < 0:
        print("--rate must be zero or greater.")
        return 2

    minimum_interval = 1.0 / args.rate if args.rate else 0.0
    last_print_time = 0.0

    def print_state(state) -> None:
        """Called when a state frame arrives."""
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

    if args.direct:
        # ---- 直连串口模式 ----
        if not args.fc_port and not args.allow_auto_port:
            print("Direct mode: pass --fc-port explicitly, or add --allow-auto-port for auto-detection.")
            return 2

        fc = FC_Controller()
        try:
            fc.start_listen_serial(
                serial_dev=args.fc_port,
                baudrate=args.fc_baud,
                print_state=False,
                callback=print_state,
            )
            print("Telemetry monitor started (direct serial mode).")
            print("No arm, takeoff, mode, or motion command is sent.")
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
    else:
        # ---- 默认 FC_Client 模式 ----
        fc = FC_Client()
        addr = (args.host, args.port)
        print(f"Connecting to FC_Server at {addr[0]}:{addr[1]} ...")
        try:
            fc.connect(
                host=args.host,
                port=args.port,
                authkey=args.authkey.encode() if isinstance(args.authkey, str) else args.authkey,
                print_state=False,
                callback=print_state,
                block=True,
                timeout=args.connect_timeout,
            )
        except Exception as exc:
            print(f"Unable to connect to FC_Server ({addr[0]}:{addr[1]}): {exc}")
            print("Please ensure server_ros.py (or equivalent FC_Server) is running on the target.")
            print("Tip: use --direct to connect to the flight controller serial port directly.")
            return 1

        print(f"Telemetry monitor started (client mode, connected to {addr[0]}:{addr[1]}).")
        print("No arm, takeoff, mode, or motion command is sent.")
        print("Press Ctrl+C to stop.")
        try:
            while fc.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nTelemetry monitor stopped.")
        except Exception as exc:
            print(f"Telemetry monitor error: {exc}")
            return 1
        finally:
            fc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

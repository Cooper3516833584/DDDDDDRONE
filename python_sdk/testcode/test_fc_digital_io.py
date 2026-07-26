"""Safely test one flight-controller digital-output channel.

This script supports two connection modes:

- **client (default)**:  Connect via the network to an already-running
  ``FC_Server`` (e.g. the ``server_ros.py`` auto-start service on the onboard
 上位机).  Uses ``FC_Client`` – no direct serial port access, so it will not
  contend with ``server_ros.py`` for the flight controller.

- **serial**:  Open the flight-controller serial port directly with
  ``FC_Controller``.  Use this only when ``server_ros.py`` is NOT running on
  the上位机.

It never calls an arm/unlock, flight-mode, PWM, take-off, or motion-control
API.  For an ``on`` test it always switches the selected channel back off
before exiting.

Examples:
    # Default: client mode (connect to server_ros.py on localhost)
    python testcode/test_fc_digital_io.py --channel 1 --state on --duration 2

    # Client mode, connect to remote onboard 上位机
    python testcode/test_fc_digital_io.py --host 192.168.31.176 --channel 1 --state on

    # Serial mode (direct FC connection; server_ros.py must NOT be running)
    python testcode/test_fc_digital_io.py --mode serial --fc-port COM5 --channel 1 --state off
"""

import argparse
import os
import sys
import time


SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Client, FC_Controller  # noqa: E402
from FlightController.Application import FC_Application  # noqa: E402


# ---------------------------------------------------------------------------
# Shared argument helpers
# ---------------------------------------------------------------------------

def _positive_float(value: str) -> float:
    """Argparse type: require a strictly positive float."""
    f = float(value)
    if f <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return f


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Test exactly one FC digital IO channel; no flight commands are sent. "
            "Default connection mode is 'client' (network → FC_Server)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # ---- Connection mode ----
    parser.add_argument(
        "--mode",
        choices=("client", "serial"),
        default="client",
        help=(
            "Connection mode: 'client' uses FC_Client to talk to an already-running "
            "FC_Server (default, works with auto-started server_ros.py); "
            "'serial' uses FC_Controller to open the FC serial port directly."
        ),
    )

    # ---- Client-mode arguments ----
    client_group = parser.add_argument_group("client-mode options (default)")
    client_group.add_argument(
        "--host",
        default="127.0.0.1",
        help="FC_Server address (default: 127.0.0.1; use 192.168.31.176 for the onboard 上位机)",
    )
    client_group.add_argument(
        "--port",
        type=int,
        default=5654,
        help="FC_Server port (default: 5654, same as server_ros.py)",
    )
    client_group.add_argument(
        "--authkey",
        default="fc",
        help="FC_Server auth key (default: fc)",
    )

    # ---- Serial-mode arguments ----
    serial_group = parser.add_argument_group("serial-mode options")
    serial_group.add_argument(
        "--fc-port",
        default=None,
        help="flight-controller serial port, e.g. COM5 or /dev/ttyACM0 (required in serial mode)",
    )
    serial_group.add_argument(
        "--fc-baud", type=int, default=500000, help="serial baud rate (serial mode only)"
    )

    # ---- IO test arguments ----
    io_group = parser.add_argument_group("IO test options")
    io_group.add_argument(
        "--channel", type=int, choices=range(4), required=True, help="IO channel: 0-3"
    )
    io_group.add_argument(
        "--state", choices=("on", "off"), required=True, help="output state to test"
    )
    io_group.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="seconds to keep the output on (only used with --state on; default: 1)",
    )

    # ---- Common ----
    parser.add_argument(
        "--connect-timeout",
        type=_positive_float,
        default=5.0,
        help="seconds to wait for a state frame from the FC (default: 5)",
    )

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Core logic (mode-agnostic after connection is up)
# ---------------------------------------------------------------------------

def _run_io_test(
    fc: FC_Application,  # FC_Controller and FC_Client share the relevant API surface
    channel: int,
    state: str,
    duration: float,
) -> int:
    """Execute the IO test *once*.  ``fc`` must already be connected."""

    output_may_be_on = False

    try:
        if fc.state.unlock.value:
            print("Flight controller reports ARMED/UNLOCKED; refusing to send an IO command.")
            return 3

        print(
            f"FC is locked. Testing digital IO channel {channel}: {state}. "
            "No arm, PWM, mode, take-off, or motion command is sent."
        )

        if state == "off":
            fc.set_digital_output(channel, False)
            print(f"Channel {channel} is now OFF.")
            return 0

        # ON test with automatic OFF
        output_may_be_on = True
        fc.set_digital_output(channel, True)
        print(f"Channel {channel} is ON for {duration:g} second(s).")
        time.sleep(duration)
        fc.set_digital_output(channel, False)
        output_may_be_on = False
        print(f"Channel {channel} is now OFF.")
        return 0

    except Exception:
        raise
    finally:
        if output_may_be_on:
            try:
                fc.set_digital_output(channel, False)
                print(f"Fail-safe: channel {channel} turned OFF.")
            except Exception as exc:
                print(f"WARNING: could not send the fail-safe OFF command: {exc}")


# ---------------------------------------------------------------------------
# Mode dispatchers
# ---------------------------------------------------------------------------

def _run_client_mode(args: argparse.Namespace) -> int:
    """Connect via FC_Client to an already-running FC_Server."""
    fc = FC_Client()
    try:
        print(
            f"[*] Client mode: connecting to FC_Server at {args.host}:{args.port} "
            f"(authkey={args.authkey}) ..."
        )
        fc.connect(
            host=args.host,
            port=args.port,
            authkey=args.authkey.encode() if isinstance(args.authkey, str) else args.authkey,
            print_state=False,
            block=True,
            timeout=args.connect_timeout,
        )
        print("[✓] Connected to FC_Server; waiting for first state frame ...")

        if not fc.wait_for_connection(timeout_s=args.connect_timeout):
            print("No flight-controller state frame received; no IO command was sent.")
            return 1

        return _run_io_test(fc, args.channel, args.state, args.duration)

    except KeyboardInterrupt:
        print("\nInterrupted; turning the selected IO channel off.")
        return 130
    except Exception as exc:
        print(f"IO test failed: {exc}")
        return 1
    finally:
        try:
            fc.close(joined=True)
        except Exception:
            pass


def _run_serial_mode(args: argparse.Namespace) -> int:
    """Open the FC serial port directly via FC_Controller."""
    fc = FC_Controller()
    try:
        print(f"[*] Serial mode: opening {args.fc_port} @ {args.fc_baud} baud ...")
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
        )

        if not fc.wait_for_connection(timeout_s=args.connect_timeout):
            print("No flight-controller state frame received; no IO command was sent.")
            return 1

        return _run_io_test(fc, args.channel, args.state, args.duration)

    except KeyboardInterrupt:
        print("\nInterrupted; turning the selected IO channel off.")
        return 130
    except Exception as exc:
        print(f"IO test failed: {exc}")
        return 1
    finally:
        try:
            fc.close(joined=True)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if args.duration < 0:
        print("--duration must be zero or greater.")
        return 2

    # Validate serial-mode requirement
    if args.mode == "serial" and args.fc_port is None:
        print("--fc-port is required when using --mode serial")
        return 2

    if args.mode == "client":
        return _run_client_mode(args)
    else:
        return _run_serial_mode(args)


if __name__ == "__main__":
    sys.exit(main())

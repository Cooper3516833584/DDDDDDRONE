"""Safely test one flight-controller digital-output channel.

This script only opens the flight-controller serial connection and calls
``set_digital_output``.  It never calls an arm/unlock, flight-mode, PWM,
take-off, or motion-control API.  For an ``on`` test it always switches the
selected channel back off before exiting.

Examples:
    # Test channel 1 high for two seconds, then turn it off automatically.
    python testcode/test_fc_digital_io.py --fc-port /dev/ttyACM0 --channel 1 --state on --duration 2

    # Explicitly force channel 1 low.
    python testcode/test_fc_digital_io.py --fc-port COM5 --channel 1 --state off
"""

import argparse
import os
import sys
import time


SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Controller  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test exactly one FC digital IO channel; no flight commands are sent."
    )
    parser.add_argument(
        "--fc-port",
        required=True,
        help="flight-controller serial port, e.g. COM5 or /dev/ttyACM0",
    )
    parser.add_argument("--fc-baud", type=int, default=500000, help="serial baud rate")
    parser.add_argument("--channel", type=int, choices=range(4), required=True, help="IO channel: 0-3")
    parser.add_argument("--state", choices=("on", "off"), required=True, help="output state to test")
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="seconds to keep the output on (only used with --state on; default: 1)",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for a state frame from the FC (default: 5)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration < 0:
        print("--duration must be zero or greater.")
        return 2
    if args.connect_timeout <= 0:
        print("--connect-timeout must be greater than zero.")
        return 2

    fc = FC_Controller()
    output_may_be_on = False
    listener_started = False
    try:
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
        )
        listener_started = True
        if not fc.wait_for_connection(timeout_s=args.connect_timeout):
            print("No flight-controller state frame received; no IO command was sent.")
            return 1

        # Do not alter the aircraft state.  Refuse the IO test if it is armed.
        if fc.state.unlock.value:
            print("Flight controller reports ARMED/UNLOCKED; refusing to send an IO command.")
            return 3

        print(
            f"FC is locked. Testing digital IO channel {args.channel}: {args.state}. "
            "No arm, PWM, mode, take-off, or motion command is sent."
        )
        if args.state == "off":
            fc.set_digital_output(args.channel, False)
            print(f"Channel {args.channel} is now OFF.")
            return 0

        # Set the flag first so that an exception during acknowledgement still
        # causes the fail-safe OFF command in finally.
        output_may_be_on = True
        fc.set_digital_output(args.channel, True)
        print(f"Channel {args.channel} is ON for {args.duration:g} second(s).")
        time.sleep(args.duration)
        fc.set_digital_output(args.channel, False)
        output_may_be_on = False
        print(f"Channel {args.channel} is now OFF.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; turning the selected IO channel off.")
        return 130
    except Exception as exc:
        print(f"IO test failed: {exc}")
        return 1
    finally:
        if listener_started and output_may_be_on:
            try:
                fc.set_digital_output(args.channel, False)
                print(f"Fail-safe: channel {args.channel} turned OFF.")
            except Exception as exc:
                print(f"WARNING: could not send the fail-safe OFF command: {exc}")
        if listener_started:
            try:
                fc.close(joined=True)
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())

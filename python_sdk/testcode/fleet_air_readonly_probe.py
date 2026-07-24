"""Explicit read-only FleetBus probe for an already running FC_Server."""

import argparse
import signal
import threading


class UnavailableNavigation:
    def pose_is_fresh(self) -> bool:
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5654)
    parser.add_argument(
        "--connect-readonly",
        action="store_true",
        help="connect and serve only FleetBus state/PING requests",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.connect_readonly:
        print("No connection made. Add --connect-readonly after confirming FC_Server.")
        return 2

    from FlightController import FC_Client
    from fleet_bus.air_node import attach_air_fleet_node

    fc = FC_Client()
    fc.connect(args.host, args.port)
    stop_event = threading.Event()
    node = attach_air_fleet_node(
        fc, UnavailableNavigation(), stop_event, readonly=True
    )
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    try:
        stop_event.wait()
    finally:
        node.close()
        fc.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

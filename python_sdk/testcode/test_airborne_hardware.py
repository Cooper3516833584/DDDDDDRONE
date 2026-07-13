#!/usr/bin/env python3
"""One-click read-only hardware and index check for the airborne computer.

This wrapper connects through SSH and reuses ``ssh_dual_camera_viewer`` in
probe-only mode.  It enumerates USB/serial/video devices, checks the known
flight-controller, HC-14 and radar USB identifiers, reports stable by-id paths
and device occupancy, and reads three frames from the front and downward
cameras.  It never opens a serial port or sends a serial/flight command.
"""

import argparse

from ssh_dual_camera_viewer import main as hardware_probe_main


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="One-click read-only airborne hardware and index check."
    )
    parser.add_argument("--host", default="192.168.31.176")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--user", default="fc")
    parser.add_argument(
        "--cameras",
        type=int,
        nargs=2,
        metavar=("FRONT", "DOWNWARD"),
        default=(2, 0),
        help="camera indexes (default: front 2, downward 0)",
    )
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--strict-host-key-checking", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    viewer_args = [
        "--probe-only",
        "--host", args.host,
        "--port", str(args.port),
        "--user", args.user,
        "--cameras", str(args.cameras[0]), str(args.cameras[1]),
        "--connect-timeout", str(args.connect_timeout),
        "--startup-timeout", str(args.startup_timeout),
    ]
    if args.strict_host_key_checking:
        viewer_args.append("--strict-host-key-checking")
    return hardware_probe_main(viewer_args)


if __name__ == "__main__":
    raise SystemExit(main())

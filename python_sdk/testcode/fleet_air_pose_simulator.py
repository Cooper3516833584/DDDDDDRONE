"""HC-14-only drone pose simulator with no flight-controller access.

The simulator replies to ground-station FleetBus polls and trace requests with
a moving synthetic pose.  It never imports or opens the flight controller and
runs for a bounded duration unless stopped earlier by SIGINT, SIGTERM, or a
FleetBus TARGETED_STOP command.
"""

import argparse
import math
from pathlib import Path
import signal
import sys
import threading
import time


SDK_DIR = Path(__file__).resolve().parents[1]
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connect-hc14",
        action="store_true",
        help="open only the airborne CH340/HC-14 serial link",
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--connect-timeout-s", type=float, default=5.0)
    return parser


class SimulatedAirStateProvider:
    """Generate a bounded circular FIELD-frame pose without touching hardware."""

    def __init__(self, state_type, node_flags):
        self._state_type = state_type
        self._node_flags = int(node_flags)
        self._started_at = time.monotonic()

    def __call__(self):
        elapsed_s = time.monotonic() - self._started_at
        angular_rate = 0.25
        angle = angular_rate * elapsed_s
        radius_cm = 70.0
        x_cm = 125.0 + radius_cm * math.cos(angle)
        y_cm = 150.0 + radius_cm * math.sin(angle)
        vx_cm_s = -radius_cm * angular_rate * math.sin(angle)
        vy_cm_s = radius_cm * angular_rate * math.cos(angle)
        return self._state_type(
            node_flags=self._node_flags,
            node_uptime_ms=round(elapsed_s * 1000.0) & 0xFFFFFFFF,
            x_cm=round(x_cm),
            y_cm=round(y_cm),
            z_cm=100,
            heading_cdeg=round(math.degrees(angle + math.pi / 2.0) * 100.0)
            % 36000,
            vx_cm_s=round(vx_cm_s),
            vy_cm_s=round(vy_cm_s),
            vz_cm_s=0,
            battery_cV=1600,
            operation_state=0,
            pose_quality=100,
            error_code=0,
        )


def main():
    args = build_parser().parse_args()
    if not args.connect_hc14:
        print("No serial port opened. Add --connect-hc14 for the bounded test.")
        return 2
    if args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    if args.connect_timeout_s <= 0.0:
        raise SystemExit("--connect-timeout-s must be positive")

    from fleet_bus.air_node import AirFleetNode
    from fleet_bus.command_queue import AirCommandQueue
    from fleet_bus.hc14_transport import HC14FleetTransport, resolve_hc14_settings
    from fleet_bus.models import AirFleetState, NodeFlags
    from fleet_bus.trace_buffer import TraceSamplingOptions

    stop_event = threading.Event()
    connected_event = threading.Event()
    holder = {}
    port, baudrate = resolve_hc14_settings(args.port, args.baudrate)
    state_provider = SimulatedAirStateProvider(
        AirFleetState,
        NodeFlags.POSE_VALID | NodeFlags.READY | NodeFlags.COORDINATE_FRAME_SYNCED,
    )
    transport = HC14FleetTransport(
        port=port,
        baudrate=baudrate,
        on_bytes=lambda data: holder["node"].feed_bytes(data),
        on_connected=lambda: connected_event.set(),
        on_disconnected=lambda error: print(
            "Air HC-14 disconnected: {}".format(error)
        ) if error is not None else None,
    )
    node = AirFleetNode(
        transport=transport,
        state_provider=state_provider,
        command_queue=AirCommandQueue(),
        stop_event=stop_event,
        readonly=True,
        trace_options=TraceSamplingOptions(
            enabled=True,
            sample_interval_s=0.10,
            buffer_capacity=1800,
            min_distance_cm=0.5,
        ),
    )
    holder["node"] = node

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node.start()
    try:
        if not connected_event.wait(args.connect_timeout_s):
            raise RuntimeError("air HC-14 did not connect within the timeout")
        print(
            "Air pose simulator connected on {} at {} baud for {:.1f}s".format(
                port, baudrate, args.duration_s
            ),
            flush=True,
        )
        stop_event.wait(args.duration_s)
    finally:
        node.close()
        print(
            "Air pose simulator stopped; trace_samples={} write_failures={}".format(
                node.trace_buffer.recorded_samples,
                node.write_failures,
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

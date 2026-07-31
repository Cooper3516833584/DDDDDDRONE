"""Publish one fixed 3x5 disaster survey through an existing FC_Server.

This commissioning helper is read-only: it does not arm, fly, navigate, open a
camera, or drive an actuator.  It only answers FleetBus polls through the
flight controller's UT2/USART2 wireless bridge.
"""

import argparse
from pathlib import Path
import signal
import sys
import threading
from typing import Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bus.models import SurveyFlags, SurveyState, TerrainCode


GRID_SIZE_CM = 70
GRID_ROWS = 3
GRID_COLS = 5


class UnavailableNavigation:
    """Keep the basic pose report invalid; this helper publishes survey data only."""

    def pose_is_fresh(self) -> bool:
        return False


def coordinate_to_cell(x_cm: int, y_cm: int) -> Tuple[int, int]:
    if x_cm % GRID_SIZE_CM or y_cm % GRID_SIZE_CM:
        raise ValueError("coordinates must be 70 cm grid centres")
    col = x_cm // GRID_SIZE_CM
    row = y_cm // GRID_SIZE_CM
    if not 0 <= row < GRID_ROWS or not 0 <= col < GRID_COLS:
        raise ValueError("coordinates are outside the 5x3 field")
    return row, col


def build_survey_state(
    water_x_cm: int = 70,
    water_y_cm: int = 0,
    fire_x_cm: int = 140,
    fire_y_cm: int = 70,
) -> SurveyState:
    water = coordinate_to_cell(water_x_cm, water_y_cm)
    fire = coordinate_to_cell(fire_x_cm, fire_y_cm)
    if water == fire:
        raise ValueError("water and wildfire must be different cells")

    terrain = [int(TerrainCode.FIELD)] * (GRID_ROWS * GRID_COLS)
    terrain[water[0] * GRID_COLS + water[1]] = int(TerrainCode.RIVER)
    terrain[fire[0] * GRID_COLS + fire[1]] = int(TerrainCode.WILDFIRE)
    cell_positions_cm = tuple(
        (int((col + 0.5) * GRID_SIZE_CM), int((row + 0.5) * GRID_SIZE_CM))
        for row in range(GRID_ROWS)
        for col in range(GRID_COLS)
    )
    return SurveyState(
        survey_revision=1,
        survey_flags=int(SurveyFlags.COMPLETE | SurveyFlags.ABSOLUTE_POSITIONS),
        wildfire_event_id=1,
        wildfire_row=fire[0],
        wildfire_col=fire[1],
        terrain_codes=tuple(terrain),
        cell_positions_cm=cell_positions_cm,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5654)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--water-x", type=int, default=70)
    parser.add_argument("--water-y", type=int, default=0)
    parser.add_argument("--fire-x", type=int, default=140)
    parser.add_argument("--fire-y", type=int, default=70)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    survey = build_survey_state(
        args.water_x, args.water_y, args.fire_x, args.fire_y
    )

    # Defer hardware-facing imports so pure mapping tests stay hardware-free.
    from FlightController import FC_Client
    from fleet_bus.air_node import attach_air_fleet_node

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stop_event.set())

    fc = FC_Client()
    node = None
    survey_requests = 0
    survey_lock = threading.Lock()

    def provide_survey():
        nonlocal survey_requests
        with survey_lock:
            survey_requests += 1
            count = survey_requests
        if count == 1 or count % 10 == 0:
            print("FleetBus survey requests served: {}".format(count), flush=True)
        return survey

    try:
        fc.connect(args.host, args.port)
        node = attach_air_fleet_node(
            fc,
            UnavailableNavigation(),
            stop_event,
            readonly=True,
            survey_provider=provide_survey,
        )
        print(
            "FleetBus survey active: water=({}, {}) fire=({}, {}) duration={}s".format(
                args.water_x,
                args.water_y,
                args.fire_x,
                args.fire_y,
                args.duration,
            ),
            flush=True,
        )
        stop_event.wait(args.duration)
        return 0
    finally:
        if node is not None:
            node.close()
        fc.close()


if __name__ == "__main__":
    raise SystemExit(main())

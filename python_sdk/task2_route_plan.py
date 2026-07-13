from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


WAREHOUSE_WIDTH_CM = 500.0
WAREHOUSE_HEIGHT_CM = 400.0
START_WAREHOUSE = (75.0, 75.0)
LANDING_WAREHOUSE = (425.0, 325.0)
NORTH_CORRIDOR_WAREHOUSE_Y = 330.0

# The aircraft scans 75 cm away from the shelf plane, matching the radar
# distance loop in 2024_D_24.py.
FACE_WAREHOUSE_X = {
    "A": 75.0,
    "B": 225.0,
    "C": 275.0,
    "D": 425.0,
}

FACE_YAW_DEG = {
    "A": 270.0,  # west side, camera points east
    "B": 90.0,   # east side, camera points west
    "C": 270.0,
    "D": 90.0,
}


Point = tuple[float, float]


@dataclass(frozen=True)
class TargetRoute:
    location: str
    face: str
    index: int
    scan_height_cm: float
    face_yaw_deg: float
    target_warehouse: Point
    outbound_local: tuple[Point, ...]
    return_local: tuple[Point, ...]

    @property
    def location_ordinal(self) -> int:
        return "ABCD".index(self.face) * 6 + self.index


def parse_location(location: str) -> tuple[str, int, int, int]:
    code = location.strip().upper()
    if len(code) != 2 or code[0] not in "ABCD" or code[1] not in "123456":
        raise ValueError(f"invalid inventory location: {location!r}")
    index = int(code[1])
    column = (index - 1) % 3
    row = (index - 1) // 3
    return code[0], index, column, row


def warehouse_to_local(point: Point) -> Point:
    """Convert diagram coordinates to Navigation coordinates relative to takeoff.

    2024_D_24.py defines Navigation x+ as north and y+ as west. The PDF uses
    diagram x+ as east and y+ as north.
    """

    warehouse_x, warehouse_y = point
    start_x, start_y = START_WAREHOUSE
    return warehouse_y - start_y, start_x - warehouse_x


def route_for_location(location: str) -> TargetRoute:
    face, index, column, row = parse_location(location)

    # From the viewer's left to right: A/C run north-to-south in the plan;
    # B/D are viewed from the other side and therefore run south-to-north.
    if face in ("A", "C"):
        target_warehouse_y = (250.0, 200.0, 150.0)[column]
    else:
        target_warehouse_y = (150.0, 200.0, 250.0)[column]

    face_warehouse_x = FACE_WAREHOUSE_X[face]
    target_warehouse = (face_warehouse_x, target_warehouse_y)
    north_start = warehouse_to_local(
        (START_WAREHOUSE[0], NORTH_CORRIDOR_WAREHOUSE_Y)
    )
    face_entry = warehouse_to_local(
        (face_warehouse_x, NORTH_CORRIDOR_WAREHOUSE_Y)
    )
    target_local = warehouse_to_local(target_warehouse)
    landing_local = warehouse_to_local(LANDING_WAREHOUSE)

    return TargetRoute(
        location=f"{face}{index}",
        face=face,
        index=index,
        scan_height_cm=140.0 if row == 0 else 100.0,
        face_yaw_deg=FACE_YAW_DEG[face],
        target_warehouse=target_warehouse,
        outbound_local=((0.0, 0.0), north_start, face_entry, target_local),
        return_local=(target_local, face_entry, landing_local),
    )


def load_inventory_map(path: str | Path) -> dict[int, str]:
    source = Path(path)
    with source.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    result: dict[int, str] = {}
    if isinstance(payload, dict):
        entries = [
            {"qr_number": key, "position": value}
            for key, value in payload.items()
        ]
    elif isinstance(payload, list):
        entries = payload
    else:
        raise ValueError("inventory record must be a list or object")

    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("inventory record entry must be an object")
        try:
            cargo_number = int(entry["qr_number"])
            location = str(entry["position"]).strip().upper()
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid inventory record entry") from exc
        parse_location(location)
        if not 1 <= cargo_number <= 24:
            raise ValueError(f"invalid cargo number: {cargo_number}")
        if cargo_number in result:
            raise ValueError(f"duplicate cargo number: {cargo_number}")
        if location in result.values():
            raise ValueError(f"duplicate inventory location: {location}")
        result[cargo_number] = location
    return result

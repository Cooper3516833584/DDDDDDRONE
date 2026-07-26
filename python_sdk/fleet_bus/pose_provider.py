"""Read-only adapters from existing navigation and FC state into FleetBus units."""

import math
import time
from typing import Any, Callable, Optional, Tuple

from .models import AirFleetState, NodeFlags


def _number(value: Any, default: float = 0.0) -> float:
    candidate = getattr(value, "value", value)
    try:
        number = float(candidate)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


class NavigationAirStateProvider:
    def __init__(
        self,
        fc: object,
        navigation: object,
        position_transform: Optional[
            Callable[[float, float], Optional[Tuple[float, float]]]
        ] = None,
        heading_offset_deg: float = 0.0,
    ) -> None:
        self._fc = fc
        self._navigation = navigation
        self._position_transform = position_transform
        self._heading_offset_deg = float(heading_offset_deg)
        self._started = time.monotonic()

    def __call__(self) -> AirFleetState:
        navigation = self._navigation
        navigation_pose_valid = bool(
            navigation is not None and navigation.pose_is_fresh()
        )
        pose_valid = navigation_pose_valid

        state = getattr(self._fc, "state", None)
        armed = bool(_number(getattr(state, "unlock", 0)))

        if navigation_pose_valid:
            # Navigation.current_* is already expressed in centimetres.
            raw_x_cm = _number(getattr(navigation, "current_x", 0.0))
            raw_y_cm = _number(getattr(navigation, "current_y", 0.0))
            z_cm = round(_number(getattr(navigation, "current_height", 0.0)))
            yaw_deg = _number(getattr(navigation, "current_yaw", 0.0))
            transformed = (
                (raw_x_cm, raw_y_cm)
                if self._position_transform is None
                else self._position_transform(raw_x_cm, raw_y_cm)
            )
            if transformed is None:
                pose_valid = False
                x_cm = y_cm = heading_cdeg = 0
            else:
                x_cm = round(transformed[0])
                y_cm = round(transformed[1])
                heading_cdeg = round(
                    ((-yaw_deg + self._heading_offset_deg) % 360.0) * 100
                ) % 36000
        else:
            x_cm = y_cm = z_cm = heading_cdeg = 0

        flags = int(NodeFlags.READY)
        if pose_valid:
            flags |= int(NodeFlags.POSE_VALID)
        if armed:
            flags |= int(NodeFlags.ARMED_OR_MOTOR_ACTIVE)

        battery_cV = round(_number(getattr(state, "bat", 0.0)) * 100)
        operation_state = round(_number(getattr(state, "mode", 0.0)))
        return AirFleetState(
            node_flags=flags,
            node_uptime_ms=round((time.monotonic() - self._started) * 1000),
            x_cm=x_cm,
            y_cm=y_cm,
            z_cm=z_cm,
            heading_cdeg=heading_cdeg,
            battery_cV=max(0, min(0xFFFF, battery_cV)),
            operation_state=max(0, min(0xFF, operation_state)),
            pose_quality=4 if pose_valid else 0,
        )

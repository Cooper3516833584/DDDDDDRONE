"""Read-only adapters from existing navigation and FC state into FleetBus units."""

import math
import time
from typing import Any

from .models import AirFleetState, NodeFlags


def _number(value: Any, default: float = 0.0) -> float:
    candidate = getattr(value, "value", value)
    try:
        number = float(candidate)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


class NavigationAirStateProvider:
    def __init__(self, fc: object, navigation: object) -> None:
        self._fc = fc
        self._navigation = navigation
        self._started = time.monotonic()

    def __call__(self) -> AirFleetState:
        navigation = self._navigation
        pose_valid = bool(navigation is not None and navigation.pose_is_fresh())
        flags = int(NodeFlags.READY)
        if pose_valid:
            flags |= int(NodeFlags.POSE_VALID)

        state = getattr(self._fc, "state", None)
        armed = bool(_number(getattr(state, "unlock", 0)))
        if armed:
            flags |= int(NodeFlags.ARMED_OR_MOTOR_ACTIVE)

        if pose_valid:
            # Navigation.current_* is already expressed in centimetres.
            x_cm = round(_number(getattr(navigation, "current_x", 0.0)))
            y_cm = round(_number(getattr(navigation, "current_y", 0.0)))
            z_cm = round(_number(getattr(navigation, "current_height", 0.0)))
            yaw_deg = _number(getattr(navigation, "current_yaw", 0.0))
            heading_cdeg = round((-yaw_deg % 360.0) * 100) % 36000
        else:
            x_cm = y_cm = z_cm = heading_cdeg = 0

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

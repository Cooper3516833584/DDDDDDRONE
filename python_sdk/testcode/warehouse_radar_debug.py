"""Manual debug entry for the warehouse radar localizer.

This script is intentionally guarded by ``--confirm-hardware`` because running
it can open a real radar serial/ROS/FC data source.  It never sends flight
control commands.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from warehouse_radar_localizer import (
    DebugOptions,
    LocalizationMode,
    LocalizationRequest,
    Pose2D,
    RadarAlgorithmConfig,
    SurfaceID,
    WarehouseRadarLocalizer,
)


SURFACE_ALIASES = {
    "west": SurfaceID.WEST_NET,
    "west_net": SurfaceID.WEST_NET,
    "east": SurfaceID.EAST_NET,
    "east_net": SurfaceID.EAST_NET,
    "south": SurfaceID.SOUTH_NET,
    "south_net": SurfaceID.SOUTH_NET,
    "north": SurfaceID.NORTH_NET,
    "north_net": SurfaceID.NORTH_NET,
    "ab": SurfaceID.SHELF_AB,
    "shelf_ab": SurfaceID.SHELF_AB,
    "cd": SurfaceID.SHELF_CD,
    "shelf_cd": SurfaceID.SHELF_CD,
}


def _parse_mode(value: str) -> LocalizationMode:
    modes = {
        "absolute": LocalizationMode.ABSOLUTE_ANCHOR,
        "anchor": LocalizationMode.ABSOLUTE_ANCHOR,
        "corridor": LocalizationMode.CORRIDOR_TRACK,
        "yaw": LocalizationMode.YAW_ONLY,
        "yaw_only": LocalizationMode.YAW_ONLY,
        "detection": LocalizationMode.DETECTION_ONLY,
        "detect": LocalizationMode.DETECTION_ONLY,
    }
    try:
        return modes[value.lower()]
    except KeyError as exc:
        raise argparse.ArgumentTypeError("unknown mode: {}".format(value)) from exc


def _parse_surfaces(values: list[str]) -> tuple[SurfaceID, ...]:
    surfaces = []
    for value in values:
        key = value.lower()
        if key not in SURFACE_ALIASES:
            raise argparse.ArgumentTypeError("unknown surface: {}".format(value))
        surfaces.append(SURFACE_ALIASES[key])
    return tuple(surfaces)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Warehouse radar localizer debug runner")
    parser.add_argument("--mode", type=_parse_mode, default=LocalizationMode.ABSOLUTE_ANCHOR)
    parser.add_argument("--surfaces", nargs="+", required=True)
    parser.add_argument("--prior-x", type=float, default=0.0)
    parser.add_argument("--prior-y", type=float, default=0.0)
    parser.add_argument("--prior-yaw", type=float, default=0.0)
    parser.add_argument("--show", action="store_true", help="show matplotlib debug window")
    parser.add_argument("--save-dir", default=None, help="directory for saved debug frames")
    parser.add_argument("--min-confidence", type=float, default=30.0)
    parser.add_argument("--distance-gate", type=float, default=45.0)
    parser.add_argument("--angle-gate", type=float, default=25.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--radar-com", default=None, help="serial port, FC object is not created here")
    parser.add_argument("--radar-type", default="LD06", choices=("LD06", "LD08"))
    parser.add_argument(
        "--confirm-hardware",
        action="store_true",
        help="required to open the real radar data source",
    )
    return parser


def _format_match(surface_id: SurfaceID, match) -> str:
    return (
        "{} valid={} candidates={} inliers={} residual={} support={} "
        "distance_error={} angle_error={} confidence={:.2f} reason={}"
    ).format(
        surface_id.name,
        match.valid,
        match.total_candidate_count,
        match.inlier_count,
        None if match.residual_rms_cm is None else round(match.residual_rms_cm, 2),
        None if match.support_length_cm is None else round(match.support_length_cm, 2),
        None if match.distance_error_cm is None else round(match.distance_error_cm, 2),
        None if match.angle_error_deg is None else round(match.angle_error_deg, 2),
        match.confidence,
        match.reason,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    surfaces = _parse_surfaces(args.surfaces)

    if not args.confirm_hardware:
        parser.error("refusing to open radar hardware without --confirm-hardware")

    from FlightController.Components.LDRadar_Driver import LD_Radar

    algorithm_config = RadarAlgorithmConfig(
        min_confidence=args.min_confidence,
        expected_distance_gate_cm=args.distance_gate,
        expected_angle_gate_deg=args.angle_gate,
    )
    debug_options = DebugOptions(
        enabled=bool(args.show or args.save_dir),
        show_window=args.show,
        save_directory=args.save_dir,
    )
    radar = LD_Radar()
    radar.start(com=args.radar_com, radar_type=args.radar_type)
    localizer = WarehouseRadarLocalizer(
        radar,
        algorithm_config=algorithm_config,
        debug_options=debug_options,
    )
    interval_s = 1.0 / max(args.rate_hz, 0.1)

    try:
        while True:
            request = LocalizationRequest(
                mode=args.mode,
                trusted_surfaces=surfaces,
                prior_pose=Pose2D(args.prior_x, args.prior_y, args.prior_yaw),
            )
            result = localizer.localize(request)
            raw_count = localizer.get_latest_raw_points().shape[0]
            print(
                "raw_points={} valid=({},{},{}) pose=({},{},{}) confidence={:.2f} reason={}".format(
                    raw_count,
                    result.valid_x,
                    result.valid_y,
                    result.valid_yaw,
                    None if result.x_cm is None else round(result.x_cm, 2),
                    None if result.y_cm is None else round(result.y_cm, 2),
                    None if result.yaw_deg is None else round(result.yaw_deg, 2),
                    result.confidence,
                    result.reason,
                )
            )
            for surface_id, match in result.surface_matches.items():
                print("  " + _format_match(surface_id, match))
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("stopping")
    finally:
        radar.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

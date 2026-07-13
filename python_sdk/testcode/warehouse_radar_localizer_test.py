"""Pure-logic tests for warehouse_radar_localizer.

Run with:
    python python_sdk/warehouse_radar_localizer_test.py
"""

from __future__ import annotations

import math
import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from warehouse_radar_localizer import (
    LineSegment2D,
    LocalizationMode,
    LocalizationRequest,
    Pose2D,
    RadarAlgorithmConfig,
    SurfaceID,
    WarehouseMapConfig,
    WarehouseRadarLocalizer,
    angle_diff_deg,
    body_to_world,
    build_default_surface_map,
    fit_line_ransac,
    normalize_angle_deg,
    point_to_segment_distance,
    undirected_angle_diff_deg,
    world_to_body,
)


class FakeMap:
    ACC = 3

    def __init__(self) -> None:
        total = 360 * self.ACC
        self.data = np.ones(total, dtype=np.float64) * -1.0
        self._deg_arr = np.arange(0, 360, 1 / self.ACC, dtype=float)
        self.time_stamp = np.zeros(total, dtype=np.float64)
        self.confidence_data = np.ones(total, dtype=np.float64) * 255.0
        self.update_count = 1
        self.avail_points = 0
        self.total_points = total

    def set_polar(self, angle_deg: float, distance_cm: float, confidence: float = 255.0) -> None:
        index = round((angle_deg % 360.0) * self.ACC) % self.data.size
        distance_mm = distance_cm * 10.0
        if self.data[index] < 0.0 or distance_mm < self.data[index]:
            self.data[index] = distance_mm
            self.confidence_data[index] = confidence
            self.time_stamp[index] = time.perf_counter()
        self.avail_points = int(np.count_nonzero(self.data >= 0.0))


class FakeRadar:
    def __init__(self, fake_map: FakeMap) -> None:
        self.map = fake_map


def _point_to_radar_polar_cm(point_body: np.ndarray) -> tuple[float, float]:
    angle_deg = math.degrees(math.atan2(-float(point_body[1]), float(point_body[0])))
    distance_cm = float(np.linalg.norm(point_body))
    return angle_deg, distance_cm


def _add_segment_to_map(
    fake_map: FakeMap,
    segment: LineSegment2D,
    pose: Pose2D,
    sample_count: int = 220,
    noise_cm: float = 0.15,
) -> None:
    p1 = np.asarray(segment.p1, dtype=float)
    p2 = np.asarray(segment.p2, dtype=float)
    direction = p2 - p1
    normal = np.array([-direction[1], direction[0]], dtype=float)
    normal /= max(float(np.linalg.norm(normal)), 1e-9)
    rng = np.random.default_rng(12345 + segment.surface_id.value)
    for t in np.linspace(0.02, 0.98, sample_count):
        world_point = p1 + (p2 - p1) * t + normal * rng.normal(0.0, noise_cm)
        body_point = world_to_body(world_point, pose).reshape(2)
        angle_deg, distance_cm = _point_to_radar_polar_cm(body_point)
        if 5.0 <= distance_cm <= 650.0:
            fake_map.set_polar(angle_deg, distance_cm)


def make_fake_radar(
    pose: Pose2D,
    surface_ids: tuple[SurfaceID, ...],
    custom_segments: dict[SurfaceID, LineSegment2D] | None = None,
    config: WarehouseMapConfig | None = None,
) -> FakeRadar:
    fake_map = FakeMap()
    surfaces = build_default_surface_map(config or WarehouseMapConfig())
    if custom_segments:
        surfaces.update(custom_segments)
    for surface_id in surface_ids:
        _add_segment_to_map(fake_map, surfaces[surface_id], pose)
    return FakeRadar(fake_map)


class WarehouseRadarLocalizerTest(unittest.TestCase):
    def test_body_world_transforms(self) -> None:
        points = np.array([[1.0, 0.0], [0.0, 1.0]])

        np.testing.assert_allclose(body_to_world(points, Pose2D(0.0, 0.0, 0.0)), points, atol=1e-6)
        np.testing.assert_allclose(
            body_to_world(points, Pose2D(0.0, 0.0, 90.0)),
            np.array([[0.0, -1.0], [1.0, 0.0]]),
            atol=1e-6,
        )
        np.testing.assert_allclose(
            body_to_world(points, Pose2D(0.0, 0.0, 180.0)),
            np.array([[-1.0, 0.0], [0.0, -1.0]]),
            atol=1e-6,
        )

        pose = Pose2D(23.0, -17.0, 35.0)
        world = body_to_world(points, pose)
        np.testing.assert_allclose(world_to_body(world, pose), points, atol=1e-6)

    def test_angle_normalization(self) -> None:
        self.assertEqual(normalize_angle_deg(181.0), -179.0)
        self.assertEqual(normalize_angle_deg(-181.0), 179.0)
        self.assertEqual(normalize_angle_deg(360.0), 0.0)
        self.assertEqual(angle_diff_deg(5.0, 355.0), 10.0)

    def test_point_to_segment_distance(self) -> None:
        points = np.array([[0.0, 2.0], [5.0, 0.0], [12.0, 0.0]])
        distances = point_to_segment_distance(points, np.array([0.0, 0.0]), np.array([10.0, 0.0]))
        np.testing.assert_allclose(distances, np.array([2.0, 0.0, 2.0]), atol=1e-6)

    def test_ransac_line_with_outliers(self) -> None:
        rng = np.random.default_rng(2024)
        xs = np.linspace(-60.0, 60.0, 100)
        ys = 2.0 * xs + 10.0 + rng.normal(0.0, 0.8, size=xs.shape)
        inliers = np.column_stack((xs, ys))
        outliers = rng.uniform(-90.0, 90.0, size=(30, 2))
        points = np.vstack((inliers, outliers))
        config = RadarAlgorithmConfig(ransac_distance_threshold_cm=3.0)
        fit = fit_line_ransac(points, config, rng)

        self.assertTrue(fit.valid, fit.reason)
        assert fit.direction is not None
        assert fit.point is not None
        assert fit.normal is not None
        expected_angle = math.degrees(math.atan2(-2.0, 1.0))
        got_angle = math.degrees(math.atan2(-fit.direction[1], fit.direction[0]))
        self.assertLess(undirected_angle_diff_deg(got_angle, expected_angle), 2.0)
        expected_distance = abs(-10.0 / math.sqrt(5.0))
        got_distance = abs(float((np.zeros(2) - fit.point) @ fit.normal))
        self.assertLess(abs(got_distance - expected_distance), 3.0)

    def test_takeoff_absolute_west_south(self) -> None:
        pose = Pose2D(0.0, 0.0, 0.0)
        radar = make_fake_radar(pose, (SurfaceID.WEST_NET, SurfaceID.SOUTH_NET))
        localizer = WarehouseRadarLocalizer(radar)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.ABSOLUTE_ANCHOR,
                (SurfaceID.WEST_NET, SurfaceID.SOUTH_NET),
                pose,
            )
        )

        self.assertTrue(result.absolute_fix, result.reason)
        self.assertAlmostEqual(result.x_cm or 999.0, 0.0, delta=3.0)
        self.assertAlmostEqual(result.y_cm or 999.0, 0.0, delta=3.0)
        self.assertAlmostEqual(result.yaw_deg or 999.0, 0.0, delta=2.0)

    def test_north_absolute_west_north(self) -> None:
        pose = Pose2D(0.0, 250.0, 0.0)
        radar = make_fake_radar(pose, (SurfaceID.WEST_NET, SurfaceID.NORTH_NET))
        localizer = WarehouseRadarLocalizer(radar)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.ABSOLUTE_ANCHOR,
                (SurfaceID.WEST_NET, SurfaceID.NORTH_NET),
                pose,
            )
        )

        self.assertTrue(result.absolute_fix, result.reason)
        self.assertAlmostEqual(result.x_cm or 999.0, 0.0, delta=3.0)
        self.assertAlmostEqual(result.y_cm or 999.0, 250.0, delta=3.0)
        self.assertAlmostEqual(result.yaw_deg or 999.0, 0.0, delta=2.0)

    def test_dual_shelf_corridor(self) -> None:
        pose = Pose2D(175.0, 150.0, 0.0)
        radar = make_fake_radar(pose, (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD))
        localizer = WarehouseRadarLocalizer(radar)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.CORRIDOR_TRACK,
                (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD),
                pose,
            )
        )

        self.assertTrue(result.valid_x, result.reason)
        self.assertFalse(result.valid_y)
        self.assertIsNone(result.y_cm)
        self.assertTrue(result.valid_yaw)
        self.assertAlmostEqual(result.x_cm or 999.0, 175.0, delta=3.0)

    def test_dual_shelf_width_inconsistent(self) -> None:
        pose = Pose2D(175.0, 150.0, 0.0)
        fake_cd = LineSegment2D(SurfaceID.SHELF_CD, (310.0, 25.0), (310.0, 225.0))
        radar = make_fake_radar(
            pose,
            (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD),
            custom_segments={SurfaceID.SHELF_CD: fake_cd},
        )
        config = RadarAlgorithmConfig(
            expected_distance_gate_cm=70.0,
            max_surface_distance_error_cm=70.0,
        )
        localizer = WarehouseRadarLocalizer(radar, algorithm_config=config)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.CORRIDOR_TRACK,
                (SurfaceID.SHELF_AB, SurfaceID.SHELF_CD),
                pose,
            )
        )

        self.assertFalse(result.valid_x)
        self.assertEqual(result.reason, "corridor_width_inconsistent")

    def test_single_shelf_outputs_x_and_yaw_only(self) -> None:
        pose = Pose2D(175.0, 150.0, 0.0)
        radar = make_fake_radar(pose, (SurfaceID.SHELF_CD,))
        localizer = WarehouseRadarLocalizer(radar)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.CORRIDOR_TRACK,
                (SurfaceID.SHELF_CD,),
                pose,
            )
        )

        self.assertTrue(result.valid_x, result.reason)
        self.assertTrue(result.valid_yaw)
        self.assertFalse(result.valid_y)
        self.assertIsNone(result.y_cm)
        self.assertAlmostEqual(result.x_cm or 999.0, 175.0, delta=3.0)

    def test_pose_jump_rejected(self) -> None:
        true_pose = Pose2D(250.0, 0.0, 0.0)
        prior_pose = Pose2D(100.0, 0.0, 0.0)
        radar = make_fake_radar(true_pose, (SurfaceID.WEST_NET, SurfaceID.SOUTH_NET))
        config = RadarAlgorithmConfig(
            expected_distance_gate_cm=220.0,
            max_surface_distance_error_cm=220.0,
        )
        localizer = WarehouseRadarLocalizer(radar, algorithm_config=config)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.ABSOLUTE_ANCHOR,
                (SurfaceID.WEST_NET, SurfaceID.SOUTH_NET),
                prior_pose,
                max_position_deviation_cm=5.0,
            )
        )

        self.assertFalse(result.absolute_fix)
        self.assertEqual(result.reason, "pose_jump_rejected")

    def test_point_cloud_insufficient(self) -> None:
        radar = FakeRadar(FakeMap())
        localizer = WarehouseRadarLocalizer(radar)
        result = localizer.localize(
            LocalizationRequest(
                LocalizationMode.ABSOLUTE_ANCHOR,
                (SurfaceID.WEST_NET, SurfaceID.SOUTH_NET),
                Pose2D(0.0, 0.0, 0.0),
            )
        )

        self.assertFalse(result.absolute_fix)
        self.assertFalse(result.partial_fix)


if __name__ == "__main__":
    unittest.main()

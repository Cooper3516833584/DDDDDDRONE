import sys
import threading
import time
import types
import unittest
from pathlib import Path


SDK_DIR = Path(__file__).resolve().parents[1]
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))

from fleet_bus.models import NodeFlags


class FakeLogger:
    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: None


class FakeNavigationBase:
    def _get_radar_pose(self, wait=True):
        return self._test_pose


fake_loguru = types.ModuleType("loguru")
fake_loguru.logger = FakeLogger()
fake_flight_controller = types.ModuleType("FlightController")
fake_flight_controller.FC_Controller = object
fake_components = types.ModuleType("FlightController.Components")
fake_components.LD_Radar = object
fake_solutions = types.ModuleType("FlightController.Solutions")
fake_navigation = types.ModuleType(
    "FlightController.Solutions.Navigation"
)
fake_navigation.Navigation = FakeNavigationBase
fake_marker = types.ModuleType("landing_marker_offset")
fake_marker.track_landing_marker = lambda _camera: iter(())
fake_marker.track_switchable_marker = lambda _camera, _h_mode: iter(())
stub_modules = {
    "loguru": fake_loguru,
    "FlightController": fake_flight_controller,
    "FlightController.Components": fake_components,
    "FlightController.Solutions": fake_solutions,
    "FlightController.Solutions.Navigation": fake_navigation,
    "landing_marker_offset": fake_marker,
}
original_modules = {
    name: sys.modules.get(name) for name in stub_modules
}
sys.modules.update(stub_modules)

from mission1_26_base import (
    FLEET_TRACE_BUFFER_CAPACITY,
    FLEET_TRACE_DRAIN_TIMEOUT_SECONDS,
    FLEET_TRACE_MIN_DISTANCE_CM,
    FLEET_TRACE_SAMPLE_INTERVAL_SECONDS,
    FLEET_TRACE_STATIONARY_KEEPALIVE_SECONDS,
    Mission,
    MissionFleetStateProvider,
    MissionOperationState,
    drain_terminal_fleet_trace,
)
from mission1_26_visual_descent_test import (
    SingleRadarNavigation,
)

sys.modules.pop("mission1_26_base", None)
sys.modules.pop("mission1_26_visual_descent_test", None)
for module_name, original in original_modules.items():
    if original is None:
        sys.modules.pop(module_name, None)
    else:
        sys.modules[module_name] = original


class Value:
    def __init__(self, value):
        self.value = value


class FakeFlightState:
    unlock = Value(0)
    bat = Value(11.53)
    mode = Value(0)


class FakeFlightController:
    state = FakeFlightState()


class FakeNavigation:
    current_x = 12.4
    current_y = -8.6
    current_height = 150.0
    current_yaw = 5.0

    @staticmethod
    def pose_is_fresh():
        return True


class FakeMission:
    def __init__(self, pose_ready, operation_state=MissionOperationState.READY):
        self.pose_ready = pose_ready
        self.operation_state = operation_state

    def fleet_status(self):
        return self.operation_state, 0

    def fleet_pose_ready(self):
        return self.pose_ready


class MissionFleetPoseReportingTests(unittest.TestCase):
    def test_mission_trace_density_matches_ground_batch_capacity(self):
        self.assertEqual(0.10, FLEET_TRACE_SAMPLE_INTERVAL_SECONDS)
        self.assertEqual(1200, FLEET_TRACE_BUFFER_CAPACITY)
        self.assertEqual(1.0, FLEET_TRACE_MIN_DISTANCE_CM)
        self.assertEqual(1.0, FLEET_TRACE_STATIONARY_KEEPALIVE_SECONDS)

    def test_terminal_trace_drain_uses_bounded_timeout(self):
        class FleetNode:
            timeout = None

            def wait_for_trace_drain(self, timeout):
                self.timeout = timeout
                return True

        node = FleetNode()

        drain_terminal_fleet_trace(node)

        self.assertEqual(FLEET_TRACE_DRAIN_TIMEOUT_SECONDS, node.timeout)

    def test_single_radar_pose_refreshes_navigation_timestamp(self):
        navigation = object.__new__(SingleRadarNavigation)
        navigation._test_pose = (10.0, 20.0, 15.0, True)
        navigation._last_pose_update = 0.0

        before = time.monotonic()
        pose = navigation._get_radar_pose()

        self.assertEqual(pose, (10.0, 20.0, 15.0, True))
        self.assertGreaterEqual(navigation._last_pose_update, before)

    def test_invalid_single_radar_pose_does_not_refresh_timestamp(self):
        navigation = object.__new__(SingleRadarNavigation)
        navigation._test_pose = (10.0, 20.0, 15.0, False)
        navigation._last_pose_update = 0.0

        pose = navigation._get_radar_pose()

        self.assertEqual(pose, (10.0, 20.0, 15.0, False))
        self.assertEqual(navigation._last_pose_update, 0.0)

    def test_provider_hides_pose_until_mission_frame_is_ready(self):
        provider = MissionFleetStateProvider(
            FakeFlightController(),
            FakeNavigation(),
            FakeMission(pose_ready=False),
        )

        state = provider()

        self.assertFalse(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertEqual((state.x_cm, state.y_cm, state.z_cm), (0, 0, 0))
        self.assertEqual(state.pose_quality, 0)

    def test_provider_reports_fresh_pose_after_mission_frame_is_ready(self):
        provider = MissionFleetStateProvider(
            FakeFlightController(),
            FakeNavigation(),
            FakeMission(pose_ready=True),
        )

        state = provider()

        self.assertTrue(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertEqual((state.x_cm, state.y_cm, state.z_cm), (12, -9, 150))
        self.assertEqual(state.pose_quality, 4)

    def test_ready_status_enables_pose_reporting_once(self):
        mission = object.__new__(Mission)
        mission._fleet_status_lock = threading.Lock()
        mission._fleet_operation_state = MissionOperationState.IDLE
        mission._fleet_error_code = 0
        mission._fleet_pose_ready = False

        mission.set_fleet_status(MissionOperationState.READY)
        mission.set_fleet_status(MissionOperationState.TAKEOFF)

        self.assertTrue(mission.fleet_pose_ready())

    def test_drop_completed_state_remains_busy_during_low_hover(self):
        provider = MissionFleetStateProvider(
            FakeFlightController(),
            FakeNavigation(),
            FakeMission(
                pose_ready=True,
                operation_state=(
                    MissionOperationState.MISSION1_DROP_COMPLETED
                ),
            ),
        )

        state = provider()

        self.assertTrue(state.node_flags & int(NodeFlags.BUSY))


if __name__ == "__main__":
    unittest.main()

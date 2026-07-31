"""非定点快速起飞的无硬件逻辑验证。"""

import unittest
from unittest.mock import patch

from FlightController.Solutions.Navigation import Navigation


class _Value:
    def __init__(self, value):
        self.value = value


class _State:
    def __init__(self):
        self.mode = _Value(0)
        self.unlock = _Value(False)

    @staticmethod
    def is_fresh(_max_age):
        return True


class _FC:
    PROGRAM_MODE = 3
    HOLD_POS_MODE = 2

    def __init__(self):
        self.state = _State()
        self.takeoff_calls = []

    def set_flight_mode(self, mode):
        self.state.mode.value = mode

    def unlock(self):
        self.state.unlock.value = True

    def take_off(self, height):
        self.takeoff_calls.append(height)

    @staticmethod
    def wait_for_takeoff_done(timeout_s):
        return timeout_s == 5

    @staticmethod
    def wait_for_hovering(timeout_s):
        return timeout_s == 8


class _StopEvent:
    @staticmethod
    def is_set():
        return False


class FastNonPointingTakeoffTest(unittest.TestCase):
    def test_hands_off_at_live_position_and_keeps_climbing(self):
        fc = _FC()
        navi = object.__new__(Navigation)
        navi.fc = fc
        navi.running = True
        navi.stop_event = _StopEvent()
        navi.navigation_flag = True
        navi.keep_height_flag = True
        navi.current_x = 12.5
        navi.current_y = -7.5
        navi._last_pose_update = 1.0
        waypoints = []
        heights = []
        pid_names = []
        height_waits = []
        navi._flight_state_is_fresh = lambda max_age=0.5: True
        navi.pose_is_fresh = lambda max_age=0.3: True
        navi.direct_set_waypoint = lambda point: waypoints.append(tuple(point))
        navi.set_height = lambda height: heights.append(float(height))
        navi.switch_pid = lambda name: pid_names.append(name)
        navi.wait_for_height = lambda **kwargs: (
            height_waits.append(kwargs) or True
        )

        with patch(
            "FlightController.Solutions.Navigation.time.sleep",
            return_value=None,
        ) as sleep_mock:
            navi.fast_non_pointing_takeoff(target_height=150)

        self.assertEqual(fc.takeoff_calls, [90])
        self.assertEqual(waypoints, [(12.5, -7.5)])
        self.assertEqual(heights, [150.0])
        self.assertEqual(pid_names, ["hover"])
        self.assertEqual(
            height_waits,
            [{"height_thres": 8, "timeout": 15}],
        )
        self.assertTrue(navi.navigation_flag)
        self.assertTrue(navi.keep_height_flag)
        sleep_mock.assert_any_call(2.0)


if __name__ == "__main__":
    unittest.main()

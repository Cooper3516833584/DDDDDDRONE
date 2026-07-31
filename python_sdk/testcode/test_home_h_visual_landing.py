"""H标记返航视觉微调和降落的无硬件逻辑验证。"""

import unittest

from testcode.home_h_visual_landing_support import visual_home_h_landing


class _Value:
    def __init__(self, value):
        self.value = value


class _State:
    def __init__(self):
        self.unlock = _Value(True)

    @staticmethod
    def is_fresh(_max_age):
        return True


class _FC:
    def __init__(self):
        self.state = _State()


class _StopEvent:
    @staticmethod
    def is_set():
        return False

    @staticmethod
    def wait(_seconds):
        return False


class _Offsets:
    def __init__(self, values):
        self._values = iter(values)
        self.closed = False

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._values)

    def close(self):
        self.closed = True


class _Navigation:
    def __init__(self):
        self.running = True
        self.keep_height_flag = False
        self.height_targets = []
        self.height_waits = []
        self.moves = []
        self.stop_calls = 0
        self.complete_calls = 0

    @staticmethod
    def pose_is_fresh():
        return True

    def set_height(self, height):
        self.height_targets.append(float(height))

    def wait_for_height(self, **kwargs):
        self.height_waits.append(kwargs)
        return True

    def move_by_direction(self, speed, direction_deg):
        self.moves.append((float(speed), float(direction_deg)))

    def stop_move(self):
        self.stop_calls += 1

    def complete_landing_after_approach(self):
        self.complete_calls += 1
        return True


class HomeHVisualLandingTest(unittest.TestCase):
    def test_descends_aligns_and_uses_navigation_landing_stage_two(self):
        offsets = _Offsets([(None, None), (40.0, 0.0), (20.0, 0.0)])
        navi = _Navigation()

        landed = visual_home_h_landing(
            fc=_FC(),
            navi=navi,
            stop_event=_StopEvent(),
            camera_index=0,
            tracker_factory=lambda _camera_index: offsets,
            landing_callback=navi.complete_landing_after_approach,
        )

        self.assertTrue(landed)
        self.assertEqual(navi.height_targets, [50.0])
        self.assertEqual(
            navi.height_waits,
            [{"height_thres": 8.0, "timeout": 8.0}],
        )
        self.assertEqual(navi.moves, [(15.0, 0.0)])
        self.assertGreaterEqual(navi.stop_calls, 2)
        self.assertEqual(navi.complete_calls, 1)
        self.assertTrue(offsets.closed)


if __name__ == "__main__":
    unittest.main()

"""Offline checks for the locked, no-recognition ground-chain entry."""

import ast
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


SOURCE = Path(__file__).resolve().parents[1] / "rescue_ground_chain_test.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


class FakeLogger:
    def info(self, *args):
        pass


def load_helpers(*names):
    nodes = [node for node in TREE.body
             if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {"time": time, "logger": FakeLogger()}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


class GroundChainTests(unittest.TestCase):
    def setUp(self):
        self.stop = threading.Event()
        self.fc = SimpleNamespace(
            connected=True,
            state=SimpleNamespace(is_fresh=lambda _: True,
                                  unlock=SimpleNamespace(value=False)),
        )

    def test_entry_has_no_flight_or_recognition_calls(self):
        forbidden = {
            "run", "prime", "poll", "detect", "set_flight_mode",
            "pointing_takeoff", "takeoff", "land", "open_servo",
            "navigation_to_waypoint_direct", "set_navigation_state",
            "set_keep_height_state",
        }
        calls = [node.func.attr for node in ast.walk(TREE)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)]
        self.assertTrue(forbidden.isdisjoint(calls), forbidden.intersection(calls))

    def test_non_linux_entry_rejects_before_hardware(self):
        main = next(node for node in TREE.body
                    if isinstance(node, ast.FunctionDef) and node.name == "main")
        namespace = {
            "parse_args": lambda: object(),
            "sys": SimpleNamespace(platform="win32",
                                   stdin=SimpleNamespace(isatty=lambda: True)),
            "FC_Client": lambda: self.fail("FC must not be opened"),
        }
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(SOURCE), "exec"),
             namespace)
        with self.assertRaisesRegex(RuntimeError, "Linux"):
            namespace["main"]()

    def test_lock_guard_rejects_armed_or_stale_fc(self):
        guard = load_helpers("require_locked")["require_locked"]
        guard(self.fc, self.stop)
        self.fc.state.unlock.value = True
        with self.assertRaisesRegex(RuntimeError, "unlocked"):
            guard(self.fc, self.stop)
        self.fc.state.unlock.value = False
        self.fc.state.is_fresh = lambda _: False
        with self.assertRaisesRegex(RuntimeError, "telemetry"):
            guard(self.fc, self.stop)

    def test_camera_reads_raw_frames_without_inference(self):
        helper = load_helpers("require_locked", "check_camera_frames")
        frames = iter([(True, SimpleNamespace(shape=(480, 640, 3)))] * 3)
        camera = SimpleNamespace(read=lambda: next(frames))

        class Vision:
            _camera = camera

            def open(self):
                self.opened = True

            def poll(self):
                raise AssertionError("recognition must not run")

            def prime(self):
                raise AssertionError("recognition must not run")

        vision = Vision()
        helper["check_camera_frames"](vision, self.fc, self.stop)
        self.assertTrue(vision.opened)

    def test_new_cloud_requires_revision_and_fresh_pose(self):
        helper = load_helpers("require_locked", "wait_for_new_pointcloud")
        planner = SimpleNamespace(get_debug_state=lambda: {
            "ready": True, "revision": 5, "last_endpoint_cells": 12,
            "min_valid_endpoint_cells": 5, "cloud_age_s": 0.1,
            "occupied_cells": 3,
        })
        navigation = SimpleNamespace(pose_is_fresh=lambda: True,
                                     height_is_fresh=lambda: True)
        helper["wait_for_new_pointcloud"](
            planner, 4, self.fc, navigation, self.stop, timeout=0.01)
        with self.assertRaisesRegex(RuntimeError, "point cloud"):
            helper["wait_for_new_pointcloud"](
                planner, 5, self.fc, navigation, self.stop, timeout=0.01)

    def test_wait_for_car_start_stays_locked(self):
        helper = load_helpers("require_locked", "wait_for_car_start")
        task = object()
        receiver = SimpleNamespace(wait_for_start=lambda timeout: task)
        self.assertIs(helper["wait_for_car_start"](
            receiver, self.fc, self.stop, 1.0), task)
        self.fc.state.unlock.value = True
        with self.assertRaisesRegex(RuntimeError, "unlocked"):
            helper["wait_for_car_start"](receiver, self.fc, self.stop, 1.0)


if __name__ == "__main__":
    unittest.main()

"""Navigation startup affinity checks without ROS or hardware imports."""

import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / "FlightController/Solutions/Navigation.py"


def load_navigation(platform="linux", allowed=None, bind_error=None):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Navigation")
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in {"start", "stop", "_navigation_thread_entry"}]
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    affinity = SimpleNamespace(sched_setaffinity=Mock(side_effect=bind_error),
                               sched_getaffinity=Mock(return_value={3} if allowed is None else allowed))
    namespace = {"threading": threading, "sys": SimpleNamespace(platform=platform),
                 "os": affinity, "logger": Mock()}
    exec(compile(module, str(SOURCE), "exec"), namespace)
    nav = namespace["Navigation"]()
    nav.running = False
    nav._control_lock = threading.Lock()
    nav._thread_list = []
    nav._lio_listener = object()
    nav._legacy_mode_warned = False
    nav.update_realtime_control = Mock()
    nav.entered = threading.Event()

    def loop():
        nav.entered.set()
        while nav.running:
            time.sleep(0.001)

    nav._navigation_task = loop
    nav._keep_height_task = loop
    nav._velocity_override_watchdog_task = loop
    return nav, affinity


class NavigationAffinityTests(unittest.TestCase):
    def test_start_binds_only_navigation_thread_and_rebinds_on_restart(self):
        nav, affinity = load_navigation()
        caller_ids = []
        affinity.sched_setaffinity.side_effect = lambda pid, cpus: caller_ids.append(threading.get_native_id())
        try:
            for _ in range(2):
                previous = len(nav._thread_list)
                nav.start()
                thread = nav._thread_list[previous]
                self.assertEqual(thread.name, "navigation_cpu3")
                self.assertEqual(caller_ids[-1], thread.native_id)
                self.assertNotEqual(caller_ids[-1], threading.get_native_id())
                affinity.sched_setaffinity.assert_called_with(0, {3})
                self.assertTrue(nav.entered.wait(1))
                nav.stop(join=True)
                self.assertFalse(any(t.is_alive() for t in nav._thread_list))
            self.assertEqual(affinity.sched_setaffinity.call_count, 2)
        finally:
            nav.stop(join=True)

    def test_bind_failure_refuses_start_and_never_enters_control_loop(self):
        for error in (PermissionError("denied"), OSError("CPU3 unavailable")):
            with self.subTest(error=error):
                nav, affinity = load_navigation(bind_error=error)
                with self.assertRaisesRegex(RuntimeError, "could not bind") as caught:
                    nav.start()
                self.assertIs(caught.exception.__cause__, error)
                self.assertFalse(nav.running)
                self.assertFalse(nav.entered.is_set())
                self.assertEqual(len(nav._thread_list), 1)
                self.assertFalse(nav._thread_list[0].is_alive())
                nav.update_realtime_control.assert_called_with(vel_x=0, vel_y=0, vel_z=0, yaw=0)

    def test_wrong_readback_refuses_start(self):
        nav, _ = load_navigation(allowed={0, 1, 2, 3})
        with self.assertRaisesRegex(RuntimeError, "could not bind"):
            nav.start()
        self.assertFalse(nav.running)
        self.assertFalse(nav.entered.is_set())

    def test_non_linux_offline_compatibility(self):
        nav, affinity = load_navigation(platform="win32")
        try:
            nav.start()
            self.assertTrue(nav.entered.wait(1))
            affinity.sched_setaffinity.assert_not_called()
        finally:
            nav.stop(join=True)


if __name__ == "__main__":
    unittest.main()

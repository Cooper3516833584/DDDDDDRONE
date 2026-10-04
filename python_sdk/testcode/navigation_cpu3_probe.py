"""Bounded live ROS check of Navigation with a simulated FC output sink.

No FC connection, serial device, service restart, unlock or flight commands.
Run from a temporary working directory after sourcing the ROS overlay:
    python3 /home/fc/dddddrone/python_sdk/testcode/navigation_cpu3_probe.py
"""

import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 60:
        parser.error("--seconds must be between 3 and 60")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from FlightController.Solutions.Navigation import Navigation
    from FlightController.Components.RosNode import RosNodeRunner
    from loguru import logger
    import rclpy

    # Avoid high-rate diagnostic disk logging affecting the measurement.
    logger.remove()
    logger.add(sys.stderr, level="WARNING")
    sink = SimpleNamespace(HOLD_POS_MODE=1, state=SimpleNamespace(
        mode=SimpleNamespace(value=1), unlock=SimpleNamespace(value=False),
        alt_add=SimpleNamespace(value=100), update_event=threading.Event()))
    sent = deque(maxlen=20000)
    counters = Counter()
    def capture(*control):
        if not all(math.isfinite(v) for v in control):
            counters["invalid_control"] += 1
        sent.append((time.monotonic(), control))
        counters["control_frames"] += 1
    sink.send_realtime_control_data = capture
    stop_event = threading.Event()
    nav = Navigation(fc=sink, stop_event=stop_event)
    report = {"simulated_fc": True, "debug_logging": False,
              "main_affinity": sorted(os.sched_getaffinity(0)), "rounds": []}
    try:
        for attempt in range(2):
            nav.start()
            navigation_thread = next(t for t in reversed(nav._thread_list)
                                     if t.name == "navigation_cpu3")
            tid = navigation_thread.native_id
            result = {"tid": tid, "affinity": sorted(os.sched_getaffinity(tid))}
            if result["affinity"] != [3]:
                raise RuntimeError("Navigation affinity is not CPU3")
            nav.calibrate_basepoint(wait=True)
            # These fields belong exclusively to the simulated FC object.
            sink.state.unlock.value = True
            nav.set_navigation_state(True)
            nav.navi_x_pid.setpoint = 10
            checks = Counter()
            callback_counts = Counter()
            last_received = {}
            gaps = {}
            rewinds = Counter()
            previous_stamp = {}
            # Existing subscribers retain bound methods; measure callback flow
            # with separate light subscribers on the same ROS executor.
            from nav_msgs.msg import Odometry
            from fast_lio.msg import LioHealth
            from sensor_msgs.msg import Imu
            from rclpy.qos import qos_profile_sensor_data
            listener = nav._lio_listener
            subscriptions = []
            def receive(name, msg):
                now = time.monotonic()
                if name in last_received:
                    gaps[name] = max(gaps.get(name, 0), now - last_received[name])
                last_received[name] = now
                callback_counts[name] += 1
                stamp = msg.state_stamp if name == "health" else msg.header.stamp
                value = stamp.sec * 1000000000 + stamp.nanosec
                if value <= previous_stamp.get(name, -1):
                    rewinds[name] += 1
                previous_stamp[name] = value
            for typ, topic, name in ((Odometry, "/Odometry_highrate", "odometry"),
                                      (LioHealth, "/LioHealth", "health"),
                                      (Imu, "/livox/imu", "imu")):
                subscriptions.append(listener.create_subscription(
                    typ, topic, lambda msg, n=name: receive(n, msg), qos_profile_sensor_data))
            started = time.monotonic()
            while time.monotonic() - started < args.seconds:
                sink.state.update_event.set()
                checks["samples"] += 1
                checks["fresh_pose"] += nav.lio_pose.get_pose() is not None
                checks["affinity_cpu3"] += os.sched_getaffinity(tid) == {3}
                checks["navigation_enabled"] += nav.navigation_flag
                stat = Path("/proc/self/task/{}/stat".format(tid)).read_text()
                cpu = int(stat[stat.rfind(")") + 2:].split()[36])
                checks["scheduled_cpu3"] += cpu == 3
                time.sleep(0.02)
            elapsed = time.monotonic() - started
            result.update(seconds=elapsed, checks=dict(checks),
                          callback_hz={k: v / elapsed for k, v in callback_counts.items()},
                          max_callback_gap_ms={k: v * 1000 for k, v in gaps.items()},
                          nonmonotonic_stamps=dict(rewinds),
                          other_thread_affinities={str(t.native_id): sorted(os.sched_getaffinity(t.native_id))
                                                   for t in nav._thread_list if t.is_alive() and t is not navigation_thread})
            # Simulate stale pose without touching any production ROS service.
            previous_get_pose = nav.lio_pose.get_pose
            nav.lio_pose.get_pose = lambda: None
            time.sleep(0.15)
            result["stale_pose_disabled_navigation"] = not nav.navigation_flag
            result["stale_pose_zero_horizontal"] = all(sent[-1][1][i] == 0 for i in (0, 1, 3))
            nav.lio_pose.get_pose = previous_get_pose
            stop_event.set()
            time.sleep(0.1)
            result["stop_event_zero_horizontal"] = all(sent[-1][1][i] == 0 for i in (0, 1, 3))
            nav.stop(join=True)
            result["threads_stopped"] = not any(t.is_alive() for t in nav._thread_list)
            result["final_zero"] = sent[-1][1] == (0, 0, 0, 0)
            for sub in subscriptions:
                listener.destroy_subscription(sub)
            sink.state.unlock.value = False
            stop_event.clear()
            report["rounds"].append(result)
        report["controls"] = dict(counters)
        report["passed"] = all(
            r["checks"]["fresh_pose"] == r["checks"]["samples"]
            and r["checks"]["affinity_cpu3"] == r["checks"]["samples"]
            and r["checks"]["scheduled_cpu3"] == r["checks"]["samples"]
            and r["checks"]["navigation_enabled"] == r["checks"]["samples"]
            and all(190 < hz < 210 for hz in r["callback_hz"].values())
            and len(r["callback_hz"]) == 3
            and not any(r["nonmonotonic_stamps"].values())
            and r["stale_pose_disabled_navigation"] and r["stale_pose_zero_horizontal"]
            and r["stop_event_zero_horizontal"]
            and r["threads_stopped"] and r["final_zero"]
            for r in report["rounds"]) and not counters["invalid_control"]
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        nav.stop(join=True)
        RosNodeRunner().stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

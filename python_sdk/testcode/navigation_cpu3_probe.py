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
    parser.add_argument("--debug-logging", action="store_true",
                        help="Retain the production debug file sinks during this bounded test")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 60:
        parser.error("--seconds must be between 3 and 60")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from FlightController.Solutions.Navigation import Navigation
    from FlightController.Components.RosNode import RosNodeRunner
    from loguru import logger
    import rclpy

    # Avoid high-rate diagnostic disk logging affecting the measurement.
    if not args.debug_logging:
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
        counters["nonzero_horizontal_frames"] += any(control[i] != 0 for i in (0, 1, 3))
    sink.send_realtime_control_data = capture
    stop_event = threading.Event()
    nav = Navigation(fc=sink, stop_event=stop_event)
    report = {"simulated_fc": True, "debug_logging": args.debug_logging,
              "main_affinity": sorted(os.sched_getaffinity(0)), "rounds": []}
    measuring = False
    callback_counts = Counter()
    last_received = {}
    gaps = {}
    rewinds = Counter()
    previous_stamp = {}
    callback_ages = {}
    imu_stats = Counter()
    gyro_max = 0.0
    for method, name in (("on_odometry", "odometry"), ("on_health", "health"), ("on_imu", "imu")):
        original = getattr(nav.lio_pose, method)
        def receive(msg, n=name, callback=original):
            nonlocal gyro_max
            now = time.monotonic()
            callback(msg)
            if not measuring:
                return
            if n in last_received:
                gaps[n] = max(gaps.get(n, 0), now - last_received[n])
            last_received[n] = now
            callback_counts[n] += 1
            stamp = msg.state_stamp if n == "health" else msg.header.stamp
            value = stamp.sec * 1000000000 + stamp.nanosec
            if value <= previous_stamp.get(n, -1):
                rewinds[n] += 1
            previous_stamp[n] = value
            callback_ages.setdefault(n, deque(maxlen=12000)).append((time.time_ns() - value) / 1e6)
            if n == "imu":
                g = msg.angular_velocity
                norm = math.sqrt(g.x*g.x + g.y*g.y + g.z*g.z)
                gyro_max = max(gyro_max, norm)
                imu_stats["gyro_over_002"] += norm > 0.02
        setattr(nav.lio_pose, method, receive)
    try:
        for attempt in range(2):
            nav.start()
            navigation_thread = next(t for t in reversed(nav._thread_list)
                                     if t.name == "navigation_cpu3")
            tid = navigation_thread.native_id
            result = {"tid": tid, "affinity": sorted(os.sched_getaffinity(tid))}
            if result["affinity"] != [3]:
                raise RuntimeError("Navigation affinity is not CPU3")
            # Wait longer in this diagnostic, retaining the production IMU,
            # mount, health and calibration gates unchanged.
            calibration_deadline = time.monotonic() + 25
            while True:
                sink.state.update_event.set()
                try:
                    nav.calibrate_basepoint(wait=False)
                    break
                except RuntimeError as exc:
                    if time.monotonic() >= calibration_deadline:
                        result.update(calibration_error=str(exc),
                                      stationary_ready=nav.lio_pose._stationary_ready,
                                      lost_latched=nav.lio_pose._lost_latched)
                        report["rounds"].append(result)
                        report["passed"] = False
                        print(json.dumps(report, indent=2))
                        return 1
                    time.sleep(0.05)
            before_nonzero = counters["nonzero_horizontal_frames"]
            # These fields belong exclusively to the simulated FC object.
            sink.state.unlock.value = True
            nav.set_navigation_state(True)
            nav.navi_x_pid.setpoint = 10
            pid_start = time.monotonic()
            while not (all(pid.auto_mode for pid in
                           (nav.navi_x_pid, nav.navi_y_pid, nav.yaw_pid))
                       and counters["nonzero_horizontal_frames"] > before_nonzero):
                if time.monotonic() - pid_start > 0.2:
                    raise RuntimeError("Navigation did not start producing simulated PID output")
                sink.state.update_event.set()
                time.sleep(0.001)
            result["pid_startup_ms"] = (time.monotonic() - pid_start) * 1000
            before_nonzero = counters["nonzero_horizontal_frames"]
            checks = Counter()
            callback_counts = Counter()
            last_received = {}
            gaps = {}
            rewinds = Counter()
            previous_stamp = {}
            callback_ages = {}
            imu_stats = Counter()
            gyro_max = 0.0
            measuring = True
            started = time.monotonic()
            while time.monotonic() - started < args.seconds:
                sink.state.update_event.set()
                checks["samples"] += 1
                checks["fresh_pose"] += nav.lio_pose.get_pose() is not None
                checks["affinity_cpu3"] += os.sched_getaffinity(tid) == {3}
                checks["navigation_enabled"] += nav.navigation_flag
                checks["pids_enabled"] += all(pid.auto_mode for pid in
                                               (nav.navi_x_pid, nav.navi_y_pid, nav.yaw_pid))
                stat = Path("/proc/self/task/{}/stat".format(tid)).read_text()
                cpu = int(stat[stat.rfind(")") + 2:].split()[36])
                checks["scheduled_cpu3"] += cpu == 3
                time.sleep(0.02)
            elapsed = time.monotonic() - started
            measuring = False
            nonzero_frames = counters["nonzero_horizontal_frames"] - before_nonzero
            nonzero_times = [stamp for stamp, control in list(sent) if stamp >= started
                             and any(control[i] != 0 for i in (0, 1, 3))]
            result.update(seconds=elapsed, checks=dict(checks),
                          nonzero_horizontal_frames=nonzero_frames,
                          max_nonzero_control_gap_ms=max(
                              ((b-a)*1000 for a, b in zip(nonzero_times, nonzero_times[1:])), default=None),
                          callback_hz={k: v / elapsed for k, v in callback_counts.items()},
                          max_callback_gap_ms={k: v * 1000 for k, v in gaps.items()},
                          max_source_age_ms={k: max(v) for k, v in callback_ages.items()},
                          imu_stats=dict(imu_stats), gyro_max_rad_s=gyro_max,
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
            sink.state.unlock.value = False
            stop_event.clear()
            report["rounds"].append(result)
        report["controls"] = dict(counters)
        report["passed"] = all(
            r["checks"]["fresh_pose"] == r["checks"]["samples"]
            and r["checks"]["affinity_cpu3"] == r["checks"]["samples"]
            and r["checks"]["scheduled_cpu3"] == r["checks"]["samples"]
            and r["checks"]["navigation_enabled"] == r["checks"]["samples"]
            and r["checks"]["pids_enabled"] == r["checks"]["samples"]
            and r["nonzero_horizontal_frames"] > r["seconds"] * 50
            and r["max_nonzero_control_gap_ms"] < 100
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
        runner = RosNodeRunner()
        runner._excuter.shutdown(timeout_sec=2)
        # Drain callbacks before destroying their nodes; Humble shutdown does
        # not itself join the Python executor worker pool.
        runner._excuter._executor.shutdown(wait=True)
        runner.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

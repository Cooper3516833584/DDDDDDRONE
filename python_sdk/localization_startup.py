"""One-shot ground startup check. The server itself never creates a DDS context.

Run --probe only for bounded, passive subscriptions; it sends no FC commands.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import fastlio_control as control


TOPICS = ("/livox/lidar", "/livox/imu", "/Odometry", "/Odometry_highrate", "/LioHealth")


def require_idle_ground(fc, mission_running):
    if not fc.connected or not fc.state.is_fresh(0.5):
        raise RuntimeError("Startup recovery requires fresh connected FC telemetry")
    if bool(fc.state.unlock.value):
        raise RuntimeError("Startup recovery refused while aircraft is armed")
    if mission_running():
        raise RuntimeError("Startup recovery refused while a mission session exists")


def ipc_cleanup_disabled():
    """Read the merged configuration; a reload/install is a separate operation."""
    text = control._run_recovery_command(
        ["systemd-analyze", "cat-config", "systemd/logind.conf"])
    section, value = "", "yes"
    for line in text.splitlines():
        line = line.strip()
        if line.startswith(("#", ";")):
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
        elif section == "Login" and "=" in line:
            key, setting = line.split("=", 1)
            if key.strip() == "RemoveIPC":
                value = setting.strip().lower()
    return value in ("no", "false", "0", "off")


def probe_localization(timeout=8):
    # A child exits completely so server_ros does not retain ROS libraries or
    # shared memory that would block the task's existing DDS recovery audit.
    result = subprocess.run(
        [sys.executable, "-B", str(Path(__file__).resolve()), "--probe", str(timeout)],
        capture_output=True, text=True, timeout=timeout + 8, check=False,
    )
    if result.returncode:
        raise RuntimeError("Passive localization probe failed: " + result.stderr.strip()[-800:])
    return json.loads(result.stdout)


def audit_startup_owners():
    audit = Path(__file__).with_name("localization_dds_audit.py")
    command = ["/usr/bin/python3", "-B", str(audit), "--released-task-pid",
               str(os.getpid()), "--details"]
    if os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    report = json.loads(control._run_recovery_command(command))
    groups = []
    for service in (control.DRIVER_SERVICE, control.FASTLIO_SERVICE):
        text = control._systemctl("show", service, "--property=ControlGroup", "--value")
        group = text.strip()
        if not group.startswith("/system.slice/") or "\n" in group:
            raise RuntimeError("Cannot identify localization service cgroup: " + service)
        groups.append(group)
    foreign = [owner for owner in report["owners"] if not any(
        cg == group or cg.startswith(group + "/")
        for cg in owner["cgroups"] for group in groups)]
    if report["unreadable"] or foreign:
        raise RuntimeError("Close other ROS users before startup recovery: " + json.dumps(report))
    return report


def prepare_localization_at_startup(fc, mission_running):
    """Check once before screen callbacks/client serving; repair proven SHM loss.

    Caller serializes mission/flight actions. Unknown sensor failures are only
    reported; never periodically restart localization or resume flight control.
    Existing tasks still reset their map and perform ground calibration.
    """
    if sys.platform != "linux":
        raise RuntimeError("Localization startup recovery requires Linux")
    guard = lambda: require_idle_ground(fc, mission_running)
    guard()
    if not ipc_cleanup_disabled():
        raise RuntimeError("Install the RemoveIPC=no deployment fix before startup recovery")
    report = probe_localization()
    guard()
    if report["ready"]:
        return dict(report, repaired=False)
    # Recover only the diagnosed condition, not a bad sensor/pose by blind restart.
    with control._dds_recovery_lock():
        guard()
        audit = audit_startup_owners()
        if not any(owner["deleted_shm"] for owner in audit["owners"]):
            raise RuntimeError("Localization unavailable without deleted DDS memory: " + json.dumps(report))
        before = {s: control._service_identity(s) for s in
                  (control.DRIVER_SERVICE, control.FASTLIO_SERVICE)}
        identities = control._rebuild_localization_services(before, guard)
        report = probe_localization()
        guard()
        if not report["ready"]:
            raise RuntimeError("Localization still unavailable after one recovery: " + json.dumps(report))
        return dict(report, repaired=True, services=identities)


def passive_probe(timeout):
    """Isolated ROS child: observe real flow and the existing paired pose gate."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from fast_lio.msg import LioHealth

    source = Path(__file__).parent / "FlightController/Components/LioPoseProvider.py"
    spec = importlib.util.spec_from_file_location("startup_pose_provider", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pose = module.LioPoseProvider()
    counts, last = dict.fromkeys(TOPICS, 0), dict.fromkeys(TOPICS, 0.0)

    def receive(topic, callback=None):
        def on_message(msg):
            counts[topic] += 1
            last[topic] = time.monotonic()
            if callback is not None:
                callback(msg)
        return on_message

    rclpy.init()
    node = None
    try:
        node = Node("localization_startup_probe_%d" % os.getpid())
        subscriptions = [node.create_subscription(
            kind, topic, receive(topic, callback), qos_profile_sensor_data,
            **({"raw": True} if topic == "/livox/lidar" else {}))
            for kind, topic, callback in (
                (CustomMsg, TOPICS[0], None), (Imu, TOPICS[1], pose.on_imu),
                (Odometry, TOPICS[2], None), (Odometry, TOPICS[3], pose.on_odometry),
                (LioHealth, TOPICS[4], pose.on_health))]
        deadline, ready = time.monotonic() + timeout, False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            ready = (all(counts[t] >= 3 and now - last[t] < 0.5 for t in TOPICS)
                     and pose._healthy(now))
            if ready:
                break
        return {"ready": ready, "counts": counts}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=float, required=True)
    args = parser.parse_args()
    if not 0 < args.probe <= 30:
        parser.error("Probe duration must be between zero and 30 seconds")
    print(json.dumps(passive_probe(args.probe)))

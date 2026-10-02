"""Ground-only FAST-LIO service restart for tasks with an existing FC connection.

Importing this module opens no serial device and starts no ROS/flight threads.
Call restart_and_calibrate_fastlio(fc, navigation) before enabling navigation.
"""

import math
import os
import subprocess
import sys
import threading
import time


FASTLIO_SERVICE = "mid360s-fastlio.service"
DRIVER_SERVICE = "mid360s-driver.service"
_restart_lock = threading.RLock()


def require_ground_restart(fc, navigation):
    if not fc.connected or not fc.state.is_fresh(0.5):
        raise RuntimeError("FAST-LIO restart requires fresh connected FC telemetry")
    if bool(fc.state.unlock.value):
        raise RuntimeError("FAST-LIO restart refused while aircraft is armed")
    if navigation.fc is not fc:
        raise RuntimeError("Navigation and restart must use the same FC connection")
    if (navigation.navigation_flag or navigation.keep_height_flag or
            navigation.traj_running_event.is_set() or
            getattr(navigation, "_velocity_override_active", False)):
        raise RuntimeError("Disable navigation, height control and trajectory before ground restart")
    stop = getattr(navigation, "stop_event", None)
    if stop is not None and stop.is_set():
        raise RuntimeError("Task stop requested; localization restart cancelled")


def _systemctl(*arguments, privileged=False, timeout=5):
    command = ["systemctl", *arguments]
    if privileged and os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError("FAST-LIO service command failed: " + (result.stderr or result.stdout).strip())
    return result.stdout


def _service_identity():
    text = _systemctl("show", FASTLIO_SERVICE, "--property=ActiveState,MainPID,InvocationID")
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def restart_fastlio_for_task(fc, navigation):
    """Restart only FAST-LIO and revoke calibration; never enable control.

    The caller must keep the aircraft disarmed and serialize flight actions.
    Requires a Linux systemd deployment and noninteractive restart permission.
    A successful result describes service startup, not valid navigation pose.
    """
    if sys.platform != "linux":
        raise RuntimeError("Task service restart is supported on the Linux flight computer")
    if not _restart_lock.acquire(blocking=False):
        raise RuntimeError("FAST-LIO restart is already in progress")
    try:
        require_ground_restart(fc, navigation)
        _systemctl("is-active", "--quiet", DRIVER_SERVICE)
        before = _service_identity()
        require_ground_restart(fc, navigation)
        navigation.lio_pose.invalidate_for_restart()
        _systemctl("restart", FASTLIO_SERVICE, privileged=True, timeout=30)
        require_ground_restart(fc, navigation)
        after = _service_identity()
        if (after.get("ActiveState") != "active" or int(after.get("MainPID", "0")) <= 0 or
                not after.get("InvocationID") or after["InvocationID"] == before.get("InvocationID")):
            raise RuntimeError("A new active FAST-LIO service instance was not confirmed")
        return {"service": FASTLIO_SERVICE, "pid": int(after["MainPID"]),
                "invocation_id": after["InvocationID"], "restart_completed_ns": time.time_ns(),
                "needs_calibration": True}
    finally:
        _restart_lock.release()


def restart_and_calibrate_fastlio(fc, navigation, *, timeout=45):
    """Ground task preparation: new map, fresh pose, existing stationarity gates.

    navigation.start() must already have installed the normal ROS listeners;
    navigation/height/trajectory flags must remain disabled. No automatic resume.
    On failure the old basepoint remains invalid and the exception reaches the task.
    """
    if not _restart_lock.acquire(blocking=False):
        raise RuntimeError("FAST-LIO restart/calibration is already in progress")
    try:
        return _restart_and_calibrate(fc, navigation, timeout)
    finally:
        _restart_lock.release()


def _restart_and_calibrate(fc, navigation, timeout):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Calibration timeout must be finite and positive")
    if getattr(navigation, "_lio_listener", None) is None:
        raise RuntimeError("Start the Navigation ROS listener before restart/calibration")
    if not navigation.lio_pose.mount_reviewed:
        raise RuntimeError("Reviewed body mount is required for task calibration")
    result = restart_fastlio_for_task(fc, navigation)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            require_ground_restart(fc, navigation)
            try:
                navigation.calibrate_basepoint(wait=False)
            except RuntimeError:
                time.sleep(0.05)
                continue
            snapshot = navigation.lio_pose.get_snapshot()
            if snapshot is not None and snapshot["stamp_ns"] >= result["restart_completed_ns"]:
                require_ground_restart(fc, navigation)
                return dict(result, needs_calibration=False, pose=snapshot)
            time.sleep(0.05)
        raise RuntimeError("New-map stationary tracking/calibration timed out")
    except BaseException:
        navigation.lio_pose.invalidate_for_restart()
        raise

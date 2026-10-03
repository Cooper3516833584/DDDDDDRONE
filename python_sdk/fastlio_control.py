"""Ground-only FAST-LIO service restart for tasks with an existing FC connection.

Importing this module opens no serial device and starts no ROS/flight threads.
Call restart_localization_for_task(fc, navigation) for full DDS recovery, or
restart_and_calibrate_fastlio(fc, navigation) for a FAST-LIO-only new map.
"""

import math
import json
import os
from contextlib import contextmanager
from pathlib import Path
import shutil
import stat
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


def _service_identity(service=FASTLIO_SERVICE):
    text = _systemctl("show", service, "--property=ActiveState,MainPID,InvocationID")
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
    return _wait_for_calibration(fc, navigation, result, timeout)


def _wait_for_calibration(fc, navigation, result, timeout):
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


@contextmanager
def _dds_recovery_lock():
    """Serialize whole-stack recovery across this user's task processes."""
    import fcntl

    path = "/tmp/robocup-localization-recovery-%d.lock" % os.getuid()
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if info.st_uid != os.getuid() or not stat.S_ISREG(info.st_mode):
            raise RuntimeError("Unsafe localization recovery lock file")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another process is recovering localization") from exc
        yield
    finally:
        os.close(fd)


def _run_recovery_command(command, timeout=15):
    result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError("Localization recovery command failed: " +
                           (result.stderr or result.stdout).strip())
    return result.stdout


def _assert_dds_released(*, timeout=0, ground_check=None):
    """Read-only, host-wide ownership audit before the official SHM cleaner.

    Other users' /proc maps require privilege. Do not kill unowned processes or
    blindly remove /dev/shm files. The task may retain loaded libraries after
    context shutdown, but must no longer map any DDS shared memory itself.
    A bounded wait allows a stopped daemon to finish exiting. Every retry must
    pass the same ownership audit; the optional ground guard runs before each.
    """
    audit = Path(__file__).with_name("localization_dds_audit.py")
    command = ["/usr/bin/python3", str(audit), "--released-task-pid", str(os.getpid())]
    if os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    deadline = time.monotonic() + timeout
    while True:
        if ground_check is not None:
            ground_check()
        report = json.loads(_run_recovery_command(command))
        if not report["owners"] and not report["unreadable"]:
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("DDS users remain; close them before retrying recovery: " +
                               json.dumps(report, ensure_ascii=False))
        time.sleep(min(0.1, remaining))


def _localization_runner():
    from FlightController.Components.RosNode import RosNodeRunner
    return RosNodeRunner()


def restart_localization_for_task(fc, navigation, *, timeout=45):
    """Ground-only driver + FAST-LIO + DDS recovery, followed by calibration.

    Reuse the task's FC/Navigation objects after navigation.start(), with all
    control flags off and a reviewed body mount. Call outside ROS callbacks;
    serialize flight actions and ROS node creation with this operation. Close
    other local ROS apps first. No FC commands or automatic control resume.

    timeout bounds the final fresh-pose/stationarity wait; service commands and
    executor shutdown each have separate bounded timeouts. On failure the old
    basepoint stays invalid. Services already stopped are not silently resumed.
    """
    if sys.platform != "linux":
        raise RuntimeError("DDS recovery requires the Linux flight computer")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Calibration timeout must be finite and positive")
    if not _restart_lock.acquire(blocking=False):
        raise RuntimeError("Localization restart is already in progress")
    try:
        with _dds_recovery_lock():
            require_ground_restart(fc, navigation)
            if not navigation.lio_pose.mount_reviewed:
                raise RuntimeError("Reviewed body mount is required for task calibration")
            listener = getattr(navigation, "_lio_listener", None)
            if listener is None:
                raise RuntimeError("Start the Navigation ROS listener before DDS recovery")
            for tool in ("ros2", "fastdds", "systemctl"):
                if shutil.which(tool) is None:
                    raise RuntimeError("Missing recovery tool; source ROS environment: " + tool)
            runner = _localization_runner()
            runner.validate_localization_recovery(listener)
            before = {s: _service_identity(s) for s in (DRIVER_SERVICE, FASTLIO_SERVICE)}
            released = False
            restore_attempted = False
            navigation.lio_pose.invalidate_for_restart()
            try:
                require_ground_restart(fc, navigation)
                runner.release_localization_context(listener)
                released = True
                navigation._lio_listener = None
                require_ground_restart(fc, navigation)
                _systemctl("stop", FASTLIO_SERVICE, DRIVER_SERVICE, privileged=True, timeout=45)
                for service in (DRIVER_SERVICE, FASTLIO_SERVICE):
                    stopped = _service_identity(service)
                    if (stopped.get("ActiveState") not in ("inactive", "failed") or
                            int(stopped.get("MainPID", "0")) != 0):
                        raise RuntimeError("Localization service did not stop: " + service)
                require_ground_restart(fc, navigation)
                _run_recovery_command(["ros2", "daemon", "stop"])
                _assert_dds_released(
                    timeout=5, ground_check=lambda: require_ground_restart(fc, navigation))
                require_ground_restart(fc, navigation)
                # ROS Humble ships fastdds as a shell wrapper without a
                # shebang on this image; invoke the wrapper through bash.
                _run_recovery_command(["bash", shutil.which("fastdds"), "shm", "clean"])
                _assert_dds_released()
                for service in (DRIVER_SERVICE, FASTLIO_SERVICE):
                    require_ground_restart(fc, navigation)
                    _systemctl("start", service, privileged=True, timeout=30)
                completed = time.time_ns()
                identities = {s: _service_identity(s) for s in before}
                for service, identity in identities.items():
                    if (identity.get("ActiveState") != "active" or
                            int(identity.get("MainPID", "0")) <= 0 or
                            not identity.get("InvocationID") or
                            identity["InvocationID"] == before[service].get("InvocationID")):
                        raise RuntimeError("New service instance not confirmed: " + service)
                restore_attempted = True
                navigation._lio_listener = runner.restore_localization_context(
                    navigation.lio_pose.on_odometry, navigation.lio_pose.on_health,
                    navigation.lio_pose.on_imu)
                released = False
                result = {"services": identities, "dds_rebuilt": True,
                          "restart_completed_ns": completed, "needs_calibration": True}
                return _wait_for_calibration(fc, navigation, result, timeout)
            except BaseException:
                navigation.lio_pose.invalidate_for_restart()
                raise
            finally:
                if released and not restore_attempted:
                    # Restore only passive subscriptions so the task can report
                    # failure/retry. Do not restart stopped services or control.
                    failure = sys.exc_info()[1]
                    try:
                        navigation._lio_listener = runner.restore_localization_context(
                            navigation.lio_pose.on_odometry, navigation.lio_pose.on_health,
                            navigation.lio_pose.on_imu)
                    except Exception as exc:
                        raise RuntimeError(
                            "Localization recovery failed (%s); ROS listener restore also failed; "
                            "restart the task process: %s" % (failure, exc)) from exc
    finally:
        _restart_lock.release()

"""Safe idle dispatcher for selecting and launching one D-task mission.

Importing this module is hardware-free.  The dispatcher owns the flight
controller only while it is waiting for a mission selection; the selected
mission is then launched as a separate process after all parent resources are
closed.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from fleet_bus.models import (
    AirFleetState,
    CommandId,
    MissionId,
    NodeFlags,
)


LOG = logging.getLogger("d-task-dispatcher")
ROOT = Path(__file__).resolve().parent
MISSION_ENTRY = {
    MissionId.MISSION1: ROOT / "mission1_26.py",
    MissionId.MISSION2: ROOT / "mission2_26.py",
}
DISPATCHER_READY = 30
DISPATCHER_SWITCHING = 31
DISPATCHER_FAULT = 32
FC_SERIAL_DEV = "/dev/ttyACM0"


def mission_entry(mission_id: MissionId) -> Path:
    """Return the fixed, non-configurable entry for a selected mission."""

    try:
        return MISSION_ENTRY[MissionId(mission_id)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("unsupported mission id") from exc


class _SingleInstance:
    def __init__(self, path: Path):
        self.path = path
        self._handle = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("a+")
        try:
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(self._handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(self._handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError) as exc:
            self._handle.close()
            self._handle = None
            raise RuntimeError("another d_task_dispatcher instance is running") from exc
        return self

    def __exit__(self, *_args):
        if self._handle is None:
            return
        try:
            if os.name != "nt":
                import fcntl

                fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None


class Dispatcher:
    def __init__(self, *, serial_dev: str = FC_SERIAL_DEV):
        self.serial_dev = serial_dev
        self.stop_event = threading.Event()
        self.child = None
        self._child_stop_requested = False

    @staticmethod
    def _state_provider(fc):
        def provider():
            state = getattr(fc, "state", None)
            flags = int(NodeFlags.READY)
            if state is not None and getattr(getattr(state, "unlock", None), "value", False):
                flags = 0
            return AirFleetState(
                node_flags=flags,
                node_uptime_ms=0,
                operation_state=(
                    DISPATCHER_READY if flags else DISPATCHER_FAULT
                ),
                error_code=0 if flags else 1,
            )

        return provider

    def request_stop(self, *_args):
        self.stop_event.set()
        child = self.child
        if child is not None and child.poll() is None and not self._child_stop_requested:
            self._child_stop_requested = True
            try:
                child.send_signal(signal.SIGINT)
            except OSError:
                LOG.exception("failed to send SIGINT to mission child")

    def _stop_child(self):
        child = self.child
        if child is None or child.poll() is not None:
            return
        self._child_stop_requested = True
        try:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            LOG.warning("mission child did not stop after SIGINT; sending SIGTERM")
            child.terminate()
            try:
                child.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                LOG.error("mission child still running after SIGTERM")

    def _launch(self, mission_id: MissionId):
        entry = mission_entry(mission_id)
        if not entry.is_file():
            raise FileNotFoundError(str(entry))
        self.child = subprocess.Popen(
            [sys.executable, "-u", str(entry)],
            cwd=str(ROOT),
            start_new_session=True,
        )
        self._child_stop_requested = False
        LOG.info("started %s (pid=%d)", entry.name, self.child.pid)

    def run_once(self):
        """Run one hardware session; return False when shutdown was requested."""

        from FlightController import FC_Controller
        from fleet_bus.air_node import attach_air_fleet_node

        fc = FC_Controller()
        node = None
        try:
            fc.start_listen_serial(serial_dev=self.serial_dev, print_state=False)
            if not fc.wait_for_connection(timeout_s=10):
                raise RuntimeError("flight-controller connection timeout")
            safe = fc.state.is_fresh(0.5) and not fc.state.unlock.value
            if not safe:
                LOG.error("dispatcher fault: telemetry stale or flight controller unlocked")
            node = attach_air_fleet_node(
                fc,
                navigation=object(),
                stop_event=self.stop_event,
                readonly=True,
                allowed_readonly_command_ids=(CommandId.DRONE_SELECT_MISSION,),
                state_provider=self._state_provider(fc),
            )
            if not safe:
                self.stop_event.wait(0.5)
                return not self.stop_event.is_set()
            LOG.info("D-task dispatcher ready; waiting for mission selection")
            while not self.stop_event.is_set():
                command = node.command_queue.receive(timeout=0.2)
                if command is None:
                    continue
                if command.command_id == int(CommandId.TARGETED_STOP):
                    node.command_queue.complete(command)
                    return False
                if command.command_id != int(CommandId.DRONE_SELECT_MISSION):
                    node.command_queue.fail(command, error_code=1)
                    continue
                try:
                    mission_id = MissionId(command.command_body)
                except (TypeError, ValueError):
                    node.command_queue.fail(command, error_code=1)
                    continue
                node.command_queue.complete(command)
                # Returning from this method lets the finally block release
                # the FleetBus transport and FC serial before the child opens it.
                return mission_id
            return False
        finally:
            if node is not None:
                node.close()
            try:
                fc.close()
            except Exception:
                LOG.exception("failed to close flight controller")

    def _switch_to_mission(self, mission_id: MissionId) -> bool:
        if self.stop_event.is_set():
            return False
        self._launch(mission_id)
        try:
            while not self.stop_event.is_set():
                if self.child.poll() is not None:
                    LOG.info("mission child exited with status %s", self.child.returncode)
                    return True
                self.stop_event.wait(0.2)
            self._stop_child()
            return False
        finally:
            self.child = None

    def run_forever(self):
        with _SingleInstance(ROOT / ".d_task_dispatcher.lock"):
            while not self.stop_event.is_set():
                try:
                    result = self.run_once()
                    if isinstance(result, MissionId):
                        keep_running = self._switch_to_mission(result)
                    else:
                        keep_running = bool(result)
                except Exception:
                    LOG.exception("dispatcher session failed; retrying")
                    keep_running = not self.stop_event.wait(2.0)
                if not keep_running:
                    break


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    dispatcher = Dispatcher()
    signal.signal(signal.SIGINT, dispatcher.request_stop)
    signal.signal(signal.SIGTERM, dispatcher.request_stop)
    dispatcher.run_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Car-controlled task reception. RX enqueues; the fleet worker handles commands."""
import logging
import struct
import threading
import time

from .air_node import AirFleetNode, attach_air_fleet_node
from .drop_task_protocol import decode_drop_start, decode_drop_task
from .models import AckReason, AckStatus, CommandId, NodeId
from .protocol import decode_command, ProtocolError

LOG = logging.getLogger(__name__)


class RescueTaskReceiver:
    def __init__(self, stop_event, *, allow_start=False, payload=None, fc=None,
                 navigation=None, validate_plan=None):
        self.stop_event = stop_event
        self.allow_start = allow_start
        self._condition = threading.Condition()
        self._task = None
        self._session = None
        self._started = False
        self.payload, self.fc, self.navigation = payload, fc, navigation
        self._validate_plan = validate_plan
        self._payload_lock = threading.RLock()
        self.run_id = None
        self._run_session = None
        self.close_done = False
        self.remap_state = 'idle'
        self.remap_error = None
        self.remap_result = None
        self._remap_worker = None

    def _ground_ready(self):
        return (not self.stop_event.is_set() and not self._started and
                self.fc is not None and self.fc.connected and
                self.fc.state.is_fresh(0.5) and not self.fc.state.unlock.value)

    def close_payload(self, session, run_id):
        with self._payload_lock:
            if not self._ground_ready() or self.payload is None:
                return AckStatus.REJECTED, AckReason.NOT_READY
            if self.run_id is not None:
                if (run_id, session) != (self.run_id, self._run_session):
                    return AckStatus.REJECTED, AckReason.BUSY
                return ((AckStatus.COMPLETED, AckReason.NONE) if self.close_done else
                        (AckStatus.FAILED, AckReason.INTERNAL_ERROR))
            self.run_id, self._run_session = run_id, session
            try:
                self.payload.close_all()
                if self.stop_event.is_set():
                    self.payload.open_all()
                    return AckStatus.REJECTED, AckReason.NOT_READY
                self.close_done = True
                LOG.info("DRONE CLOSE SENT run_id=%d (no mechanical feedback)", run_id)
                return AckStatus.COMPLETED, AckReason.NONE
            except Exception:
                LOG.exception("DRONE CLOSE failed run_id=%d", run_id)
                return AckStatus.FAILED, AckReason.INTERNAL_ERROR

    def rebuild_map(self, session, run_id):
        with self._condition:
            if (not self._ground_ready() or not self.close_done or
                    (run_id, session) != (self.run_id, self._run_session)):
                return AckStatus.REJECTED, AckReason.NOT_READY
            if self.remap_state == 'idle':
                self.remap_state = 'running'
                self._remap_worker = threading.Thread(target=self._rebuild,
                    name='rescue-remap', daemon=True)
                self._remap_worker.start()
                return AckStatus.ACCEPTED, AckReason.NONE
            if self.remap_state == 'ready':
                return AckStatus.COMPLETED, AckReason.NONE
            if self.remap_state == 'running':
                return AckStatus.ACCEPTED, AckReason.NONE
            return AckStatus.FAILED, AckReason.INTERNAL_ERROR

    def _rebuild(self):
        try:
            from fastlio_control import restart_and_calibrate_fastlio
            if self.stop_event.is_set():
                return
            if not self.navigation.running:
                self.navigation.start()
            result = restart_and_calibrate_fastlio(self.fc, self.navigation, timeout=45)
            if result.get('needs_calibration') is not False or not result.get('pose'):
                raise RuntimeError('new-map calibration did not complete')
            with self._condition:
                if not self.stop_event.is_set():
                    self.remap_result = result
                    self.remap_state = 'ready'
                    LOG.info('DRONE NEW LIO READY run_id=%d result=%s', self.run_id, result)
        except Exception as exc:
            with self._condition:
                self.remap_error = str(exc)
                self.remap_state = 'cancelled' if self.stop_event.is_set() else 'failed'
            LOG.exception('DRONE remap failed')

    def join_remap(self, timeout=2.0):
        worker = self._remap_worker
        if worker is not None:
            worker.join(timeout)
            return not worker.is_alive()
        return True

    def set_task(self, session, task):
        with self._condition:
            if self.stop_event.is_set():
                return AckStatus.REJECTED, AckReason.NOT_READY
            if self._task == task and self._session == session:
                return AckStatus.COMPLETED, AckReason.NONE
            if self._started:
                return AckStatus.REJECTED, AckReason.BUSY
            # A task ID cannot be reused with different contents in a session.
            if self._task is not None and self._task.task_id == task.task_id and self._session == session:
                return AckStatus.REJECTED, AckReason.BAD_PAYLOAD
            self._task, self._session = task, session
            LOG.info("Task saved: id=%d allocation=%s", task.task_id, task.allocation())
            return AckStatus.COMPLETED, AckReason.NONE

    def request_start(self, session, task_id):
        with self._condition:
            if (self.stop_event.is_set() or not self.allow_start or self._task is None
                    or self._session != session or self._task.task_id != task_id
                    or self._run_session != session or not self.close_done
                    or self.remap_state != 'ready'):
                return AckStatus.REJECTED, AckReason.NOT_READY
            try:
                if self._validate_plan is None:
                    raise RuntimeError('physical release plan validator missing')
                if self._validate_plan() is False:
                    LOG.warning('START allowed with payload release SKIPPED: installation unconfigured')
            except Exception:
                LOG.exception('Physical release plan is not ready')
                return AckStatus.REJECTED, AckReason.NOT_READY
            self._started = True
            self._condition.notify_all()
            return AckStatus.ACCEPTED, AckReason.NONE

    def stop(self):
        self.stop_event.set()
        with self._condition:
            self.remap_state = 'cancelled'
            self._condition.notify_all()
        # The task's cleanup may open payloads only after lock is confirmed.

    def wait_for_start(self, timeout=None):
        deadline = None if timeout is None else time.monotonic()+timeout
        with self._condition:
            while not self._started:
                if self.stop_event.is_set():
                    raise RuntimeError("car terminated the waiting task")
                remaining = None if deadline is None else deadline-time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise TimeoutError("no matching car start received")
                self._condition.wait(.1 if remaining is None else min(.1, remaining))
            if self.stop_event.is_set():
                raise RuntimeError("car terminated the task before launch")
            return self._task


class RescueFleetNode(AirFleetNode):
    def __init__(self, *args, receiver, **kwargs):
        self.receiver = receiver
        kwargs["controller_node"] = NodeId.CAR
        super().__init__(*args, **kwargs)

    def _handle(self, request):
        if request.dst != NodeId.DRONE:
            return None
        return super()._handle(request)

    def _command(self, request):
        command_id = request.payload[0] if request.payload else 0
        try:
            command = decode_command(request.payload)
            command_id = command.command_id
            if command.command_flags:
                raise ValueError("drop-task command flags must be zero")
            if command_id == CommandId.DRONE_SET_DROP_TASK:
                status, reason = self.receiver.set_task(request.session, decode_drop_task(command.command_body))
            elif command_id == CommandId.DRONE_START_DROP_TASK:
                status, reason = self.receiver.request_start(request.session, decode_drop_start(command.command_body))
            elif command_id in (CommandId.DRONE_PAYLOAD_CLOSE, CommandId.DRONE_REBUILD_MAP):
                if len(command.command_body) != 5:
                    raise ValueError('run body must be <BI>')
                version, run_id = struct.unpack('<BI', command.command_body)
                if version != 1 or run_id == 0:
                    raise ValueError('invalid run version/id')
                action = (self.receiver.close_payload if command_id == CommandId.DRONE_PAYLOAD_CLOSE
                          else self.receiver.rebuild_map)
                status, reason = action(request.session, run_id)
            elif command_id in (CommandId.TARGETED_STOP, CommandId.PING):
                if command.command_body:
                    raise ValueError("command body must be empty")
                if command_id == CommandId.TARGETED_STOP:
                    self.receiver.stop()
                    # ACK confirms delivery; landing belongs to the mission thread.
                    status, reason = AckStatus.ACCEPTED, AckReason.NONE
                else:
                    status, reason = AckStatus.COMPLETED, AckReason.NONE
            else:
                status, reason = AckStatus.REJECTED, AckReason.UNSUPPORTED
        except (ValueError, ProtocolError):
            status, reason = AckStatus.REJECTED, AckReason.BAD_PAYLOAD
        return self._ack(request, command_id, status, reason)


def attach_rescue_fleet_node(fc, navigation, stop_event, *, allow_start=False, payload=None,
                            validate_plan=None):
    receiver = RescueTaskReceiver(stop_event, allow_start=allow_start, payload=payload,
        fc=fc, navigation=navigation, validate_plan=validate_plan)
    node = attach_air_fleet_node(fc, navigation, stop_event, readonly=True,
        controller_node=NodeId.CAR,
        node_factory=lambda *args, **kwargs: RescueFleetNode(*args, receiver=receiver, **kwargs))
    return node, receiver

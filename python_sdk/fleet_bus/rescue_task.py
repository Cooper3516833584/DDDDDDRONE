"""Car-controlled task reception. RX never calls navigation or relay methods."""
import logging
import threading
import time

from .air_node import AirFleetNode, attach_air_fleet_node
from .drop_task_protocol import decode_drop_start, decode_drop_task
from .models import AckReason, AckStatus, CommandId, NodeId
from .protocol import decode_command, ProtocolError

LOG = logging.getLogger(__name__)


class RescueTaskReceiver:
    def __init__(self, stop_event, *, allow_start=False):
        self.stop_event = stop_event
        self.allow_start = allow_start
        self._condition = threading.Condition()
        self._task = None
        self._session = None
        self._started = False

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
                    or self._session != session or self._task.task_id != task_id):
                return AckStatus.REJECTED, AckReason.NOT_READY
            self._started = True
            self._condition.notify_all()
            return AckStatus.ACCEPTED, AckReason.NONE

    def stop(self):
        self.stop_event.set()
        with self._condition:
            self._condition.notify_all()

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


def attach_rescue_fleet_node(fc, navigation, stop_event, *, allow_start=False):
    receiver = RescueTaskReceiver(stop_event, allow_start=allow_start)
    node = attach_air_fleet_node(fc, navigation, stop_event, readonly=True,
        controller_node=NodeId.CAR,
        node_factory=lambda *args, **kwargs: RescueFleetNode(*args, receiver=receiver, **kwargs))
    return node, receiver

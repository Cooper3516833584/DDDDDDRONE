"""Thread-safe handoff from FleetBus reception to the existing flight task layer."""

import itertools
import queue
import threading
from typing import Optional

from .models import AirCommand, CommandId, CommandStatus


class AirCommandQueue:
    """Bounded priority queue; this class never invokes a flight-control method."""

    def __init__(self, max_items: int = 16) -> None:
        if max_items <= 0:
            raise ValueError("max_items must be positive")
        self._queue = queue.PriorityQueue(max_items)
        self._counter = itertools.count()
        self._status = CommandStatus()
        self._status_lock = threading.Lock()

    def put(self, command: AirCommand) -> bool:
        priority = 0 if command.command_id == int(CommandId.TARGETED_STOP) else 10
        try:
            self._queue.put_nowait((priority, next(self._counter), command))
        except queue.Full:
            return False
        return True

    def receive(self, timeout: Optional[float] = None) -> Optional[AirCommand]:
        try:
            _, _, command = self._queue.get(timeout=timeout)
        except queue.Empty:
            return None
        return command

    def accept(self, command: AirCommand) -> None:
        with self._status_lock:
            self._status = CommandStatus(command.ground_seq, 2, 0)

    def complete(self, command: AirCommand) -> None:
        with self._status_lock:
            self._status = CommandStatus(command.ground_seq, 4, 0)

    def fail(self, command: AirCommand, error_code: int) -> None:
        with self._status_lock:
            self._status = CommandStatus(command.ground_seq, 5, error_code)

    def status(self) -> CommandStatus:
        with self._status_lock:
            return self._status

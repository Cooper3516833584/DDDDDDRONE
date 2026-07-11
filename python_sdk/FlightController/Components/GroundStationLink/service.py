import os
import threading
from typing import Optional

from .link import GroundStationLink, MissionCommand
from .models import GroundLinkMode, LEDControl, MissionState, RejectReason


DEFAULT_KEY_ENV = "GROUND_STATION_HMAC_KEY_HEX"


class GroundStationConfigurationError(RuntimeError):
    pass


class AircraftGroundStation:
    """Task-facing facade for the aircraft ground-station link.

    This object deliberately exposes command queue and result methods instead of
    callbacks. The mission manager remains the only place that executes flight
    actions; the HC-14 receive thread only validates and queues commands.
    """

    def __init__(
        self,
        fc,
        key: bytes,
        stop_event: Optional[threading.Event] = None,
        **link_options
    ):
        if len(key) < 16:
            raise GroundStationConfigurationError(
                "ground-station HMAC key must contain at least 16 bytes"
            )
        self._stop_event = stop_event or threading.Event()
        self._link = GroundStationLink(
            fc=fc,
            key=key,
            stop_event=self._stop_event,
            **link_options
        )

    @classmethod
    def from_environment(
        cls,
        fc,
        stop_event: Optional[threading.Event] = None,
        key_env: str = DEFAULT_KEY_ENV,
        **link_options
    ) -> "AircraftGroundStation":
        encoded = os.environ.get(key_env, "").strip()
        if not encoded:
            raise GroundStationConfigurationError(
                "{} is not configured".format(key_env)
            )
        try:
            key = bytes.fromhex(encoded)
        except ValueError as exc:
            raise GroundStationConfigurationError(
                "{} must contain a hexadecimal key".format(key_env)
            ) from exc
        return cls(
            fc=fc,
            key=key,
            stop_event=stop_event,
            **link_options
        )

    @property
    def connected(self) -> bool:
        return self._link.connected

    @property
    def stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def start(self) -> None:
        self._link.start()

    def close(self) -> None:
        self._link.close()

    def receive_command(
        self, timeout: Optional[float] = None
    ) -> Optional[MissionCommand]:
        return self._link.get_command(timeout=timeout)

    def command_done(self) -> None:
        self._link.task_done()

    @property
    def mode(self) -> GroundLinkMode:
        return self._link.mode

    def set_mode(self, mode: GroundLinkMode) -> None:
        self._link.set_mode(mode)

    def enable_command_reception(self) -> None:
        self._link.enable_command_reception()

    def enable_telemetry_transmission(self) -> None:
        self._link.enable_telemetry_transmission()

    def accept(self, command: MissionCommand) -> None:
        self._link.accept(command)

    def reject(self, command: MissionCommand, reason: RejectReason) -> None:
        self._link.reject(command, reason)

    def complete(self, command: MissionCommand) -> None:
        self._link.complete(command)

    def fail(self, command: MissionCommand, reason: RejectReason) -> None:
        self._link.fail(command, reason)

    def prepare_new_mission(self) -> None:
        self._stop_event.clear()
        self._link.reset_stop_latch_for_new_mission()

    def send_status(
        self,
        state: MissionState,
        target1: Optional[int] = None,
        target2: Optional[int] = None,
        progress: int = 0,
        error_code: int = 0,
        message: str = "",
    ) -> bool:
        return self._link.report_mission_status(
            state=state,
            target1=target1,
            target2=target2,
            progress=progress,
            error_code=error_code,
            message=message,
        )

    def send_alarm(self, code: int, message: str) -> bool:
        return self._link.report_alarm(code, message)

    def send_led_control(self, control: LEDControl) -> bool:
        return self._link.report_led_control(control)

    def send_state(self) -> bool:
        return self._link.send_fc_state_now()

    def __enter__(self) -> "AircraftGroundStation":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

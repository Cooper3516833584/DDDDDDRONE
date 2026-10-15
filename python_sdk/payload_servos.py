"""Four SG90 user outputs. Writes confirm IPC delivery, never shaft position."""
import math
import threading
import logging

SERVO_CHANNELS = (0, 1, 2, 3)
OPEN_ANGLE = 0
CLOSE_ANGLE = 90
OPEN_PWM_PERCENT = 60.0
CLOSE_PWM_PERCENT = 80.0
SERVO_BODY_POSITIONS_CM = (None, None, None, None)
DROP_SERVO_SEQUENCE = (None, None, None, None)
PAYLOAD_COUNT = 4
LOG = logging.getLogger(__name__)


def drop_plan_ready():
    return (all(slot is not None for slot in DROP_SERVO_SEQUENCE) and
            all(position is not None for position in SERVO_BODY_POSITIONS_CM))


def slot_for_drop_attempt(drop_number):
    if isinstance(drop_number, bool) or not isinstance(drop_number, int) or not 1 <= drop_number <= PAYLOAD_COUNT:
        raise ValueError("drop number must be 1..4")
    slot = DROP_SERVO_SEQUENCE[drop_number - 1]
    if isinstance(slot, bool) or not isinstance(slot, int) or slot not in SERVO_CHANNELS:
        raise RuntimeError("physical release mapping is unknown for drop %d" % drop_number)
    # A binary open cannot release a second cargo from an already opened slot.
    if slot in DROP_SERVO_SEQUENCE[:drop_number - 1]:
        raise RuntimeError("reused binary servo cannot release another cargo; mechanical plan required")
    return slot


def payload_offset_body_cm(drop_number):
    position = SERVO_BODY_POSITIONS_CM[slot_for_drop_attempt(drop_number)]
    if (position is None or len(position) != 2 or
            not all(math.isfinite(value) for value in position)):
        raise RuntimeError("measured cargo release position is required")
    return tuple(position)


def validate_drop_plan():
    if not drop_plan_ready():
        LOG.warning('PAYLOAD SKIPPED: four-cargo release mapping/positions are unconfigured')
        return False
    if len(DROP_SERVO_SEQUENCE) != PAYLOAD_COUNT or len(SERVO_BODY_POSITIONS_CM) != PAYLOAD_COUNT:
        raise ValueError('four-cargo release plan must contain four entries')
    for number in range(1, PAYLOAD_COUNT + 1):
        payload_offset_body_cm(number)
    return True


class DronePayloadServos:
    def __init__(self, fc):
        self.fc = fc
        self._lock = threading.RLock()

    def _write(self, channels, percent):
        errors = []
        with self._lock:
            for channel in channels:
                try:
                    self.fc.set_PWM_output(channel, percent)
                except Exception as exc:
                    errors.append("channel %d: %s" % (channel, exc))
        if errors:
            raise RuntimeError("payload command failed: " + "; ".join(errors))

    def open_all(self):
        self._write(SERVO_CHANNELS, OPEN_PWM_PERCENT)

    def close_all(self):
        self._write(SERVO_CHANNELS, CLOSE_PWM_PERCENT)

    def open_slot(self, slot):
        if isinstance(slot, bool) or not isinstance(slot, int) or slot not in SERVO_CHANNELS:
            raise ValueError("servo slot must be 0..3")
        self._write((slot,), OPEN_PWM_PERCENT)

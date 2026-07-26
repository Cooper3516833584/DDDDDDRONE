"""Replay waypoint terrain-label decisions from a vision log.

This tool is offline-only: it does not import the flight-control package, open
the camera, or connect to any hardware.  It supports both waypoint log formats
used by ``2026_disaster_survey.py`` and
``2026_disaster_survey_waypoint.py``.

Examples:

    python3 testcode/test_waypoint_vision_replay.py \
        vision_for_simulation/ring_detection_20260726_133030.log

    python3 testcode/test_waypoint_vision_replay.py LOG \
        --distance-thresholds none,75,120,150,180 \
        --arrival-timeout 3.0 \
        --min-duration 1.0 \
        --delay 0.0
"""

import argparse
import math
import re
import statistics
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


EXPECTED_LABELS = (
    "river",
    "lake",
    "snow_mountain",
    "snow_mountain",
    "river",
    "field",
    "lake",
    "wildfire",
    "settlements",
    "field",
    "snow_mountain",
    "debris_flow",
    "settlements",
    "settlements",
    "field",
)
MIN_CONFIRM_FRAMES = 2
MIN_CONFIRM_RATIO = 0.60
TIMESTAMP_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

_TIMESTAMP = r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)"
_RING_LABEL_RE = re.compile(
    rf"^{_TIMESTAMP} RING frame=(\d+): "
    r"label=([a-z_]+) conf=([0-9.]+) "
    r"offset=\(x=([-0-9.]+), y=([-0-9.]+)\)px age=(\d+)ms$"
)
_RING_NONE_RE = re.compile(
    rf"^{_TIMESTAMP} RING frame=(\d+): \(none\) age=(\d+)ms$"
)
_ARRIVED_RE = re.compile(
    rf"^{_TIMESTAMP} WP (\d+) ARRIVED .* after_frame=(\d+)$"
)
_ENTER_RE = re.compile(
    rf"^{_TIMESTAMP} WP (\d+) ENTER zone .* after_frame=(\d+)$"
)
_LABEL_RE = re.compile(
    rf"^{_TIMESTAMP} WP (\d+) (?:LABEL|LABEL_TIMEOUT) "
    r".* consensus=([a-z_]+|None)$"
)
_EXIT_RE = re.compile(
    rf"^{_TIMESTAMP} WP (\d+) EXIT zone .* consensus=([a-z_]+|None)$"
)


@dataclass(frozen=True)
class Detection:
    timestamp: datetime
    frame_seq: int
    label: Optional[str]
    confidence: float
    distance_px: Optional[float]
    age_ms: int


@dataclass
class WaypointWindow:
    waypoint_index: int
    kind: str
    start: datetime
    after_frame: int
    end: Optional[datetime] = None
    recorded_label: Optional[str] = None


@dataclass(frozen=True)
class ReplayResult:
    label: Optional[str]
    sample_count: int
    decided_after_s: Optional[float]


def _parse_timestamp(value: str) -> datetime:
    return datetime.strptime(value, TIMESTAMP_FORMAT)


def _parse_label(value: str) -> Optional[str]:
    return None if value == "None" else value


def _percentile(values: Sequence[int], fraction: float) -> int:
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


def parse_log(
    path: Path,
) -> Tuple[List[Detection], Dict[int, WaypointWindow], int, int]:
    raw = path.read_bytes()
    nul_bytes = raw.count(b"\x00")
    text = raw.replace(b"\x00", b"").decode("utf-8", errors="replace")

    detections: List[Detection] = []
    windows: Dict[int, WaypointWindow] = {}
    unmatched_timestamped_lines = 0

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        match = _RING_LABEL_RE.match(line)
        if match:
            offset_x = float(match.group(5))
            offset_y = float(match.group(6))
            detections.append(
                Detection(
                    timestamp=_parse_timestamp(match.group(1)),
                    frame_seq=int(match.group(2)),
                    label=match.group(3),
                    confidence=float(match.group(4)),
                    distance_px=math.hypot(offset_x, offset_y),
                    age_ms=int(match.group(7)),
                )
            )
            continue

        match = _RING_NONE_RE.match(line)
        if match:
            detections.append(
                Detection(
                    timestamp=_parse_timestamp(match.group(1)),
                    frame_seq=int(match.group(2)),
                    label=None,
                    confidence=0.0,
                    distance_px=None,
                    age_ms=int(match.group(3)),
                )
            )
            continue

        match = _ARRIVED_RE.match(line)
        if match:
            waypoint_index = int(match.group(2))
            windows[waypoint_index] = WaypointWindow(
                waypoint_index=waypoint_index,
                kind="arrived",
                start=_parse_timestamp(match.group(1)),
                after_frame=int(match.group(3)),
            )
            continue

        match = _ENTER_RE.match(line)
        if match:
            waypoint_index = int(match.group(2))
            windows[waypoint_index] = WaypointWindow(
                waypoint_index=waypoint_index,
                kind="zone",
                start=_parse_timestamp(match.group(1)),
                after_frame=int(match.group(3)),
            )
            continue

        match = _LABEL_RE.match(line)
        if match:
            waypoint_index = int(match.group(2))
            window = windows.get(waypoint_index)
            if window is not None:
                window.end = _parse_timestamp(match.group(1))
                window.recorded_label = _parse_label(match.group(3))
            continue

        match = _EXIT_RE.match(line)
        if match:
            waypoint_index = int(match.group(2))
            window = windows.get(waypoint_index)
            if window is not None:
                window.end = _parse_timestamp(match.group(1))
                window.recorded_label = _parse_label(match.group(3))
            continue

        if re.match(r"^\d{4}-\d\d-\d\d ", line):
            unmatched_timestamped_lines += 1

    return detections, windows, nul_bytes, unmatched_timestamped_lines


def select_label(detections: Iterable[Detection]) -> Optional[str]:
    valid = [item for item in detections if item.label is not None]
    if len(valid) < MIN_CONFIRM_FRAMES:
        return None

    counts: Dict[str, int] = {}
    confidence_sums: Dict[str, float] = {}
    for detection in valid:
        label = str(detection.label)
        counts[label] = counts.get(label, 0) + 1
        confidence_sums[label] = (
            confidence_sums.get(label, 0.0) + detection.confidence
        )

    selected = max(
        counts,
        key=lambda label: (counts[label], confidence_sums[label]),
    )
    if counts[selected] / len(valid) < MIN_CONFIRM_RATIO:
        return None
    return selected


def replay_window(
    window: WaypointWindow,
    detections: Sequence[Detection],
    distance_threshold: Optional[float],
    arrival_timeout: float,
    min_duration: float,
    delay: float,
) -> ReplayResult:
    start = window.start + timedelta(seconds=max(0.0, delay))
    if window.kind == "arrived":
        end = window.start + timedelta(seconds=max(0.0, delay) + arrival_timeout)
    else:
        end = window.end

    if end is None:
        return ReplayResult(None, 0, None)

    samples: List[Detection] = []
    for detection in detections:
        if detection.frame_seq <= window.after_frame:
            continue
        if detection.timestamp <= start or detection.timestamp > end:
            continue
        if detection.label is None:
            continue
        if (
            distance_threshold is not None
            and (
                detection.distance_px is None
                or detection.distance_px >= distance_threshold
            )
        ):
            continue

        samples.append(detection)
        if window.kind == "arrived":
            selected = select_label(samples)
            decided_after_s = (
                detection.timestamp - window.start
            ).total_seconds()
            if selected is not None and decided_after_s >= min_duration:
                return ReplayResult(
                    selected,
                    len(samples),
                    decided_after_s,
                )

    return ReplayResult(
        select_label(samples),
        len(samples),
        None,
    )


def parse_distance_thresholds(value: str) -> List[Optional[float]]:
    thresholds: List[Optional[float]] = []
    for raw_token in value.split(","):
        token = raw_token.strip().lower()
        if token in ("none", "all", "off"):
            thresholds.append(None)
            continue
        threshold = float(token)
        if threshold <= 0:
            raise ValueError("distance thresholds must be positive")
        thresholds.append(threshold)
    if not thresholds:
        raise ValueError("at least one distance threshold is required")
    return thresholds


def threshold_name(value: Optional[float]) -> str:
    return "all" if value is None else "{:g}px".format(value)


def max_debris_run(
    detections: Sequence[Detection], threshold: Optional[float]
) -> int:
    longest = 0
    current = 0
    for detection in detections:
        candidate = (
            detection.label == "debris_flow"
            and detection.distance_px is not None
            and (
                threshold is None
                or detection.distance_px < threshold
            )
        )
        if candidate:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def print_report(
    path: Path,
    detections: Sequence[Detection],
    windows: Dict[int, WaypointWindow],
    thresholds: Sequence[Optional[float]],
    expected_labels: Sequence[str],
    arrival_timeout: float,
    min_duration: float,
    delay: float,
    nul_bytes: int,
    unmatched_timestamped_lines: int,
) -> None:
    print("Log: {}".format(path))
    print(
        "Detections: {}  Waypoints: {}  NUL bytes removed: {}  "
        "Unmatched timestamped lines: {}".format(
            len(detections),
            len(windows),
            nul_bytes,
            unmatched_timestamped_lines,
        )
    )

    ages = [item.age_ms for item in detections]
    if ages:
        print(
            "Frame age: median={}ms  p90={}ms  p95={}ms  max={}ms".format(
                round(statistics.median(ages)),
                _percentile(ages, 0.90),
                _percentile(ages, 0.95),
                max(ages),
            )
        )

    print(
        "Replay: arrival_timeout={:.2f}s min_duration={:.2f}s delay={:.2f}s "
        "min_frames={} min_ratio={:.2f}".format(
            arrival_timeout,
            min_duration,
            delay,
            MIN_CONFIRM_FRAMES,
            MIN_CONFIRM_RATIO,
        )
    )
    print()

    headers = ["WP", "expected", "recorded"] + [
        threshold_name(item) for item in thresholds
    ]
    print("\t".join(headers))

    summaries = defaultdict(lambda: {"correct": 0, "wrong": 0, "none": 0})
    for waypoint_index in sorted(windows):
        expected = (
            expected_labels[waypoint_index]
            if waypoint_index < len(expected_labels)
            else "?"
        )
        window = windows[waypoint_index]
        row = [
            str(waypoint_index),
            expected,
            window.recorded_label or "-",
        ]
        for threshold in thresholds:
            replayed = replay_window(
                window,
                detections,
                threshold,
                arrival_timeout,
                min_duration,
                delay,
            )
            label = replayed.label
            row.append(label or "-")
            summary = summaries[threshold_name(threshold)]
            if label is None:
                summary["none"] += 1
            elif label == expected:
                summary["correct"] += 1
            else:
                summary["wrong"] += 1
        print("\t".join(row))

    print()
    for threshold in thresholds:
        name = threshold_name(threshold)
        summary = summaries[name]
        print(
            "{}: correct={} wrong={} none={} max_consecutive_debris={}".format(
                name,
                summary["correct"],
                summary["wrong"],
                summary["none"],
                max_debris_run(detections, threshold),
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline replay of waypoint terrain-label decisions"
    )
    parser.add_argument("log", type=Path, help="ring_detection_*.log path")
    parser.add_argument(
        "--distance-thresholds",
        default="none,75,120,150,180",
        help="comma-separated pixel thresholds; use 'none' for no filtering",
    )
    parser.add_argument(
        "--arrival-timeout",
        type=float,
        default=3.0,
        help="replay window for ARRIVED logs in seconds (default: 3.0)",
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=1.0,
        help="wait this long before accepting consensus (default: 1.0)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="ignore detections for this many seconds after ARRIVED (default: 0)",
    )
    parser.add_argument(
        "--expected",
        default=",".join(EXPECTED_LABELS),
        help="comma-separated expected labels ordered by WP index",
    )
    args = parser.parse_args()

    if args.arrival_timeout <= 0:
        parser.error("--arrival-timeout must be positive")
    if args.min_duration < 0:
        parser.error("--min-duration must not be negative")
    if args.min_duration > args.arrival_timeout:
        parser.error("--min-duration must not exceed --arrival-timeout")
    if args.delay < 0:
        parser.error("--delay must not be negative")
    if not args.log.is_file():
        parser.error("log file does not exist: {}".format(args.log))

    try:
        thresholds = parse_distance_thresholds(args.distance_thresholds)
    except ValueError as exc:
        parser.error(str(exc))
    expected_labels = tuple(
        token.strip() for token in args.expected.split(",") if token.strip()
    )

    detections, windows, nul_bytes, unmatched = parse_log(args.log)
    if not detections:
        parser.error("no RING records found in log")
    if not windows:
        parser.error("no waypoint windows found in log")

    print_report(
        args.log,
        detections,
        windows,
        thresholds,
        expected_labels,
        args.arrival_timeout,
        args.min_duration,
        args.delay,
        nul_bytes,
        unmatched,
    )


if __name__ == "__main__":
    main()

"""Temporary live viewer for the downward-camera landing marker.

The script opens USB camera index 0 through ``landing_marker_offset``.
Press Q in the video window, or Ctrl+C in the terminal, to stop and release
the camera. It does not connect to the flight controller.
"""

import time as _time

import landing_marker_offset as _marker

_CAMERA_INDEX = 1
_FPS_REPORT_SECONDS = 1.0


def main() -> None:
    """Run the component's live debug renderer for one USB camera."""
    _marker._DEBUG = True
    offsets = _marker.track_landing_marker(_CAMERA_INDEX)
    interval_started_at = _time.perf_counter()
    interval_frames = 0
    try:
        for _offset in offsets:
            interval_frames += 1
            now = _time.perf_counter()
            elapsed = now - interval_started_at
            if elapsed < _FPS_REPORT_SECONDS:
                continue
            fps = interval_frames / elapsed
            _marker._DEBUG_FPS = fps
            print(f"Landing marker pipeline FPS: {fps:.1f}", flush=True)
            interval_started_at = now
            interval_frames = 0
    except KeyboardInterrupt:
        pass
    finally:
        offsets.close()
        _marker._DEBUG = False
        _marker._DEBUG_FPS = None


if __name__ == "__main__":
    main()

"""
下视标记偏移与飞控激光高度实时监视（不飞行）。

本脚本只做以下事情：
- 直连飞控串口，读取状态遥测中的 state.alt_add；
- 打开指定摄像机，调用 landing_marker_offset.track_landing_marker()；
- 按设定频率打印像素偏移和高度。

不会调用 unlock、take_off、land、set_flight_mode、set_digital_output、
send_realtime_control_data 或任何电机/PWM 接口。若飞控在运行期间报告已
解锁，脚本会退出，避免在飞行中继续占用直连串口。

运行前确认 server_ros.py 及其他 FC_Server 程序已关闭，避免抢占飞控串口。
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterator, Optional, Tuple


SDK_DIR = Path(__file__).resolve().parent.parent
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))

from FlightController import FC_Controller  # noqa: E402
from landing_marker_offset import track_landing_marker  # noqa: E402


DEFAULT_FC_PORT = "/dev/ttyACM0"
DEFAULT_CAMERA_INDEX = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实时打印 landing marker 像素偏移和飞控 alt_add"
    )
    parser.add_argument(
        "--fc-port",
        default=DEFAULT_FC_PORT,
        help="飞控串口，默认 /dev/ttyACM0",
    )
    parser.add_argument(
        "--fc-baud",
        type=int,
        default=500000,
        help="飞控串口波特率，默认 500000",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=DEFAULT_CAMERA_INDEX,
        help="下视相机索引，默认 0",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="最大打印频率 Hz，默认 10；设为 0 表示每帧打印",
    )
    return parser.parse_args()


def _format_offset(
    x_px: Optional[float],
    y_px: Optional[float],
) -> Tuple[str, str]:
    if x_px is None or y_px is None:
        return "--", "--"
    return "{:+7.2f}".format(x_px), "{:+7.2f}".format(y_px)


def monitor(
    fc: FC_Controller,
    offsets: Iterator[Tuple[Optional[float], Optional[float]]],
    rate: float,
) -> None:
    """消费视觉输出，并读取同一时刻最新的飞控高度遥测。"""
    interval = 1.0 / rate if rate > 0 else 0.0
    last_print_at = 0.0

    print("[TEST] Monitoring started. Press Ctrl+C to stop.")
    print("[TEST] x_px: forward positive; y_px: left positive; alt_add: cm")
    for x_px, y_px in offsets:
        now = time.monotonic()
        if interval and now - last_print_at < interval:
            continue
        last_print_at = now

        if not fc.state.is_fresh(0.5):
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print("{} | FC telemetry stale".format(timestamp))
            continue
        if fc.state.unlock.value:
            raise RuntimeError(
                "Flight controller became unlocked; stop monitor before flight"
            )

        x_text, y_text = _format_offset(x_px, y_px)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(
            "{} | marker_x_px={} | marker_y_px={} | alt_add={:6d} cm".format(
                timestamp,
                x_text,
                y_text,
                int(fc.state.alt_add.value),
            )
        )


def main() -> int:
    args = parse_args()
    if args.rate < 0:
        print("[TEST] --rate must be zero or greater")
        return 2
    if args.camera_index < 0:
        print("[TEST] --camera-index must be non-negative")
        return 2
    if args.fc_baud <= 0:
        print("[TEST] --fc-baud must be positive")
        return 2

    fc = FC_Controller()
    offsets = None
    try:
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
        )
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("Flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError(
                "Flight controller is already unlocked; refuse monitor startup"
            )

        print("[TEST] Flight controller connected; no flight command will be sent.")
        offsets = track_landing_marker(args.camera_index)
        monitor(fc, offsets, args.rate)
        return 0
    except KeyboardInterrupt:
        print("\n[TEST] Monitor stopped by user")
        return 130
    except Exception as exc:
        print("[TEST] Monitor failed: {}".format(exc))
        return 1
    finally:
        if offsets is not None:
            try:
                offsets.close()
            except Exception:
                pass
        try:
            fc.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

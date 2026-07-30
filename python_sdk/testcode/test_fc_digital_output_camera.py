"""
数字输出与相机单帧测试（不飞行）。

流程：
1. 直连飞控并确认遥测新鲜、飞控已锁桨；
2. set_digital_output(0, True)；
3. 等待终端输入 s；
4. 用索引 0 的摄像机拍摄一帧，保存到项目根目录；
5. 关闭本地串口连接，但不发送 set_digital_output(0, False)。

运行前确认 server_ros.py 及其他 FC_Server 程序已关闭，避免抢占飞控串口。
"""

import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


SDK_DIR = Path(__file__).resolve().parent.parent
PROJECT_ROOT = SDK_DIR.parent
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))

from FlightController import FC_Controller  # noqa: E402


FC_SERIAL_DEV = "/dev/ttyACM0"
CAMERA_INDEX = 0
START_COMMAND = "s"
WARMUP_FRAMES = 10


def wait_for_start_command() -> None:
    """等待操作者显式确认，避免连接后立即打开相机。"""
    print("[TEST] Digital output 0 is enabled.")
    while True:
        try:
            command = input(
                "[TEST] Enter '{}' to capture from camera {}: ".format(
                    START_COMMAND,
                    CAMERA_INDEX,
                )
            ).strip().lower()
        except EOFError as exc:
            raise RuntimeError("Terminal input closed before confirmation") from exc
        if command == START_COMMAND:
            return
        print("[TEST] Ignored input; enter '{}' to continue.".format(START_COMMAND))


def _open_camera(index: int) -> cv2.VideoCapture:
    """按平台优先顺序打开指定索引的相机。"""
    if sys.platform.startswith("linux"):
        backends = (cv2.CAP_V4L2, cv2.CAP_ANY)
    elif sys.platform.startswith("win"):
        backends = (cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)

    for backend in backends:
        capture = cv2.VideoCapture(index, backend)
        if capture.isOpened():
            return capture
        capture.release()
    raise RuntimeError("Unable to open camera index {}".format(index))


def capture_confirmation_photo() -> Path:
    """预热相机后拍摄一帧，保存到项目根目录。"""
    capture = _open_camera(CAMERA_INDEX)
    try:
        frame = None
        for _ in range(WARMUP_FRAMES):
            ok, candidate = capture.read()
            if ok and candidate is not None:
                frame = candidate
            time.sleep(0.05)
        if frame is None:
            raise RuntimeError("Camera did not return a valid frame")

        photo_path = PROJECT_ROOT / (
            "fc_digital_output_camera0_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".jpg"
        )
        if not cv2.imwrite(str(photo_path), frame):
            raise RuntimeError("Failed to write photo: {}".format(photo_path))
        return photo_path
    finally:
        capture.release()


def main() -> int:
    fc = FC_Controller()
    digital_output_enabled = False
    try:
        fc.start_listen_serial(serial_dev=FC_SERIAL_DEV, print_state=False)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight-controller connection timeout")
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("Flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError(
                "Flight controller is unlocked; refuse digital-output test"
            )

        fc.set_digital_output(0, True)
        digital_output_enabled = True
        print("[TEST] Digital output 0 enabled.")

        wait_for_start_command()
        photo_path = capture_confirmation_photo()
        print("[TEST] Photo saved: {}".format(photo_path))
        print("[TEST] Digital output 0 remains enabled by design.")
        return 0
    except KeyboardInterrupt:
        print("[TEST] Interrupted by user; digital output 0 is not changed.")
        return 130
    except Exception as exc:
        print("[TEST] Failed: {}".format(exc))
        if digital_output_enabled:
            print("[TEST] Digital output 0 is intentionally left enabled.")
        return 1
    finally:
        # 用户要求本测试不发送 set_digital_output(0, False)。
        try:
            fc.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

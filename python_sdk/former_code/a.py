import argparse
import os
import time
from typing import Optional, Tuple

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import PARAMS


os.chdir(os.path.dirname(os.path.abspath(__file__)))


def wait_first_pose(radar: LD_Radar, timeout_sec: float) -> Optional[Tuple[float, float, float]]:
    deadline = time.perf_counter() + timeout_sec
    while time.perf_counter() < deadline:
        if radar.rt_pose_update_event.wait(0.5):
            radar.rt_pose_update_event.clear()
            x, y, yaw = radar.rt_pose
            inited = getattr(radar, "_rt_pose_inited", [False, False, False])
            if all(inited):
                return float(x), float(y), float(yaw)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Radar-only localization test. No flight commands are sent."
    )
    # 【核心修改点 1】：把默认 source 改为 direct，默认雷达端口改为 /dev/ttyUSB0
    parser.add_argument("--source", choices=("fc", "direct"), default="direct", help="Radar source mode")
    parser.add_argument("--fc-port", default="/dev/ttyACM0", help="FC serial port for source=fc")
    parser.add_argument("--radar-port", default="/dev/ttyUSB0", help="Radar serial port for source=direct")
    parser.add_argument("--radar-type", default="LD06", choices=("LD06", "LD08"))
    parser.add_argument("--first-timeout", type=float, default=15.0, help="Seconds to wait for first pose")
    args = parser.parse_args()

    fc = None
    radar = LD_Radar()

    try:
        if args.source == "fc":
            fc = FC_Controller()
            fc.start_listen_serial(serial_dev=args.fc_port, print_state=False)
            fc.wait_for_connection()
            radar.start(fc, subtask_skip=1)
            logger.info(f"[TEST] Radar source: FC forwarding ({args.fc_port})")
        else:
            # 【核心修改点 2】：增加权限和连接状态的友好检查
            if not os.path.exists(args.radar_port):
                logger.error(f"找不到端口 {args.radar_port}！请检查雷达是否插好。")
                return
            if not os.access(args.radar_port, os.R_OK | os.W_OK):
                logger.error(f"没有读写权限！请先在终端执行: sudo chmod 666 {args.radar_port}")
                return

            radar.start(args.radar_port, args.radar_type, subtask_skip=1)
            logger.info(f"[TEST] Radar source: direct serial ({args.radar_port})")

        radar.start_resolve_pose(
            size=PARAMS.MAP_SIZE,
            scale_ratio=PARAMS.SCALE_RATIO,
            low_pass_ratio=PARAMS.LOW_PASS_RATIO,
            polyline=PARAMS.POLYLINE,
        )
        logger.info("[TEST] Radar pose solver started")

        first_pose = wait_first_pose(radar, timeout_sec=args.first_timeout)
        if first_pose is None:
            raise RuntimeError(
                "No radar pose solved within timeout. Check radar hardware/link/forwarding."
            )

        origin = np.array([first_pose[0], first_pose[1]], dtype=float)
        logger.info(
            f"[TEST] Origin set at current pose: x={origin[0]:.2f}cm y={origin[1]:.2f}cm yaw={first_pose[2]:.2f}"
        )
        logger.info("[TEST] Printing relative pose (x,y,yaw). Press Ctrl+C to exit.")

        while True:
            if radar.rt_pose_update_event.wait(1.0):
                radar.rt_pose_update_event.clear()
                x, y, yaw = map(float, radar.rt_pose)
                rel_x = x - origin[0]
                rel_y = y - origin[1]
                logger.info(f"[POSE] rel_x={rel_x:8.2f}cm rel_y={rel_y:8.2f}cm yaw={yaw:7.2f}")
            else:
                logger.warning("[POSE] No pose update in 1s")
    except KeyboardInterrupt:
        logger.info("[TEST] Stopped by user")
    finally:
        try:
            radar.stop_resolve_pose()
        except Exception:
            pass
        try:
            radar.stop()
        except Exception:
            pass
        if fc is not None:
            try:
                fc.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
"""使用平滑多航点轨迹执行一次真实飞行测试。

飞行流程：
1. 通过 FC_Client 连接已经运行的 FC_Server，不打开飞控串口；
2. 使用 fusion-ros 定位，在 (0, 0) 定点起飞到 120 cm；
3. 一次生成并跟随经过 (100, 0)、(100, -100)、(0, 0) 的平滑轨迹；
4. 在 (0, 0) 定点降落。

这是会解锁并驱动真实无人机的测试脚本。必须确认桨叶区域净空、定位稳定、
急停可用，并显式传入 ``--confirm-flight`` 才会执行。

默认假设 ROS 定位组件已经由 server_ros.py 或人工启动。只有确认不会干扰
现有 ROS 进程时，才可增加 ``--launch-ros`` 让脚本按 base_test.py 启动组件。

示例：
    cd python_sdk
    python3 testcode/test_smooth_trajectory_flight.py --confirm-flight
"""

import argparse
import os
import sys
import time
from typing import Optional

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Client  # noqa: E402
from FlightController.Components import LD_Radar  # noqa: E402
from FlightController.Components.RealSense import T265  # noqa: E402
from FlightController.Components.RosManager import RosManager  # noqa: E402
from FlightController.Components.RosMapper import RosMapper  # noqa: E402
from FlightController.Components.RosNode import RosNodeRunner  # noqa: E402
from FlightController.Solutions.Navigation import Navigation  # noqa: E402
from loguru import logger  # noqa: E402


CRUISE_HEIGHT = 120.0  # cm
NAVIGATION_SPEED = 22.0  # cm/s
VERTICAL_SPEED = 22.0  # cm/s
TAKEOFF_POINT = (0.0, 0.0)
TRAJECTORY_WAYPOINTS = (
    (100.0, 0.0),
    (100.0, -100.0),
    (0.0, 0.0),
)
POSITION_THRESHOLD = 10.0  # cm
POSE_READY_TIMEOUT = 20.0  # s


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过 FC_Client 执行 120cm 高度的平滑多航点真实飞行测试"
    )
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="确认现场满足真实飞行条件；未提供时不会连接或解锁",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="FC_Server 地址，默认 127.0.0.1",
    )
    parser.add_argument("--port", type=int, default=5654, help="FC_Server 端口")
    parser.add_argument("--authkey", default="fc", help="FC_Server 认证密钥")
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="连接 FC_Server 的超时时间 / s",
    )
    parser.add_argument(
        "--launch-ros",
        action="store_true",
        help="按 base_test.py 启动 ROS 定位组件；默认复用已经运行的组件",
    )
    return parser.parse_args()


def launch_ros_components() -> None:
    """按 base_test.py 的配置启动 ROS 定位组件。"""

    manager = RosManager()
    manager.chmod("/dev/ttyUSB0")
    manager.chmod("/dev/ttyACM0")
    manager.chmod("/dev/video1")
    manager.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    manager.launch_package("realsense2_camera", "rs_launch.py")
    manager.launch_package("cartographer_ros", "cartographer.launch.py")
    manager.run_package(
        "tf2_ros",
        "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link",
    )


def wait_for_navigation_pose(navi: Navigation, timeout: float) -> None:
    """等待 fusion-ros 导航输出新鲜位姿，超时则拒绝起飞。"""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if navi.pose_is_fresh():
            logger.info(
                "[TEST] Navigation pose ready: ({:.1f}, {:.1f}, {:.1f})".format(
                    navi.current_x,
                    navi.current_y,
                    navi.current_height,
                )
            )
            return
        time.sleep(0.1)
    raise RuntimeError("fusion-ros navigation pose was not ready before timeout")


def emergency_land(fc: FC_Client) -> None:
    """异常退出时请求降落；未确认落地前不强制锁桨。"""

    logger.warning("[TEST] Flight interrupted; requesting emergency landing")
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock(timeout_s=15):
        logger.error("[TEST] Landing lock was not confirmed; refusing forced lock")


def main() -> int:
    args = parse_args()
    if not args.confirm_flight:
        print("[!] 此脚本会执行真实飞行。确认现场安全后添加 --confirm-flight。")
        return 2
    if args.connect_timeout <= 0:
        print("[!] --connect-timeout 必须大于 0")
        return 2

    fc: Optional[FC_Client] = None
    radar: Optional[LD_Radar] = None
    t265: Optional[T265] = None
    navi: Optional[Navigation] = None
    ros_runner: Optional[RosNodeRunner] = None
    takeoff_attempted = False

    try:
        if args.launch_ros:
            logger.warning("[TEST] Launching ROS components as explicitly requested")
            launch_ros_components()

        fc = FC_Client()
        fc.connect(
            host=args.host,
            port=args.port,
            authkey=args.authkey.encode(),
            print_state=False,
            block=True,
            timeout=args.connect_timeout,
        )
        logger.info("[TEST] Connected to FC_Server through FC_Client")
        if not fc.state.update_event.wait(2.0):
            raise RuntimeError("fresh flight-controller telemetry was not received")
        fc.state.update_event.clear()
        if not fc.state.is_fresh(0.5):
            raise RuntimeError("flight-controller telemetry is stale")
        if fc.state.unlock.value:
            raise RuntimeError("flight controller is already unlocked; test will not take control")

        t265 = T265("ros")
        t265.start()
        radar = LD_Radar()
        radar.start("ros")
        mapper = RosMapper()
        navi = Navigation(fc=fc, rs=t265, radar=radar, mapper=mapper)

        ros_runner = RosNodeRunner()
        ros_runner.add_nodes().run()

        navi.set_navigation_speed(NAVIGATION_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)
        navi.start(mode="fusion-ros")
        navi.set_rs_speed_report(True, 2)
        wait_for_navigation_pose(navi, POSE_READY_TIMEOUT)

        logger.info(
            "[TEST] Pointing takeoff at {} to {:.0f}cm".format(
                TAKEOFF_POINT, CRUISE_HEIGHT
            )
        )
        takeoff_attempted = True
        navi.pointing_takeoff(TAKEOFF_POINT, target_height=CRUISE_HEIGHT)
        navi.set_yaw(0)
        if not navi.wait_for_yaw():
            raise RuntimeError("yaw stabilization was not confirmed")

        logger.info(
            "[TEST] Following smooth trajectory: {}".format(TRAJECTORY_WAYPOINTS)
        )
        trajectory_ok = navi.navigation_follow_waypoints(
            TRAJECTORY_WAYPOINTS,
            altitude=CRUISE_HEIGHT,
            wait=True,
            pos_thres=POSITION_THRESHOLD,
        )
        if not trajectory_ok:
            raise RuntimeError("smooth trajectory following failed")

        logger.info("[TEST] Trajectory complete; pointing landing at (0, 0)")
        if not navi.pointing_landing(TAKEOFF_POINT):
            raise RuntimeError("pointing landing was not confirmed")
        takeoff_attempted = False
        logger.info("[TEST] Smooth trajectory flight test completed")
        return 0
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[TEST] Smooth trajectory flight test failed")
        return 1
    finally:
        if navi is not None:
            try:
                navi.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")

        if (
            takeoff_attempted
            and fc is not None
            and fc.connected
            and fc.state.unlock.value
        ):
            try:
                emergency_land(fc)
            except Exception:
                logger.exception("[TEST] Emergency landing request failed")

        if t265 is not None:
            try:
                t265.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop T265")
        if ros_runner is not None:
            try:
                ros_runner.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop ROS node runner")
        if fc is not None:
            try:
                fc.close()
            except Exception:
                logger.exception("[TEST] Failed to close FC_Client")


if __name__ == "__main__":
    sys.exit(main())

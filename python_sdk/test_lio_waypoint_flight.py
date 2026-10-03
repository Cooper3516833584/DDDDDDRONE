"""MID360S 建图后的 1 m 高度逐点飞行测试。

在机载 Linux 上运行，复用已运行的 server_ros.py / FC_Server：
    cd python_sdk
    python3 test_lio_waypoint_flight.py --confirm-flight

流程：未解锁时重启 MID360S driver、FAST-LIO 和任务 DDS 上下文，等待
新地图有效位姿及静止原点校准，再定点起飞到 100 cm，依次飞往
(100, 100)、(200, 0)、(100, -100)、(0, 0)，在原点定点降落。
所有坐标、高度单位为 cm；原点为重启后飞机静止位置，X 前、Y 左。
重启会重新建图，不保留先前 FAST-LIO 地图。定点起飞复用现有函数，
先经过 30 cm 离地、70 cm 中间高度，再到 100 cm 巡航高度。

需要已复核的 mid360s_mount.json、ROS 环境和现有非交互服务重启权限。
运行前关闭其他使用定位 DDS 的任务、viewer 和 ROS 工具，并确认现场
飞行条件；无需关闭只持有飞控串口的 FC_Server。未传 --confirm-flight
时不连接飞控、不重启定位、不解锁。
"""

import argparse
import math
import sys
import threading
import time

from fastlio_control import require_ground_restart, restart_localization_for_task


CRUISE_HEIGHT = 100.0
NAVIGATION_SPEED = 22.0  # cm/s，与现有任务模板一致
VERTICAL_SPEED = 22.0  # cm/s
HOME = (0.0, 0.0)
WAYPOINTS = ((100.0, 100.0), (200.0, 0.0), (100.0, -100.0), HOME)


def positive_timeout(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("超时时间必须为有限正数")
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="MID360S 重启建图后 1 m 逐点飞行测试")
    parser.add_argument("--confirm-flight", action="store_true", help="确认执行真实飞行")
    parser.add_argument("--host", default="127.0.0.1", help="本机 FC_Server 地址")
    parser.add_argument("--port", type=int, default=5654)
    parser.add_argument("--authkey", default="fc")
    parser.add_argument("--connect-timeout", type=positive_timeout, default=5.0)
    parser.add_argument("--pose-timeout", type=positive_timeout, default=45.0,
                        help="重启完成后等待新地图静止校准的超时 / s")
    return parser.parse_args(argv)


class Mission:
    def __init__(self, fc, navi, stop_event):
        self.fc = fc
        self.navi = navi
        self.stop_event = stop_event
        self.takeoff_attempted = False

    def require_ready(self, ground=False):
        if self.stop_event.is_set():
            raise RuntimeError("Task stop requested")
        if not self.fc.connected or not self.fc.state.is_fresh(0.5):
            raise RuntimeError("Fresh connected FC telemetry is required")
        if not self.navi.pose_is_fresh():
            raise RuntimeError("Calibrated fresh LIO pose is required")
        if ground:
            require_ground_restart(self.fc, self.navi)
        elif (not self.fc.state.unlock.value or
              self.fc.state.mode.value != self.fc.HOLD_POS_MODE):
            raise RuntimeError("Unlocked HOLD_POS feedback is required for navigation")

    def prepare(self, timeout):
        self.navi.set_navigation_speed(NAVIGATION_SPEED)
        self.navi.set_vertical_speed(VERTICAL_SPEED)
        self.navi.start(mode="lio")  # 只启动监听；闭环控制标志保持关闭
        logger.info("[TEST] Restart MID360S localization and calibrate a new map")
        result = restart_localization_for_task(self.fc, self.navi, timeout=timeout)
        if result["needs_calibration"]:
            raise RuntimeError("New-map calibration was not confirmed")
        self.require_ready(ground=True)
        logger.info("[TEST] New-map localization ready; current position is home")

    def run(self):
        self.require_ready(ground=True)
        self.takeoff_attempted = True  # 起飞函数部分失败时也必须进入降落兜底
        self.navi.pointing_takeoff(HOME, target_height=CRUISE_HEIGHT)
        self.require_ready()
        # 现有起飞函数内部部分等待结果未上抛，任务层再次确认后才开始航线。
        if not self.navi.wait_for_height():
            raise RuntimeError("100 cm cruise height was not confirmed")
        if not self.navi.wait_for_waypoint():
            raise RuntimeError("Takeoff position hold was not confirmed")
        self.navi.set_yaw(0)
        if not self.navi.wait_for_yaw():
            raise RuntimeError("Yaw stabilization was not confirmed")

        for point in WAYPOINTS:
            self.require_ready()
            logger.info("[TEST] Navigate to {} cm at {} cm height", point, CRUISE_HEIGHT)
            if not self.navi.navigation_to_waypoint(point, wait=True):
                raise RuntimeError("Failed to reach waypoint {}".format(point))

        self.require_ready()
        logger.info("[TEST] Returned home; pointing landing")
        if not self.navi.pointing_landing(HOME):
            raise RuntimeError("Pointing landing was not confirmed")
        if not self.fc.wait_for_lock(timeout_s=4) or not self.fc.state.is_fresh(0.5):
            raise RuntimeError("Fresh landing lock feedback was not confirmed")
        self.takeoff_attempted = False
        logger.info("[TEST] Waypoint flight completed")


def emergency_land(fc):
    """停止水平运动后请求降落；未确认落地时不强制锁桨。"""
    logger.warning("[TEST] Flight interrupted; requesting emergency landing")
    fc.set_flight_mode(fc.PROGRAM_MODE)
    time.sleep(0.1)
    fc.stablize()
    fc.land()
    if not fc.wait_for_lock(timeout_s=20):
        logger.error("[TEST] Landing lock not confirmed; keep landing command active")
        fc.land()


def main(argv=None):
    global logger
    args = parse_args(argv)
    if not args.confirm_flight:
        print("此脚本会重启定位并执行真实飞行；确认现场条件后添加 --confirm-flight。")
        return 2
    if sys.platform != "linux":
        print("请在机载 Linux 上运行，使用本机 FC_Server 和定位服务。")
        return 2

    # 延迟硬件/ROS 导入，帮助信息和未确认模式无需这些依赖。
    from loguru import logger
    from FlightController import FC_Client
    from FlightController.Components.RosNode import RosNodeRunner
    from FlightController.Solutions.Navigation import Navigation

    fc = None
    navi = None
    ros_runner = None
    mission = None
    stop_event = threading.Event()
    try:
        fc = FC_Client()
        fc.connect(host=args.host, port=args.port, authkey=args.authkey.encode(),
                   print_state=False, block=True, timeout=args.connect_timeout)
        if not fc.state.update_event.wait(2.0) or not fc.state.is_fresh(0.5):
            raise RuntimeError("Fresh flight-controller telemetry was not received")
        if not fc.connected or fc.state.unlock.value:
            raise RuntimeError("Connected disarmed aircraft is required before preparation")
        navi = Navigation(fc=fc, stop_event=stop_event)
        ros_runner = RosNodeRunner()
        mission = Mission(fc, navi, stop_event)
        mission.prepare(args.pose_timeout)
        mission.run()
        return 0
    except KeyboardInterrupt:
        logger.warning("[TEST] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[TEST] LIO waypoint flight failed")
        return 1
    finally:
        stop_event.set()
        if navi is not None:
            try:
                navi.set_navigation_state(False)
                navi.set_keep_height_state(False)
                navi.stop()  # 先停导航并发送零速度，再交给飞控降落
            except Exception:
                logger.exception("[TEST] Failed to stop navigation")
        if (mission is not None and mission.takeoff_attempted and
                fc is not None and fc.connected and fc.state.unlock.value):
            try:
                emergency_land(fc)
            except Exception:
                logger.exception("[TEST] Emergency landing request failed")
        if ros_runner is not None:
            try:
                ros_runner.stop()
            except Exception:
                logger.exception("[TEST] Failed to stop task ROS node runner")
        if fc is not None:
            try:
                fc.close()
            except Exception:
                logger.exception("[TEST] Failed to close FC client")


if __name__ == "__main__":
    sys.exit(main())

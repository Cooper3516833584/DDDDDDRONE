"""
无人机基本功能测试代码

支持两种定位模式:
  - radar : 单雷达定位（检测雷达、飞控、摄像头）
  - ros   : ROS 融合定位（在单雷达基础上增加 T265 检测）

测试流程:
  1. 硬件连接检测（数量可自定义）
  2. 定点起飞至巡航高度 120cm
  3. 向前飞行 30cm
  4. 定高悬停 5 秒
  5. 返回起飞点
  6. 定点降落
"""

import os
import sys
import time
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
from loguru import logger

# ============================================================
# 可配置参数
# ============================================================

# 定位模式: "radar" 或 "ros"
POSITIONING_MODE = "radar"

# 各硬件期望连接数量
EXPECTED_RADAR_COUNT = 1        # 雷达数量
EXPECTED_FC_COUNT = 1           # 飞控数量
EXPECTED_CAMERA_COUNT = 1       # 摄像头数量
EXPECTED_T265_COUNT = 1          # T265 数量（仅 ROS 模式）

# 飞控串口设备（单雷达模式）
FC_SERIAL_DEV = "/dev/ttyACM0"

# ROS 模式下需要 chmod 的设备列表
ROS_CHMOD_DEVICES = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/video1"]

# 巡航高度 / cm
CRUISE_HEIGHT = 120

# 导航速度 / cm/s
NAVIGATION_SPEED = 22

# 垂直速度 / cm/s
VERTICAL_SPEED = 22

# 前飞距离 / cm
FORWARD_DISTANCE = 30

# 定高等待时间 / s
HOVER_WAIT_TIME = 5

# 起飞点坐标
TAKEOFF_POINT = (0.0, 0.0)


# ============================================================
# 硬件检测函数
# ============================================================

def check_radar(expected_count: int = 1) -> bool:
    """
    检测雷达连接。
    通过尝试创建 LD_Radar 实例并启动来验证。
    """
    from FlightController.Components import LD_Radar

    success_count = 0
    for i in range(expected_count):
        try:
            radar = LD_Radar()
            radar.start()
            time.sleep(1.0)
            if radar.running:
                logger.info(f"[CHECK] 雷达 #{i+1} 连接成功")
                success_count += 1
                radar.stop()
            else:
                logger.error(f"[CHECK] 雷达 #{i+1} 启动后未运行")
        except Exception as e:
            logger.error(f"[CHECK] 雷达 #{i+1} 连接失败: {e}")

    logger.info(f"[CHECK] 雷达检测结果: {success_count}/{expected_count} 通过")
    return success_count >= expected_count


def check_flight_controller_serial(expected_count: int = 1) -> bool:
    """
    检测飞控串口连接（单雷达模式）。
    """
    from FlightController import FC_Controller

    success_count = 0
    for i in range(expected_count):
        try:
            fc = FC_Controller()
            fc.start_listen_serial(serial_dev=FC_SERIAL_DEV, print_state=False)
            connected = fc.wait_for_connection(timeout_s=5)
            if connected:
                logger.info(f"[CHECK] 飞控 #{i+1} 连接成功 (串口: {FC_SERIAL_DEV})")
                success_count += 1
                fc.close()
            else:
                logger.error(f"[CHECK] 飞控 #{i+1} 连接超时")
        except Exception as e:
            logger.error(f"[CHECK] 飞控 #{i+1} 连接失败: {e}")

    logger.info(f"[CHECK] 飞控检测结果: {success_count}/{expected_count} 通过")
    return success_count >= expected_count


def check_flight_controller_client(expected_count: int = 1) -> bool:
    """
    检测飞控客户端连接（ROS 模式）。
    """
    from FlightController import FC_Client

    success_count = 0
    for i in range(expected_count):
        try:
            fc = FC_Client()
            fc.connect()
            time.sleep(0.5)
            # FC_Client 连接后没有显式的 connected 标志，
            # 若 connect() 不抛异常则认为成功
            logger.info(f"[CHECK] 飞控客户端 #{i+1} 连接成功")
            success_count += 1
            fc.close()
        except Exception as e:
            logger.error(f"[CHECK] 飞控客户端 #{i+1} 连接失败: {e}")

    logger.info(f"[CHECK] 飞控客户端检测结果: {success_count}/{expected_count} 通过")
    return success_count >= expected_count


def check_camera(expected_count: int = 1) -> Tuple[bool, Optional[cv2.VideoCapture]]:
    """
    检测摄像头连接，返回 (是否全部通过, 第一个可用摄像头实例)。
    """
    success_count = 0
    first_cap = None

    for i in range(expected_count):
        cap = None
        try:
            # 先尝试索引方式
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(i)
            if not cap.isOpened():
                # 尝试 /dev/video* 路径
                import glob
                devs = sorted(glob.glob("/dev/video*"))
                for dev in devs:
                    if cap is not None:
                        cap.release()
                    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                    if cap.isOpened():
                        break

            if cap is not None and cap.isOpened():
                # 预热摄像头
                for _ in range(5):
                    cap.read()
                    time.sleep(0.02)
                logger.info(f"[CHECK] 摄像头 #{i+1} 连接成功")
                success_count += 1
                if first_cap is None:
                    first_cap = cap
                elif i > 0:
                    cap.release()  # 只保留第一个
            else:
                logger.error(f"[CHECK] 摄像头 #{i+1} 连接失败: 无法打开")
        except Exception as e:
            logger.error(f"[CHECK] 摄像头 #{i+1} 连接失败: {e}")
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    logger.info(f"[CHECK] 摄像头检测结果: {success_count}/{expected_count} 通过")
    return success_count >= expected_count, first_cap


def check_t265(expected_count: int = 1) -> bool:
    """
    检测 T265 连接（仅 ROS 模式）。
    """
    from FlightController.Components.RealSense import T265

    success_count = 0
    for i in range(expected_count):
        try:
            t265 = T265("ros")
            t265.start()
            time.sleep(0.5)
            if t265.running:
                logger.info(f"[CHECK] T265 #{i+1} 连接成功 (ROS 模式)")
                success_count += 1
                t265.stop()
            else:
                logger.error(f"[CHECK] T265 #{i+1} 启动后未运行")
        except Exception as e:
            logger.error(f"[CHECK] T265 #{i+1} 连接失败: {e}")

    logger.info(f"[CHECK] T265 检测结果: {success_count}/{expected_count} 通过")
    return success_count >= expected_count


# ============================================================
# 任务类
# ============================================================

class TestMission:
    """基本功能测试任务"""

    def __init__(self, fc, radar, navi, t265=None):
        self.fc = fc
        self.radar = radar
        self.navi = navi
        self.t265 = t265
        self._emergency_stop = threading.Event()

    def stop(self):
        self._emergency_stop.set()
        try:
            self.navi.stop()
        except Exception:
            pass
        logger.info("[MISSION] 任务已停止")

    def run(self):
        fc = self.fc
        navi = self.navi

        logger.info("=" * 60)
        logger.info("[MISSION] 基本功能测试开始")
        logger.info(f"[MISSION] 巡航高度: {CRUISE_HEIGHT} cm")
        logger.info(f"[MISSION] 前飞距离: {FORWARD_DISTANCE} cm")
        logger.info(f"[MISSION] 悬停时间: {HOVER_WAIT_TIME} s")
        logger.info("=" * 60)

        # ---- 参数设置 ----
        navi.set_navigation_speed(NAVIGATION_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)

        # ---- 启动导航 ----
        navi.start()
        logger.info("[MISSION] 导航已启动")

        # 给传感器预热时间
        for _ in range(60):
            if self._emergency_stop.is_set():
                return
            time.sleep(0.1)

        # ---- 校准基准点 ----
        navi.calibrate_basepoint()
        logger.info(f"[MISSION] 基准点已校准: {navi.basepoint}")

        # ---- 定点起飞 ----
        fc.set_action_log(True)
        logger.info(f"[MISSION] 定点起飞 -> {TAKEOFF_POINT}, 目标高度 {CRUISE_HEIGHT} cm")
        navi.pointing_takeoff(TAKEOFF_POINT, CRUISE_HEIGHT)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)
        logger.info(f"[MISSION] 起飞完成, 当前高度约 {navi.current_height} cm")

        # ---- 向前飞行 30cm ----
        target_forward = (TAKEOFF_POINT[0] + FORWARD_DISTANCE, TAKEOFF_POINT[1])
        logger.info(f"[MISSION] 向前飞行 {FORWARD_DISTANCE} cm -> {target_forward}")
        navi.navigation_to_waypoint(target_forward, wait=True)
        logger.info("[MISSION] 已到达前飞目标点")

        # ---- 定高悬停 5 秒 ----
        logger.info(f"[MISSION] 定高悬停 {HOVER_WAIT_TIME} 秒")
        for i in range(HOVER_WAIT_TIME):
            if self._emergency_stop.is_set():
                return
            time.sleep(1)
            logger.info(f"[MISSION] 悬停中... {i+1}/{HOVER_WAIT_TIME}s, "
                        f"高度: {navi.current_height:.0f}cm, "
                        f"位置: ({navi.current_x:.1f}, {navi.current_y:.1f})")

        # ---- 返回起飞点 ----
        logger.info(f"[MISSION] 返回起飞点 {TAKEOFF_POINT}")
        navi.navigation_to_waypoint(TAKEOFF_POINT, wait=True)
        logger.info("[MISSION] 已到达起飞点")

        # ---- 定点降落 ----
        logger.info("[MISSION] 定点降落")
        navi.pointing_landing(TAKEOFF_POINT)
        logger.info("[MISSION] 降落完成")

        logger.info("=" * 60)
        logger.info("[MISSION] 基本功能测试完成!")
        logger.info("=" * 60)


# ============================================================
# 主入口
# ============================================================

def run_radar_mode():
    """单雷达定位模式"""
    from FlightController import FC_Controller
    from FlightController.Components import LD_Radar
    from FlightController.Solutions.Navigation import Navigation

    logger.info("=" * 60)
    logger.info("[MODE] 当前模式: 单雷达定位 (radar)")
    logger.info("=" * 60)

    # ---- 硬件检测 ----
    logger.info("[CHECK] 开始硬件连接检测...")

    all_ok = True

    # 检测雷达
    if not check_radar(EXPECTED_RADAR_COUNT):
        logger.error("[CHECK] 雷达检测未通过，终止任务")
        all_ok = False

    # 检测飞控
    if not check_flight_controller_serial(EXPECTED_FC_COUNT):
        logger.error("[CHECK] 飞控检测未通过，终止任务")
        all_ok = False

    # 检测摄像头
    cam_ok, cap = check_camera(EXPECTED_CAMERA_COUNT)
    if not cam_ok:
        logger.error("[CHECK] 摄像头检测未通过，终止任务")
        all_ok = False

    if not all_ok:
        logger.error("[CHECK] 硬件检测失败，请检查连接后重试")
        return

    logger.info("[CHECK] 所有硬件检测通过!")

    # ---- 初始化硬件 ----
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev=FC_SERIAL_DEV, print_state=False)
    fc.wait_for_connection()
    logger.info("[MANAGER] 飞控已连接")

    radar = LD_Radar()
    radar.start()
    time.sleep(0.5)
    logger.info("[MANAGER] 雷达已启动")

    navi = Navigation(fc=fc, radar=radar)

    mission = TestMission(fc=fc, radar=radar, navi=navi)

    # ---- 执行任务 ----
    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] 任务异常: {e}")
    finally:
        mission.stop()
        # 紧急降落保护
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] 自动降落")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                for _ in range(100):
                    if fc.state.alt_add.value < 10:
                        break
                    time.sleep(0.1)
                fc.lock()
        except Exception as e:
            logger.exception(f"[MANAGER] 自动降落失败: {e}")

        # 释放资源
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        fc.close()
        logger.info("[MANAGER] 资源已释放, 程序结束")


def run_ros_mode():
    """ROS 融合定位模式"""
    from FlightController import FC_Client
    from FlightController.Components import LD_Radar
    from FlightController.Components.RealSense import T265
    from FlightController.Components.RosMapper import RosMapper
    from FlightController.Components.RosNode import RosNodeRunner
    from FlightController.Components.RosManager import RosManager
    from FlightController.Solutions.Navigation import Navigation

    logger.info("=" * 60)
    logger.info("[MODE] 当前模式: ROS 融合定位 (ros)")
    logger.info("=" * 60)

    # ---- 硬件检测 ----
    logger.info("[CHECK] 开始硬件连接检测...")

    all_ok = True

    # 检测雷达
    if not check_radar(EXPECTED_RADAR_COUNT):
        logger.error("[CHECK] 雷达检测未通过，终止任务")
        all_ok = False

    # 检测飞控（ROS 模式用 FC_Client）
    if not check_flight_controller_client(EXPECTED_FC_COUNT):
        logger.error("[CHECK] 飞控客户端检测未通过，终止任务")
        all_ok = False

    # 检测摄像头
    cam_ok, cap = check_camera(EXPECTED_CAMERA_COUNT)
    if not cam_ok:
        logger.error("[CHECK] 摄像头检测未通过，终止任务")
        all_ok = False

    # 检测 T265
    if not check_t265(EXPECTED_T265_COUNT):
        logger.error("[CHECK] T265 检测未通过，终止任务")
        all_ok = False

    if not all_ok:
        logger.error("[CHECK] 硬件检测失败，请检查连接后重试")
        return

    logger.info("[CHECK] 所有硬件检测通过!")

    # ---- ROS 环境初始化 ----
    rm = RosManager()
    for dev in ROS_CHMOD_DEVICES:
        rm.chmod(dev)
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package("tf2_ros", "static_transform_publisher",
                   "0 0 0 0 0 0 camera_pose_frame base_link")
    logger.info("[MANAGER] ROS 环境初始化完成")

    # ---- 初始化硬件 ----
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    logger.info("[MANAGER] 飞控客户端已连接")

    t265 = T265("ros")
    t265.start()
    logger.info("[MANAGER] T265 已启动 (ROS)")

    radar = LD_Radar()
    radar.start("ros")
    logger.info("[MANAGER] 雷达已启动 (ROS)")

    mapper = RosMapper()
    RosNodeRunner().add_nodes().run()

    navi = Navigation(fc=fc, rs=t265, radar=radar, mapper=mapper)

    mission = TestMission(fc=fc, radar=radar, navi=navi, t265=t265)

    # ---- 执行任务 ----
    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] 任务异常: {e}")
    finally:
        mission.stop()
        # 紧急降落保护
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] 自动降落")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                for _ in range(100):
                    if fc.state.alt_add.value < 10:
                        break
                    time.sleep(0.1)
                fc.lock()
        except Exception as e:
            logger.exception(f"[MANAGER] 自动降落失败: {e}")

        # 释放资源
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        fc.close()
        logger.info("[MANAGER] 资源已释放, 程序结束")


def main():
    """主入口：根据 POSITIONING_MODE 选择对应模式"""

    logger.info("=" * 60)
    logger.info("无人机基本功能测试")
    logger.info(f"定位模式: {POSITIONING_MODE}")
    logger.info(f"雷达数量: {EXPECTED_RADAR_COUNT}, 飞控数量: {EXPECTED_FC_COUNT}, "
                f"摄像头数量: {EXPECTED_CAMERA_COUNT}" +
                (f", T265数量: {EXPECTED_T265_COUNT}" if POSITIONING_MODE == "ros" else ""))
    logger.info("=" * 60)

    if POSITIONING_MODE == "radar":
        run_radar_mode()
    elif POSITIONING_MODE == "ros":
        run_ros_mode()
    else:
        logger.error(f"[MANAGER] 未知定位模式: {POSITIONING_MODE}，可选值: radar, ros")
        sys.exit(1)


if __name__ == "__main__":
    main()

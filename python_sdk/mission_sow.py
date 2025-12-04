"""
使用雷达作为位置闭环的任务模板
"""
import threading
import time
from typing import List

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.Vision import *
from hmi import HMI
from loguru import logger


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = np.array([81.28021476, 418.3601995])

BASE_POINT = np.array([0, 0])

ENTRY_POINT = np.array([230, 0])

M_OFFSET = np.array([-16, 18])
CORNER_POINT = np.array([30, -359])
Y_BOX = np.array([0, 50])
X_BOX = np.array([50, 0])
m_point = lambda x, y: CORNER_POINT + X_BOX * x + Y_BOX * y + M_OFFSET
TARGET_POINTS = np.array(
    [
        m_point(4, 6),m_point(5, 6),m_point(5, 5),m_point(4, 5),m_point(4, 4),
        m_point(5, 4),m_point(5, 3),m_point(5, 2),m_point(5, 1),
        m_point(5, 0),m_point(4, 0),m_point(3, 0),m_point(3, 1),
        m_point(4, 1),m_point(4, 2),m_point(4, 3),m_point(3, 3),
        m_point(3, 2),m_point(2, 2),m_point(2, 3),m_point(1, 3),
        m_point(1, 2),m_point(1, 1),m_point(1, 0),m_point(0, 0),
        m_point(0, 1),m_point(0, 2),m_point(0, 3),
    ]
)  # fmt: skip

LANDING_POINT = np.array([0, 0])


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, hmi: HMI, rs: T265):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.hmi = hmi
        self.rs = rs
        self.inital_yaw = self.fc.state.yaw.value
        self.navi = Navigation(fc, radar, rs)

        self.cnt = 0

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        ############### 参数 #################
        self.camera_down_pwm = 67
        self.camera_up_pwm = 28
        self.navigation_speed = 30  # 导航速度
        self.precision_speed = 20  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.navi.set_navigation_speed(self.navigation_speed)
        self.navi.set_vertical_speed(self.vertical_speed)
        self.navi.start()  # 启动导航线程
        self.navi.switch_navigation_mode("fusion")
        logger.info("[MISSION] Navigation started")
        ################ 初始化 ################
        fc.set_action_log(False)
        change_cam_resolution(cam, 640, 480, 60)
        set_cam_autowb(cam, True)
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_digital_output(0, True)  # 激光笔
        fc.set_action_log(True)
        ################  校准 ################
        self.navi.set_basepoint(BASE_CALI_POINT)
        ################ 初始化完成 ################
        self.radar.debug = True
        logger.warning("Press Enter to takeoff")
        input()
        self.radar.debug = False
        logger.info("[MISSION] Mission-1 Started")
        self.navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        fc.set_digital_output(0, False)  # 激光笔
        ################ 开始任务 ################
        self.navi.navigation_to_waypoint(ENTRY_POINT)
        self.navi.wait_for_waypoint()
        for target_point in TARGET_POINTS:
            self.navi.navigation_to_waypoint(target_point)
            self.navi.wait_for_waypoint()
            self.sow()
        ################ 降落 ################
        logger.info("[MISSION] Go to landing point")
        self.navi.navigation_to_waypoint(LANDING_POINT)
        self.navi.wait_for_waypoint()
        self.navi.set_navigation_speed(self.precision_speed)
        self.navi.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson-1 Finished")

    def sow(self):  # 播撒
        if not self.check_green_ground():
            logger.info("[MISSION] No green ground, skipped")
            return
        logger.info("[MISSION] Green ground detected")
        self.fc.set_action_log(False)
        self.fc.set_digital_output(0, True)
        time.sleep(1.2)
        self.fc.set_digital_output(0, False)
        self.fc.set_action_log(True)
        logger.info("[MISSION] Sow Done")

    def check_green_ground(self):
        LOWER = np.array([35, 29, 126])
        UPPER = np.array([77, 255, 255])
        THRESHOLD = 0.5  # 颜色判断阈值
        ROI = (0.357, 0.41, 0.15, 0.1)  # 根据高度调整
        CHECK_NUM = 20  # 检测次数
        self.cnt += 1
        for _ in range(CHECK_NUM):
            img = self.cam.read()[1]
        img = self.cam.read()[1]
        img_roi = get_ROI(img, ROI)
        result = hsv_checker(img_roi, LOWER, UPPER, THRESHOLD)
        cv2.imwrite(f"test_img/{self.cnt}-img-{result}.jpg", img)
        cv2.imwrite(f"test_img/{self.cnt}-roi-{result}.jpg", img_roi)
        return result


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial("/dev/ttyS6", print_state=True)
    fc = FC_Client()
    fc.connect()
    fc.start_listen(False)
    fc.wait_for_connection()
    t265 = T265()
    t265.start()
    t265.hardware_reset()
    radar = LD_Radar()
    radar.start("/dev/ttyUSB0", "LD06")
    cam, i = open_camera()
    logger.info(f"Camera {i} opened")
    hmi = None

    mission = Mission(fc, radar, cam, hmi, t265)  # type: ignore
    logger.warning("Press Enter to start mission")
    input()
    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] Mission Failed")
    finally:
        mission.stop()
        if fc.state.unlock.value:
            logger.warning("[MANAGER] Auto Landing")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            ret = fc.wait_for_lock()
            if not ret:
                fc.lock()
    logger.info("[MANAGER] Mission finished")
    fc.close()

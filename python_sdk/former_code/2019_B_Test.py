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
from FlightController.Components import LD_Radar, Point_2D
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array("point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT = np.array([0, 0])
LANDING_POINT = np.array([0, 0])


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, rs: T265):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.rs = rs
        self.inital_yaw = self.fc.state.yaw.value
        self.navi = Navigation(fc, radar, rs)
        self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.camera_down_pwm = 67
        self.camera_up_pwm = 28
        self.navigation_speed = 30  # 导航速度
        self.precision_speed = 20  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion")
        logger.info("[MISSION] Navigation started")
        ################  校准 ################
        navi.set_basepoint(BASE_CALI_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        change_cam_resolution(cam, 640, 480, 60)
        set_cam_autowb(cam, True)
        fc.set_PWM_output(0, self.camera_up_pwm)
        fc.set_indicator_led(0, 255, 0)
        fc.event.key_short.clear()
        fc.event.key_short.wait_clear()
        fc.set_digital_output(0, True)  # 激光笔
        for _ in range(6):
            time.sleep(0.25)
            fc.set_digital_output(1, True)  # 蜂鸣器
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_digital_output(1, False)  # 蜂鸣器
            fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        ################ 开始任务 ################
        R = 60
        navi.navigation_to_waypoint([125, 0])
        navi.wait_for_waypoint()
        ##########################找杆##########################
        radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=50, to_=180 - 50)
        time.sleep(1)  # 等待雷达稳定
        point = radar.map_func_results[0][0]
        point.distance /= 10  # mm -> cm
        center_point = point.to_xy()
        
        now_point = navi.current_point
        logger.debug(f"[MISSION] Center point: {center_point}")
        center_point = now_point + center_point
        logger.debug(f"[MISSION] Abs center point: {center_point}")
        navi.navigation_to_waypoint(center_point + Point_2D(-90, R).to_xy())
        navi.wait_for_waypoint()
        logger.debug(f"[MISSION] Start circle")
        navi.keep_height_by_rs = True
        for deg in range(0, 360):
            time.sleep(20 / 360)
            waypoint = Point_2D(-90 - deg, R).to_xy() + center_point
            navi.navi_x_pid.setpoint = waypoint[0]
            navi.navi_y_pid.setpoint = waypoint[1]
        navi.keep_height_by_rs = False
        navi.wait_for_waypoint()
        ################ 降落 ################
        logger.info("[MISSION] Go to landing point")
        navi.navigation_to_waypoint(LANDING_POINT)
        navi.wait_for_waypoint()
        navi.set_navigation_speed(self.precision_speed)
        navi.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Misson-1 Finished")


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial(print_state=True)
    # fc.wait_for_connection()
    fc = FC_Client()
    fc.connect()
    t265 = T265()
    t265.start()
    t265.hardware_reset()
    radar = LD_Radar()
    radar.start(fc)
    cam, i = open_camera()
    logger.info(f"Camera {i} opened")

    mission = Mission(fc, radar, cam, t265)  # type: ignore
    # logger.warning("Press Enter to start mission")
    # input()
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
    fc.set_indicator_led(0, 255, 0)
    fc.set_digital_output(1, True)
    time.sleep(0.5)
    fc.set_digital_output(1, False)
    fc.set_indicator_led(0, 0, 0)
    fc.close()

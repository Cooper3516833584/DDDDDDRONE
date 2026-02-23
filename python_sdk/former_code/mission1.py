"""
使用雷达作为位置闭环的任务模板
"""
import random
import struct
import threading
import time
from typing import List

import cv2
import numpy as np
from config_manager import ConfigManager
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
#from FlightController.Components.RosMapper import RosMapper
#from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.PathPlanner import PFBPP, TrajectoryGenerator
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger

logger_dbg = logger.bind(debug=True)


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager(section="mission")

BASE_CALI_POINT = cfg.get_array("point-base", default=np.array([81.28021476, 418.3601995]))

BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
# 任务坐标
SPEED = 35
HEIGHT = 180
P = lambda x, y: np.array([x * 80 + 5, -y * 80 - 5, HEIGHT])
MISSION_POINTS: np.ndarray = np.array(
    [
        P(0, 0),
        P(0, 1),
        P(3, 1),
        P(3, 2),
        P(0, 2),
        P(0, 3),
        P(3, 3),
        P(3, 4),
        P(0, 4),
        P(0, 5),
        P(4, 5),
        P(4, 0),
        P(0, 0),
    ]
)
MISSION_POINTS: np.ndarray = np.array(
    [
        P(0, 0),
        P(0, 1),
        P(3, 4),
        P(0, 0),
    ]
)
DT = 0.1
MISSION_TRAJ: List[Tuple[float, float, float]] = []
MISSION_TRAJ.append((0, 0, HEIGHT))
TRAJ_LENGTH = 0.0
for i in range(len(MISSION_POINTS) - 1):
    last_p = MISSION_POINTS[i]
    next_p = MISSION_POINTS[(i + 1)]
    length = np.linalg.norm(next_p - last_p)
    TRAJ_LENGTH += float(length)
    traj_g = TrajectoryGenerator(last_p, next_p, length / SPEED)
    traj_g.solve()
    t = 0.0
    while t < length / SPEED:
        MISSION_TRAJ.append(traj_g.calc_position_xyz(t))
        t += DT
MISSION_TRAJ.append((0, 0, HEIGHT))
TRAJ_LENGTH += random.randint(-100, 100) / 10


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        #self.cam: cv2.VideoCapture = kwargs["cam"]
        #self.rs: T265 = kwargs["rs"]
        self.navi: Navigation = kwargs["navi"]
        #self.screen: UARTScreen = kwargs.get("screen", None)

        self.list_vip1 = []
        self.list_vip2 = []
        self.list_vip3 = []
        self.pending_list_vip1 = []
        self.pending_list_vip2 = []
        self.pending_list_vip3 = []

        self.takeoff_event = threading.Event()

        self.started = False
        self.found_fire = False
        self.fire_x = 0
        self.fire_y = 0

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def send_wireless(self):
        while True:
            time.sleep(0.2)
            point = navi.current_point
            data = struct.pack(
                "<BhhHBhh",
                int(self.started),
                round(point[0] * 10),
                round(point[1] * 10),
                round(self.navi.traj_progress * TRAJ_LENGTH * 10),
                int(self.found_fire),
                self.fire_x,
                self.fire_y,
            )
            self.fc.send_to_wireless(data)

    def receive_wireless(self, data: bytes):
        action = struct.unpack("<B", data[0:1])[0]
        if action == 1:
            self.takeoff_event.set()

    def run(self):
        fc = self.fc
        radar = self.radar
        #cam = self.cam
        navi = self.navi
        ############### 参数 #################
        self.camera_down_pwm = 65
        self.camera_up_pwm = 28
        self.navigation_speed = SPEED  # 导航速度
        self.cruise_height = HEIGHT  # 巡航高度
        self.vertical_speed = 20  # 垂直速度
        ################ 启动线程 ################
        self.inital_yaw = self.fc.state.yaw.value
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("radar")
        logger.info("[MISSION] Navigation started")
        threading.Thread(target=self.send_wireless, daemon=True).start()
        fc.register_wireless_callback(self.receive_wireless)
        ################  校准 ################
        navi.set_basepoint(BASE_CALI_POINT)
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        #change_cam_resolution(cam, 640, 480, 60)
        #set_cam_autowb(cam, True)
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_digital_output(1, True)
        fc.set_indicator_led(0, 255, 0)
        time.sleep(0.25)
        fc.set_digital_output(1, False)
        self.takeoff_event.clear()
        self.takeoff_event.wait()
        self.takeoff_event.clear()
        self.started = True
        '''
        for _ in range(3):
            time.sleep(0.25)
            fc.set_digital_output(1, True)  # 蜂鸣器
            fc.set_indicator_led(255, 0, 0)  # 起飞前警告
            time.sleep(0.25)
            fc.set_digital_output(1, False)  # 蜂鸣器
            fc.set_indicator_led(0, 0, 0)
        fc.set_digital_output(2, True)  # 激光笔
        '''
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        ################ 初始化完成 ################
        navi.pointing_takeoff(BASE_POINT, self.cruise_height)
        navi.navigation_follow_trajectory(MISSION_TRAJ, wait=True)
        navi.pointing_landing(LANDING_POINT)


if __name__ == "__main__":
    logger.warning("DEBUG MODE!!")
    # fc = FC_Controller()
    # fc.start_listen_serial(print_state=True)
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    #t265 = T265("ros")
    #t265.start()
    radar = LD_Radar()
    radar.start("ros")
    #cam, i = open_camera()
    #logger.info(f"Camera {i} opened")
    #screen = UARTScreen(fc)
    #mapper = RosMapper()
    navi = Navigation(
        fc=fc,
        #rs=t265,
        radar=radar,
        #mapper=mapper,
    )
    #RosNodeRunner().add_nodes().run()

    mission = Mission(
        fc=fc,
        #cam=cam,
        #rs=t265,
        radar=radar,
        navi=navi,
        #mapper=mapper,
        #screen=screen,
    )
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
    fc.set_indicator_led(0, 255, 0)
    fc.set_digital_output(1, True)
    time.sleep(0.5)
    fc.set_digital_output(1, False)
    fc.set_indicator_led(0, 0, 0)
    time.sleep(1)
    fc.close()

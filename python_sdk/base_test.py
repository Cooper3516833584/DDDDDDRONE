"""
使用ROS作为位置闭环的任务模板
"""
import threading
import time
import cv2
import numpy as np
# from config_manager import ConfigManager
from FlightController import FC_Client, FC_Like #FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Solutions.PathPlanner import PFBPP
from FlightController.Solutions.Vision import *
from loguru import logger
from FlightController.Components.RosManager import RosManager
from FlightController.Components.UartScreen import UARTScreen
import numpy as np
import struct
    
CURISE_SPEED = 22
CUREISE_HEIGHT = 120

class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.navi: Navigation = kwargs["navi"]
        self.rs: T265 = kwargs["rs"]
        # self.screen: UARTScreen = kwargs.get("screen", None)
       
    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def run(self):
        fc = self.fc
        radar = self.radar
        navi = self.navi
        ############### 参数 #################
        self.navigation_speed = CURISE_SPEED  # 导航速度
        self.cruise_height = CUREISE_HEIGHT  # 巡航高度
        self.vertical_speed = 22  # 垂直速度
        ################ 启动线程 ################
        navi.set_navigation_speed(self.navigation_speed)
        navi.set_vertical_speed(self.vertical_speed)
        navi.start()  # 启动导航线程
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started")
        navi.set_rs_speed_report(True, 2)
        ################ 初始化 ################
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        self.started = 1
        ################ Carto检测 #############
        while True:
            time.sleep(1)
            logger.info(f"current_point: {navi.current_point}")
            #     if navi.current_point[0] + navi.current_point[1] != 0:
            #         break
        # fc.set_indicator_led(0, 0, 0)
        # # 定点起飞
        # navi.pointing_takeoff((0,0),self.cruise_height)
        # navi.set_yaw(0)
        # navi.wait_for_yaw()
        # time.sleep(0.5)
        # # # 进入导航模式
        # navi.navigation_to_waypoint((100,0))
        # time.sleep(1)
        # navi.navigation_to_waypoint((100,-100))
        # time.sleep(1)
        # navi.navigation_to_waypoint((0,-100))
        # time.sleep(1)
        # navi.pointing_landing((0,0))
        






if __name__ == "__main__":
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")
    rm.chmod("/dev/ttyACM0")
    rm.chmod("/dev/video1")
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package("tf2_ros", "static_transform_publisher", "0 0 0 0 0 0 camera_pose_frame base_link")
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    screen = UARTScreen(fc)
    mapper = RosMapper()
    navi = Navigation(
        fc=fc,
        rs=t265,
        radar=radar,
        mapper=mapper,
    )
    RosNodeRunner().add_nodes().run()
    mission = Mission(
        fc=fc,
        rs=t265,
        radar=radar,
        navi=navi,
        mapper=mapper,
        screen=screen,
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
            # fc.set_digital_output(1, False)  # 激光笔
            ret = fc.wait_for_lock()
            if not ret:
                fc.lock()
    logger.info("[MANAGER] Mission finished")
    fc.close()

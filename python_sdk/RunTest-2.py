#!/usr/bin/env python3
"""
mission.py  ——  单文件双进程版本
主进程:Mission 逻辑
子进程:OpenCV 取流 + YOLO 推理，通过 mp.Queue 回传结果
"""
# ------------------ 0. 额外 import ------------------
import multiprocessing as mp
# ---------------------------------------------------

import threading
import time
from loguru import logger
from ultralytics import YOLO
import struct
import heapq
from collections import defaultdict
from typing import Tuple
# ---------------- 1. ROS / FC 相关 import ----------------
from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Vision import *

CURISE_SPEED = 20
CUREISE_HEIGHT = 120

class Mission(object):
    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")
    def __init__(self, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.navi: Navigation = kwargs["navi"]

    def run(self):
        fc, navi = self.fc, self.navi
        navi.set_navigation_speed(CURISE_SPEED)
        navi.set_vertical_speed(20)
        navi.start(mode="radar")
        navi.switch_navigation_mode("radar")
        logger.info("[MISSION] Navigation started")
        fc.set_indicator_led(0, 0, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        navi.calibrate_basepoint()
        navi.pointing_takeoff((0, 0), CUREISE_HEIGHT)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        # 悬停 3s
        time.sleep(3)
        # 定点降落
        navi.pointing_landing((0, 0))

if __name__ == "__main__":
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    radar = LD_Radar()
    radar.start()
    navi = Navigation(fc=fc, radar=radar)
    mission = Mission(fc=fc, radar=radar, navi=navi)
    try:
        mission.run()
    except Exception as e:
        logger.exception("[MANAGER] Mission Failed")
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
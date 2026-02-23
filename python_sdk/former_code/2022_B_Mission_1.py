'''
Mission_1:
直接飞到设定好的目的地送货
'''
#####################################################
import os,time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from FlightController import FC_Controller, FC_Client, FC_Server
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.PathPlanner import TrajectoryGenerator
from loguru import logger
from SolutionsNew.Vision_Net import FastestDetOnnx
import cv2

WAY_POINT_ARRAY = [
    (0, 0),      #基地点 point_0
    ###### 红色三角形  r_tri######
    (275, -50),  # point_1
    (125, -200), # point_2
    ###### 蓝色三角形  b_tri######
    (200, -275), # point_3
    (-25, -350), # point_4
    ###### 红色圆形  r_cir ######
    (275, -350), # point_5
    (50, -275),  # point_6
    ###### 蓝色圆形  b_cir ######
    (50, -125),  # point_7
    (200, -125), # point_8
    ###### 红色正方形  r_rec######
    (125, -50),  # point_9
    (-25, -200), # point_10
    ###### 蓝色正方形  b_rec######
    (275, -200), # point_11
    (125, -350), # point_12
]

def Deliver(waypoint_num):
    if waypoint_num == 'r_tri':
        i = 1
    if waypoint_num == 'b_tri':
        i = 3
    if waypoint_num == 'r_cir':
        i = 5
    if waypoint_num == 'b_cir':
        i = 7
    if waypoint_num == 'r_rec':
        i = 9
    if waypoint_num == 'b_rec':
        i = 11
    navi.navigation_to_waypoint(WAY_POINT_ARRAY[i])
    navi.set_height(DELIVER_HEIGHT)
    navi.wait_for_height()
    #navi.direct_set_waypoint(WAY_POINT_ARRAY[i])
    fc.set_PWM_output(0,70)
    time.sleep(5)
    fc.set_PWM_output(0,40)
    time.sleep(3.5)
    fc.set_PWM_output(0,50)
    navi.set_height(CRUISE_HEIGHT)
    navi.wait_for_height()
    navi.navigation_to_waypoint(WAY_POINT_ARRAY[i+1])
    navi.set_height(DELIVER_HEIGHT)
    navi.wait_for_height()
    #navi.direct_set_waypoint(WAY_POINT_ARRAY[i+1])
    fc.set_PWM_output(0,70)
    time.sleep(5)
    fc.set_PWM_output(0,40)
    time.sleep(3.5)
    fc.set_PWM_output(0,50)
    navi.set_height(CRUISE_HEIGHT)
    navi.wait_for_height()
    ...

fc = FC_Controller()
fc.start_listen_serial(serial_dev="/dev/ttyACM0")
fc.wait_for_connection()
radar = LD_Radar()
radar.start()
time.sleep(0.5)
navi = Navigation(fc=fc,radar=radar)
CRUISE_HEIGHT = 150 # 巡航高度
DELIVER_HEIGHT = 80 # 送货高度
LANDING_POINT: np.ndarray = np.array([0, 0])
# 速度
navi.set_navigation_speed(speed=25)
navi.set_vertical_speed(speed=20)
navi.start(mode="radar")  # 启动导航线程
navi.switch_navigation_mode("radar")
fc.set_action_log(True)
logger.info("[MISSION] Mission Start!")
try:
    ################校准##################
    navi.calibrate_basepoint()
    ################定点起飞##############
    navi.pointing_takeoff((0,0), CRUISE_HEIGHT)
    navi.set_yaw(0)
    navi.wait_for_yaw()
    time.sleep(1)
    ################导航至目标点送货##############
    ###### 送货标签，手动设置 #####
    waypoint_num = 'b_cir'
    Deliver(waypoint_num)
    ################返航降落##############
    navi.pointing_landing(LANDING_POINT)
except Exception as e:
    logger.exception(f"[MANAGER] Mission Failed")
finally:
    navi.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()


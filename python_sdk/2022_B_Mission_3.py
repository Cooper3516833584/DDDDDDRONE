'''
Mission_3:
钻圈
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

# 计算两点之间的中点
def midpoint(point1, point2):
    return ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)
# 计算斜率
def slope(point1, point2):
    return (point2[1] - point1[1]) / (point2[0] - point1[0])
# 计算垂直平分线的斜率
def perpendicular_slope(s):
    return -1 / s
# 计算中垂线的方程
def perpendicular_bisector(point1, point2):
    mid = midpoint(point1, point2)
    m = slope(point1, point2)
    if m == 0:
        return (None, mid[1])  # 若斜率为0，中垂线方程为y=mid[1]
    else:
        b = mid[1] - perpendicular_slope(m) * mid[0]
        return (perpendicular_slope(m), b)
# 在中垂线上选取两个点
def points_on_perpendicular_bisector(point1, point2, distance):
    mid = midpoint(point1, point2)
    m, b = perpendicular_bisector(point1, point2)
    if m is None:  # 如果中垂线垂直于y轴
        return [(mid[0] + distance, mid[1]), (mid[0] - distance, mid[1])]
    else:
        y1 = mid[1] + np.sqrt(distance ** 2 / (1 + m ** 2))
        y2 = mid[1] - np.sqrt(distance ** 2 / (1 + m ** 2))
        x1 = (y1 - b) / m
        x2 = (y2 - b) / m
        return [(x1, y1), (x2, y2)]
# 生成轨迹点列表
def generateTrajectory(point1, point2, distance):
    if point1[0] != point2[0] and point1[1] != point2[1]:
        points = points_on_perpendicular_bisector(point1, point2, distance)
        trajectory = [points[0], points[1]]  # 仅返回两个轨迹的初始两个点
        return trajectory
    if point1[0] == point2[0]:
        logger.warning("Enter 0")
        trajectory = [ ( point1[0]-distance/2, (point1[1]+point2[1])/2 ),( point1[0]+distance/2, (point1[1]+point2[1])/2 ) ]  # 仅返回两个轨迹的初始两个点
        return trajectory
    if point1[1] == point2[1]:
        logger.warning("Enter 1")
        trajectory = [ ( (point1[0]+point2[0])/2, point1[1]+distance/2 ),( (point1[0]+point2[0])/2, point1[1]-distance/2)]  # 仅返回两个轨迹的初始两个点
        return trajectory

fc = FC_Controller()
fc.start_listen_serial(serial_dev="/dev/ttyACM0")
fc.wait_for_connection()
radar = LD_Radar()
radar.start()
time.sleep(0.5)
navi = Navigation(fc=fc,radar=radar)
HEIGHT=110
LANDING_POINT: np.ndarray = np.array([0, 0])
# 速度
navi.set_navigation_speed(speed=25)
navi.set_vertical_speed(speed=20)
navi.start(mode="radar")  # 启动导航线程
navi.switch_navigation_mode("radar")
fc.set_action_log(True)
logger.info("[MISSION] Mission Started")

try:
    ################校准##################
    navi.calibrate_basepoint()
    ################定点起飞##############
    navi.pointing_takeoff((0,0), HEIGHT)
    navi.set_yaw(0)
    navi.wait_for_yaw()
    time.sleep(1)
    ################检测环的边界坐标########
    radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=0, to_=90,num=2)  
    time.sleep(5)
    logger.info("found two poles")
    point_1 = radar.map_func_results[0][0]
    point_1.distance /= 10  # mm -> cm
    xy_point_1 = point_1.to_xy()
    point_1_x = xy_point_1[0]
    point_1_y = xy_point_1[1]
    point_2 = radar.map_func_results[0][1]
    point_2.distance /= 10  # mm -> cm
    xy_point_2 = point_2.to_xy()
    point_2_x = xy_point_2[0]
    point_2_y = xy_point_2[1]
    logger.info(xy_point_1)
    logger.info(xy_point_2)
    now_point = navi.current_point
    now_point_x = now_point[0]
    now_point_y = now_point[1]
    #####################数学运算###################
    DISTANCE = 100
    TRAGECTORY = generateTrajectory((point_1_x,point_1_y),(point_2_x,point_2_y),DISTANCE)
    navi.navigation_to_waypoint(TRAGECTORY[0])
    navi.navigation_to_waypoint(TRAGECTORY[1])
    navi.navigation_to_waypoint(TRAGECTORY[0])
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



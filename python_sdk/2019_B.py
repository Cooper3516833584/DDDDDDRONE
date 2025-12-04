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

def check_yellow():
    cap = cv2.VideoCapture(0)
    deep = FastestDetOnnx(drawOutput=False)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    while True:
        # time.sleep(0.01)
        image = cap.read()[1]
        if image is None:
            continue
        # cv2.imshow("origin",image) # 调试时使用
        get = deep.detect(image)
        
        if len(get) > 0 :
            x = get[0][0][0]/width
            y = get[0][0][1]/height
            #logger.info(f"[Yellow] x: {x}, y: {y}")
            if x>0.4 and x<0.6 and y<0.6:
                navi.navigation_stop_here()
                logger.info(f"[MISSION] Yellow found!")
                time.sleep(1)
                cv2.imwrite("Yellow_1.jpg", image)
                time.sleep(0.1)
                cv2.imwrite("Yellow_2.jpg", image)
                time.sleep(0.1)
                cv2.imwrite("Yellow_3.jpg", image)
                logger.info("[MISSION] photo save, continue trajectory!")
                cap.release()
                navi.navigation_follow_trajectory(navi.traj_list_before_stop, wait=True)
                break
        if not navi.traj_running_event.is_set():
            logger.info("[MISSION] Trajectory finished, no yellow found")
            cap.release()
            break
        

################校准##################
navi.calibrate_basepoint()
################ 初始化 ##############
fc.set_action_log(True)
################初始化完成############
logger.info("[MISSION] Mission Started")
################定点起飞##############
navi.pointing_takeoff((0,0), HEIGHT)
navi.set_yaw(0)
navi.wait_for_yaw()
time.sleep(1)
################检测A杆B杆坐标########
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
# radar.unregister_map_func()
################数学转换########
now_point = navi.current_point
now_point_x = now_point[0]
now_point_y = now_point[1]
R = 77
WAY_POINT_A: np.ndarray = np.array([point_1_x - R, point_1_y])
WAY_POINT_B: np.ndarray = np.array([point_2_x - R, point_2_y])
################导航到A杆########
navi.navigation_to_waypoint(WAY_POINT_A)
logger.info("[MISSION] Reach A")
################导航到B杆,过程中巡线########
navi.set_navigation_speed(speed=10)
time.sleep(0.5)
navi.navigation_to_waypoint(WAY_POINT_B, wait=False)
check_yellow()
################ 在B杆处拍摄二维码 ################
logger.info("[MISSION] Reach B")
navi.set_navigation_speed(speed=25)
time.sleep(0.5)
cap = cv2.VideoCapture(0)
count = 0  # 用于计数保存图片的数量
while count < 3:  # 控制保存图片的数量为3张
    ret, image = cap.read()
    time.sleep(0.5)
    if image is None:
        continue
    # cv2.imshow("origin", image)  # 调试时使用
    # 保存图片
    filename = "QR_{}.jpg".format(count + 1)
    cv2.imwrite(filename, image)
    logger.info("[MISSION] Saved image {}".format(filename))
    count += 1
cap.release()
################ 绕B杆 ################
logger.info("[MISSION] Circle B")
WAY_POINT_B: np.ndarray = np.array([point_2_x, point_2_y])
navi.navigation_around_waypoint(WAY_POINT_B, degree=1*np.pi, mode="counterclockwise")
################ 导航到A杆 ################
WAY_POINT_A: np.ndarray = np.array([point_1_x + R, point_1_y])  
navi.navigation_to_waypoint(WAY_POINT_A,wait=True)
################ 绕A杆 ################
logger.info("[MISSION] Circle A")
WAY_POINT_A: np.ndarray = np.array([point_1_x, point_1_y])  
navi.navigation_around_waypoint(WAY_POINT_A, degree=1*np.pi, mode="counterclockwise")
################ 定点降落 ################
navi.pointing_landing(LANDING_POINT)


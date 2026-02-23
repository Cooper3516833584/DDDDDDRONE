import os,time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from FlightController import FC_Controller, FC_Client, FC_Server
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.PathPlanner import TrajectoryGenerator
from loguru import logger
from SolutionsNew.Vision_Net import FastestDet
import cv2

fc = FC_Controller()
fc.start_listen_serial(serial_dev="/dev/ttyACM0")
fc.wait_for_connection()
radar = LD_Radar()
radar.start()
time.sleep(0.5)
navi = Navigation(fc=fc,radar=radar)
SPEED=20
HEIGHT=100
BASE_POINT: np.ndarray = np.array([0, 0])
LANDING_POINT: np.ndarray = np.array([0, 0])
WAY_POINT: np.ndarray = np.array([0, 100])
# 速度
navi.set_navigation_speed(speed=25)
navi.set_vertical_speed(speed=20)
navi.start(mode="radar")  # 启动导航线程
navi.switch_navigation_mode("radar")

def check_yellow():
    cap = cv2.VideoCapture(0)
    deep = FastestDet(drawOutput=False)
    while True:
        time.sleep(0.01)
        image = cap.read()[1]
        if image is None:
            continue
        # cv2.imshow("origin",image) # 调试时使用
        get = deep.detect(image)
        if len(get) > 0 and get[0][0][0] > 310 and get[0][0][0] < 330 and get[0][0][1] > 230 and get[0][0][1] < 250:
            navi.navigation_stop_here()
            logger.info(f"[MISSION] Yellow found!")
            cv2.imwrite("Yellow_1.jpg", image)
            time.sleep(0.5)
            cv2.imwrite("Yellow_2.jpg", image)
            time.sleep(0.5)
            cv2.imwrite("Yellow_3.jpg", image)
            logger.info("[MISSION] photo save, continue trajectory!")
            navi.navigation_follow_trajectory(navi.traj_list_before_stop, wait=True)
            cap.release()
            cv2.destroyAllWindows()
            break
        if not navi.traj_running_event.is_set():
            logger.info("[MISSION] Trajectory finished, no fire found")
            cap.release()
            cv2.destroyAllWindows()
            break
        

################  校准 ################
navi.calibrate_basepoint()
################ 初始化 ################
fc.set_action_log(True)
################ 初始化完成 ################
logger.info("[MISSION] Mission Started")
################ 定点起飞 ################
navi.pointing_takeoff((0,0), HEIGHT)
navi.radar_find_target(TARGET_NUM)
time.sleep(2)
################ 绕A杆 ################
navi.navigation_around_waypoint(WAY_POINT, degree=1*np.pi, mode="clockwise")
time.sleep(1)
################ 导航到B杆 ################
B_POINT: np.ndarray = np.array([0, 100])
navi.navigation_to_waypoint(B_POINT, wait=False)
################ 检测条形码并拍摄 ################
check_yellow()
################ 拍摄二维码 ################
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
cv2.destroyAllWindows()
################ 绕B杆 ################
navi.navigation_around_waypoint(B_POINT, degree=1*np.pi, mode="counterclockwise")
################ 导航到A杆 ################
A_POINT: np.ndarray = np.array([0, 100])  
navi.navigation_to_waypoint(A_POINT,wait=True)
################ 绕A杆 ################
navi.navigation_around_waypoint(A_POINT, degree=1*np.pi, mode="counterclockwise")
################ 定点降落 ################
navi.pointing_landing(LANDING_POINT)


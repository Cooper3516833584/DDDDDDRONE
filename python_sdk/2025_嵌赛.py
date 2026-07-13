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
from ultralytics import YOLO
import struct
    
CURISE_SPEED = 22
CUREISE_HEIGHT = 120
MISSION_POINTS = [
    (-70,  -70), 
    (-160, -70),
    (-250, -70),
    (-340, -70),
    (-340, -160),
    (-340, -250),
    (-250, -250),
    (-250, -160),
    (-160, -160),
    (-160, -250),
    (-70,  -250),
    (-70,  -160),
]

class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.navi: Navigation = kwargs["navi"]
        self.rs: T265 = kwargs["rs"]
        self.screen: UARTScreen = kwargs.get("screen", None)
        # 识别部分
        self.target_dx   = None
        self.target_dy   = None
        self.target_name = None
        self.target_id   = None
        # 无线传输部分
        self.started = False
        self.cnt = 0 # 无人机位置计数
        self.location_array = [
            0,0,0,0,0,0,0,0,0,0,0,0
        ]
        # 水源位置
        self.lake_point = None
        # 火源位置
        self.wildfire_point = None
        # 泥石流位置
        self.debris_point = None
        # 山火任务
        self.fire_mission_flag = False

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def send_wireless(self):
        while True:
            time.sleep(0.2)
            data = struct.pack(
                "<BBBBBBBBBBBBB",
                int(self.started),
                int(self.location_array[0]),
                int(self.location_array[1]),
                int(self.location_array[2]),
                int(self.location_array[3]),
                int(self.location_array[4]),
                int(self.location_array[5]),
                int(self.location_array[6]),
                int(self.location_array[7]),
                int(self.location_array[8]),
                int(self.location_array[9]),
                int(self.location_array[10]),
                int(self.location_array[11])
            )
            self.fc.send_to_wireless(data)

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
        ################ 开启检测线程 ###########
        threading.Thread(target=self.check_target,  daemon=True).start()
        threading.Thread(target=self.send_wireless, daemon=True).start()
        ################ Carto检测 #############
        while True:
            time.sleep(1)
            logger.info(f"current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
        fc.set_indicator_led(0, 0, 0)
        # 定点起飞
        navi.pointing_takeoff((0,0),self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)
        # 具体任务部分
        for self.cnt in range(len(MISSION_POINTS)):
            navi.navigation_to_waypoint(MISSION_POINTS[self.cnt], wait=True)
            time.sleep(2)
            logger.info(f"第{self.cnt+1}块为{self.target_name}")
            if self.target_name != None:
                self.location_array[self.cnt] = self.target_id
            else:
                self.location_array[self.cnt] = 0
            if self.target_id == 6: # lake
                self.lake_point = navi.current_point
                self.wildfire_mission()
            elif self.target_id == 1: # fire
                fc.set_indicator_led(255,0,0)
                time.sleep(1)
                fc.set_indicator_led(0,0,0)
                self.wildfire_point = navi.current_point
                self.wildfire_mission()
            elif self.target_id == 4: # debris
                self.debris_point = navi.current_point
                self.debris_mission()
        ###############返航定点降落############
        logger.info(self.location_array)
        navi.pointing_landing((0,0))

    def wildfire_mission(self):
        fc = self.fc
        navi = self.navi
        # 如果在发现火源前有水源，或在发现火源后发现水源
        if self.lake_point is not None and self.wildfire_point is None:
            logger.info(f"found lake at {self.lake_point},but no wildfire!")
        if self.wildfire_point is not None and self.lake_point is None:
            logger.info(f"found wildfire at {self.wildfire_point},but no lake!")
        if self.lake_point is not None and self.wildfire_point is not None:
            # 降落取水
            navi.pointing_landing(self.lake_point)
            time.sleep(3)
            # 重新起飞
            fc.set_indicator_led(0,0,255) # 亮蓝灯代表取水完成
            navi.pointing_takeoff(self.lake_point,self.cruise_height)
            navi.navigation_to_waypoint(self.wildfire_point)
            navi.set_height(80)
            navi.wait_for_height()
            time.sleep(1.5)
            fc.set_indicator_led(0,0,0) # 熄蓝灯代表放水结束
            navi.set_height(self.cruise_height)
            navi.wait_for_height()
            time.sleep(1)
            self.wildfire_point = None # 此处山火已熄灭
            self.lake_point = None
            # self.fire_mission_flag = True
        # 如果前面没有水源，什么都不用做，继续走即可

    def debris_mission(self):
        fc = self.fc
        navi = self.navi
        navi.set_height(80)
        navi.wait_for_height()
        fc.set_digital_output(1, 0)
        fc.set_digital_output(2, 0)
        fc.set_digital_output(3, 0)
        time.sleep(1)
        navi.set_height(self.cruise_height)
        navi.wait_for_height()
        self.debris_point = None

    def check_target(self):
        model = YOLO('ultralytics/weights/best_int8_openvino_model', task='detect')
        results = model.predict(
            source=0,
            imgsz=320,
            stream=True,
            show=False,
            conf=0.7,
            verbose=False
        )
        # 提前把类别名称读出来，后面好映射 id → name
        names = model.names
        for result in results:
            time.sleep(0.01)
            # 初始化变量，用于存储最近的目标
            closest_target_dx = None
            closest_target_dy = None
            closest_target_name = None
            closest_target_id = None
            closest_distance = float('inf')  # 初始化为无穷大
            # 遍历当前帧里所有检测框
            for box in result.boxes:
                time.sleep(0.01)
                # 1. 坐标 (x1, y1, x2, y2) 是左上、右下两个角点
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                # 如果想拿中心点
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                # 2. 置信度
                conf = float(box.conf[0])
                # 3. 类别 id 和名称
                cls_id = int(box.cls[0])
                cls_name = names[cls_id]
                # 计算中心点与图像中心的距离
                dx = cx - 320
                dy = cy - 240
                distance = abs(dx) + abs(dy)  # 曼哈顿距离
                # 如果当前目标更靠近中心，更新最近目标
                if conf > 0.7 and distance < closest_distance:
                    closest_target_dx = dx
                    closest_target_dy = dy
                    closest_target_name = cls_name
                    closest_target_id = cls_id + 1
                    closest_distance = distance
            # 将最近目标的值赋给 self 变量
            self.target_dx = closest_target_dx
            self.target_dy = closest_target_dy
            self.target_name = closest_target_name
            self.target_id = closest_target_id
            if cv2.waitKey(1) == ord('q'):
                break
        cv2.destroyAllWindows()


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

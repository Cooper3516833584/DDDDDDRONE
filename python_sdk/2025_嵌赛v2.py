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
YAW_DIAGNOSTIC_INTERVAL = 0.5

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

    @staticmethod
    def _shortest_yaw_difference(target_yaw: float, current_yaw: float) -> float:
        """计算 [-180, 180) 范围内的有符号航向差。"""
        return (float(target_yaw) - float(current_yaw) + 180.0) % 360.0 - 180.0

    def _yaw_diagnostic_task(self, stop_event: threading.Event):
        """在转向与定点降落期间低频记录航向闭环的输入、输出和飞控回传。"""
        navi = self.navi
        fc = self.fc
        previous_time = None
        previous_navi_yaw = None
        previous_fc_yaw = None

        logger.info(
            f"[YAW-DIAG] logger started, interval={YAW_DIAGNOSTIC_INTERVAL:.2f}s"
        )
        while not stop_event.is_set():
            try:
                now = time.perf_counter()
                navi_yaw = float(navi.current_yaw)
                fc_yaw = float(fc.state.yaw.value)
                yaw_target = float(navi.yaw_target)

                if previous_time is None:
                    navi_yaw_rate = float("nan")
                    fc_yaw_rate = float("nan")
                else:
                    dt = max(now - previous_time, 1e-6)
                    navi_yaw_rate = self._shortest_yaw_difference(
                        navi_yaw, previous_navi_yaw
                    ) / dt
                    fc_yaw_rate = self._shortest_yaw_difference(
                        fc_yaw, previous_fc_yaw
                    ) / dt

                with navi._control_lock:
                    control_x, control_y, control_z, control_yaw = tuple(
                        navi._realtime_control_data_in_xyzYaw
                    )

                current_point = navi.current_point
                target_point = navi.navigation_target
                navi_error = self._shortest_yaw_difference(yaw_target, navi_yaw)
                fc_error = self._shortest_yaw_difference(yaw_target, fc_yaw)
                navi_fc_difference = self._shortest_yaw_difference(navi_yaw, fc_yaw)

                logger.info(
                    f"[YAW-DIAG] phase={self._yaw_diagnostic_phase} "
                    f"target={yaw_target:.2f}deg navi={navi_yaw:.2f}deg "
                    f"fc={fc_yaw:.2f}deg navi-fc={navi_fc_difference:+.2f}deg "
                    f"error(navi/fc)=({navi_error:+.2f}/{fc_error:+.2f})deg "
                    f"rate(navi/fc)=({navi_yaw_rate:+.2f}/{fc_yaw_rate:+.2f})deg/s "
                    f"control_body=({control_x},{control_y},{control_z})cm/s "
                    f"control_yaw={control_yaw:+.1f}deg/s(cw+) "
                    f"fc_attitude=({fc.state.rol.value:.2f},"
                    f"{fc.state.pit.value:.2f},{fc_yaw:.2f})deg "
                    f"fc_velocity=({fc.state.vel_x.value},{fc.state.vel_y.value},"
                    f"{fc.state.vel_z.value})cm/s "
                    f"map_pos=({current_point[0]:.1f},{current_point[1]:.1f})cm "
                    f"map_target=({target_point[0]:.1f},{target_point[1]:.1f})cm "
                    f"mode={fc.state.mode.value} unlock={fc.state.unlock.value} "
                    f"navigation={navi.navigation_flag} keep_height={navi.keep_height_flag} "
                    f"fc_command={fc.state.command_now}"
                )

                previous_time = now
                previous_navi_yaw = navi_yaw
                previous_fc_yaw = fc_yaw
            except Exception:
                logger.exception("[YAW-DIAG] failed to collect yaw diagnostics")

            stop_event.wait(YAW_DIAGNOSTIC_INTERVAL)

        logger.info("[YAW-DIAG] logger stopped")

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
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
        fc.set_indicator_led(0, 0, 0)
        # 定点起飞
        navi.pointing_takeoff((0,0),self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)
        # # 进入导航模式
        navi.navigation_to_waypoint((100,0))
        time.sleep(1)
        yaw_diagnostic_stop = threading.Event()
        self._yaw_diagnostic_phase = "turn_to_180"
        yaw_diagnostic_thread = threading.Thread(
            target=self._yaw_diagnostic_task,
            args=(yaw_diagnostic_stop,),
            name="yaw-diagnostic",
            daemon=True,
        )
        yaw_diagnostic_thread.start()
        try:
            navi.set_yaw(180)
            navi.wait_for_yaw()

            # navi.navigation_to_waypoint((100,-100))
            # time.sleep(1)
            # navi.navigation_to_waypoint((0,-100))
            # time.sleep(1)

            self._yaw_diagnostic_phase = "pointing_landing_to_origin"
            logger.info("[YAW-DIAG] starting pointing_landing((0, 0))")
            navi.pointing_landing((0,0))
            self._yaw_diagnostic_phase = "landing_complete"
            logger.info("[YAW-DIAG] pointing_landing((0, 0)) returned")
        finally:
            yaw_diagnostic_stop.set()
            yaw_diagnostic_thread.join(timeout=2.0)
            if yaw_diagnostic_thread.is_alive():
                logger.warning("[YAW-DIAG] logger did not stop within 2 seconds")
        






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

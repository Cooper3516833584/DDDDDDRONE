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
YAW_DIAGNOSTIC_INTERVAL = 0.1

PRECISE_YAW_LOOP_INTERVAL = 0.05

PRECISE_YAW_FAR_SPEED = 18
PRECISE_YAW_MEDIUM_SPEED = 12
PRECISE_YAW_NEAR_SPEED = 8
PRECISE_YAW_FINE_SPEED = 5
PRECISE_YAW_BRAKE_SPEED = 3
PRECISE_YAW_HOLD_SPEED = 8

PRECISE_YAW_FINAL_ERROR = 2.0
PRECISE_YAW_FINAL_RATE = 2.5
PRECISE_YAW_SETTLE_TIME = 1.0
PRECISE_YAW_TIMEOUT = 18.0
PRECISE_YAW_BRAKE_TRIGGER_ERROR = 6.0
PRECISE_YAW_BRAKE_SETTLE_TIME = 0.4
PRECISE_YAW_MAX_EXTRA_ROTATION = 25.0
PRECISE_YAW_STOP_DWELL = 1.0

PRECISE_YAW_RATE_FILTER_ALPHA = 0.30
PRECISE_YAW_MAX_VALID_RATE = 90.0


def _precise_yaw_speed_limit(
    abs_error: float,
    abs_yaw_rate: float,
    target_crossed: bool,
) -> int:
    """根据剩余误差、角速度和过目标状态返回 yaw PID 输出上限。"""
    if abs_error > 70.0:
        speed_limit = PRECISE_YAW_FAR_SPEED
    elif abs_error > 35.0:
        speed_limit = PRECISE_YAW_MEDIUM_SPEED
    elif abs_error > 15.0:
        speed_limit = PRECISE_YAW_NEAR_SPEED
    elif abs_error > 6.0:
        speed_limit = PRECISE_YAW_FINE_SPEED
    else:
        speed_limit = PRECISE_YAW_BRAKE_SPEED

    if abs_yaw_rate > 1.0:
        estimated_time_to_target = abs_error / abs_yaw_rate
        if estimated_time_to_target < 1.2:
            if abs_error > 6.0:
                speed_limit = min(speed_limit, PRECISE_YAW_FINE_SPEED)
            else:
                speed_limit = min(speed_limit, PRECISE_YAW_BRAKE_SPEED)

    if target_crossed and abs_yaw_rate > PRECISE_YAW_FINAL_RATE:
        speed_limit = min(speed_limit, PRECISE_YAW_BRAKE_SPEED)

    return int(speed_limit)


class Mission(object):
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.navi: Navigation = kwargs["navi"]
        self.rs: T265 = kwargs["rs"]
        self._yaw_diagnostic_phase = "idle"
        self._precise_yaw_speed_limit = None
        self._precise_yaw_filtered_rate = None
        self._precise_yaw_target_crossed = False
        self._precise_yaw_stable_time = 0.0
        self._precise_yaw_braking = False
        self._precise_yaw_accumulated_rotation = 0.0
        # self.screen: UARTScreen = kwargs.get("screen", None)
       
    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    @staticmethod
    def _navigation_yaw_error(target_yaw: float, current_yaw: float) -> float:
        """计算与 Navigation 相同语义的最短有符号航向误差。"""
        raw_error = float(target_yaw) - float(current_yaw)
        error = (raw_error + 180.0) % 360.0 - 180.0
        if error == -180.0 and raw_error > 0:
            return 180.0
        return error

    @staticmethod
    def _yaw_delta(current_yaw: float, previous_yaw: float) -> float:
        """计算相邻航向采样之间处理过环绕的变化量。"""
        return (float(current_yaw) - float(previous_yaw) + 180.0) % 360.0 - 180.0

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
                    navi_yaw_rate = self._yaw_delta(
                        navi_yaw, previous_navi_yaw
                    ) / dt
                    fc_yaw_rate = self._yaw_delta(
                        fc_yaw, previous_fc_yaw
                    ) / dt

                with navi._control_lock:
                    control_x, control_y, control_z, control_yaw = tuple(
                        navi._realtime_control_data_in_xyzYaw
                    )

                current_point = navi.current_point
                target_point = navi.navigation_target
                navi_error = self._navigation_yaw_error(yaw_target, navi_yaw)
                fc_error = self._navigation_yaw_error(yaw_target, fc_yaw)
                navi_fc_difference = self._yaw_delta(navi_yaw, fc_yaw)
                precise_rate = self._precise_yaw_filtered_rate
                precise_rate_text = (
                    "None" if precise_rate is None else f"{precise_rate:+.2f}"
                )

                logger.info(
                    f"[YAW-DIAG] phase={self._yaw_diagnostic_phase} "
                    f"target={yaw_target:.2f}deg navi={navi_yaw:.2f}deg "
                    f"fc={fc_yaw:.2f}deg navi-fc={navi_fc_difference:+.2f}deg "
                    f"error(navi/fc)=({navi_error:+.2f}/{fc_error:+.2f})deg "
                    f"rate(navi/fc)=({navi_yaw_rate:+.2f}/{fc_yaw_rate:+.2f})deg/s "
                    f"control_body=({control_x},{control_y},{control_z})cm/s "
                    f"control_yaw={control_yaw:+.1f}deg/s(cw+) "
                    f"precise_limit={self._precise_yaw_speed_limit} "
                    f"precise_rate={precise_rate_text}deg/s "
                    f"crossed={self._precise_yaw_target_crossed} "
                    f"stable={self._precise_yaw_stable_time:.2f}s "
                    f"braking={self._precise_yaw_braking} "
                    f"rotation={self._precise_yaw_accumulated_rotation:.1f}deg "
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

    def _hold_current_yaw(self, reason: str, braking: bool, dwell: float = 0.0) -> float:
        """将 Navigation 的 yaw 输出限制为零，并把目标锁在当前航向。"""
        self.navi.set_yaw_speed(0)
        current_yaw = float(self.navi.current_yaw)
        if np.isfinite(current_yaw):
            self.navi.set_yaw(current_yaw)
        else:
            logger.error(
                "[YAW-PRECISE] current yaw is invalid; "
                "yaw output remains clamped to zero"
            )
        self._precise_yaw_speed_limit = 0
        self._precise_yaw_braking = braking
        logger.warning(
            f"[YAW-PRECISE] yaw command stopped: reason={reason} "
            f"hold={current_yaw:.2f}deg dwell={dwell:.2f}s"
        )
        if dwell > 0:
            time.sleep(dwell)
        return current_yaw

    def turn_to_yaw_precise(
        self,
        target_yaw: float,
        timeout: float = PRECISE_YAW_TIMEOUT,
    ) -> bool:
        """使用 Navigation 现有 yaw 外环完成带动态减速的精确转向。"""
        navi = self.navi
        fc = self.fc
        target_yaw = float(target_yaw)

        start_time = time.perf_counter()
        previous_time = start_time
        previous_yaw = float(navi.current_yaw)
        initial_error = self._navigation_yaw_error(target_yaw, previous_yaw)

        initial_direction = 0
        if initial_error > 0:
            initial_direction = 1
        elif initial_error < 0:
            initial_direction = -1

        filtered_yaw_rate = 0.0
        stable_since = None
        brake_low_rate_since = None
        current_speed_limit = PRECISE_YAW_FAR_SPEED
        target_crossed = False
        braking = False
        fine_correction = False
        previous_error = initial_error
        accumulated_rotation = 0.0
        max_rotation = max(
            PRECISE_YAW_MAX_EXTRA_ROTATION,
            abs(initial_error) + PRECISE_YAW_MAX_EXTRA_ROTATION,
        )

        self._precise_yaw_speed_limit = current_speed_limit
        self._precise_yaw_filtered_rate = filtered_yaw_rate
        self._precise_yaw_target_crossed = target_crossed
        self._precise_yaw_stable_time = 0.0
        self._precise_yaw_braking = braking
        self._precise_yaw_accumulated_rotation = accumulated_rotation

        navi.set_yaw_speed(PRECISE_YAW_FAR_SPEED)
        navi.set_yaw(target_yaw)
        logger.info(
            f"[YAW-PRECISE] start target={target_yaw:.2f}deg "
            f"current={previous_yaw:.2f}deg "
            f"initial_error={initial_error:+.2f}deg "
            f"direction={initial_direction:+d}"
        )

        while True:
            time.sleep(PRECISE_YAW_LOOP_INTERVAL)
            now = time.perf_counter()
            current_yaw = float(navi.current_yaw)
            dt = max(now - previous_time, 1e-6)

            yaw_delta = self._yaw_delta(current_yaw, previous_yaw)
            raw_yaw_rate = yaw_delta / dt
            if abs(raw_yaw_rate) <= PRECISE_YAW_MAX_VALID_RATE:
                accumulated_rotation += abs(yaw_delta)
                filtered_yaw_rate = (
                    (1.0 - PRECISE_YAW_RATE_FILTER_ALPHA) * filtered_yaw_rate
                    + PRECISE_YAW_RATE_FILTER_ALPHA * raw_yaw_rate
                )
            else:
                logger.warning(
                    f"[YAW-PRECISE] ignored invalid yaw-rate sample: "
                    f"{raw_yaw_rate:+.2f}deg/s"
                )

            yaw_error = self._navigation_yaw_error(target_yaw, current_yaw)
            abs_error = abs(yaw_error)
            abs_yaw_rate = abs(filtered_yaw_rate)
            crossed_this_sample = (
                previous_error != 0
                and yaw_error != 0
                and previous_error * yaw_error < 0
            )

            if crossed_this_sample:
                target_crossed = True

            self._precise_yaw_filtered_rate = filtered_yaw_rate
            self._precise_yaw_target_crossed = target_crossed
            self._precise_yaw_accumulated_rotation = accumulated_rotation

            if not navi.running:
                logger.error("[YAW-PRECISE] navigation stopped during turn")
                self._hold_current_yaw(
                    "navigation-stopped", braking=False, dwell=PRECISE_YAW_STOP_DWELL
                )
                return False
            if not fc.state.unlock.value:
                logger.error("[YAW-PRECISE] aircraft locked during turn")
                self._hold_current_yaw(
                    "aircraft-locked", braking=False, dwell=PRECISE_YAW_STOP_DWELL
                )
                return False
            if fc.state.mode.value != fc.HOLD_POS_MODE:
                logger.error(
                    f"[YAW-PRECISE] unexpected flight mode: {fc.state.mode.value}"
                )
                self._hold_current_yaw(
                    "unexpected-flight-mode",
                    braking=False,
                    dwell=PRECISE_YAW_STOP_DWELL,
                )
                return False
            if not navi.navigation_flag:
                logger.error("[YAW-PRECISE] navigation flag disabled during turn")
                self._hold_current_yaw(
                    "navigation-disabled", braking=False, dwell=PRECISE_YAW_STOP_DWELL
                )
                return False

            if accumulated_rotation > max_rotation:
                logger.error(
                    f"[YAW-PRECISE] rotation guard triggered: "
                    f"rotation={accumulated_rotation:.1f}deg limit={max_rotation:.1f}deg"
                )
                self._hold_current_yaw(
                    "rotation-guard", braking=False, dwell=PRECISE_YAW_STOP_DWELL
                )
                return False

            if now - start_time >= timeout:
                self._hold_current_yaw(
                    "timeout", braking=False, dwell=PRECISE_YAW_STOP_DWELL
                )
                logger.warning(
                    f"[YAW-PRECISE] timeout target={target_yaw:.2f}deg "
                    f"current={current_yaw:.2f}deg "
                    f"error={yaw_error:+.2f}deg "
                    f"rate={filtered_yaw_rate:+.2f}deg/s "
                    f"crossed={target_crossed} "
                    f"elapsed={now-start_time:.2f}s"
                )
                return False

            should_brake = (
                crossed_this_sample
                or (
                    abs_error <= PRECISE_YAW_BRAKE_TRIGGER_ERROR
                    and abs_yaw_rate > PRECISE_YAW_FINAL_RATE
                )
            )
            if should_brake and not braking:
                braking = True
                fine_correction = True
                stable_since = None
                brake_low_rate_since = None
                current_speed_limit = 0
                self._hold_current_yaw("target-brake", braking=True)

            if braking:
                if abs_yaw_rate <= PRECISE_YAW_FINAL_RATE:
                    if brake_low_rate_since is None:
                        brake_low_rate_since = now

                    if abs_error <= PRECISE_YAW_FINAL_ERROR:
                        if stable_since is None:
                            stable_since = now
                        self._precise_yaw_stable_time = now - stable_since
                        if self._precise_yaw_stable_time >= PRECISE_YAW_SETTLE_TIME:
                            self._hold_current_yaw("target-reached", braking=False)
                            logger.info(
                                f"[YAW-PRECISE] reached target={target_yaw:.2f}deg "
                                f"current={current_yaw:.2f}deg "
                                f"error={yaw_error:+.2f}deg "
                                f"rate={filtered_yaw_rate:+.2f}deg/s "
                                f"elapsed={now-start_time:.2f}s"
                            )
                            return True
                    else:
                        stable_since = None
                        self._precise_yaw_stable_time = 0.0
                        if (
                            now - brake_low_rate_since
                            >= PRECISE_YAW_BRAKE_SETTLE_TIME
                        ):
                            braking = False
                            self._precise_yaw_braking = False
                            current_speed_limit = PRECISE_YAW_BRAKE_SPEED
                            navi.set_yaw_speed(current_speed_limit)
                            self._precise_yaw_speed_limit = current_speed_limit
                            navi.set_yaw(target_yaw)
                            logger.info(
                                f"[YAW-PRECISE] resume fine correction: "
                                f"error={yaw_error:+.2f}deg limit={current_speed_limit}deg/s"
                            )
                else:
                    brake_low_rate_since = None
                    stable_since = None
                    self._precise_yaw_stable_time = 0.0

                previous_yaw = current_yaw
                previous_time = now
                previous_error = yaw_error
                continue

            desired_speed_limit = _precise_yaw_speed_limit(
                abs_error=abs_error,
                abs_yaw_rate=abs_yaw_rate,
                target_crossed=target_crossed,
            )
            if fine_correction:
                desired_speed_limit = min(
                    desired_speed_limit, PRECISE_YAW_BRAKE_SPEED
                )
            if desired_speed_limit != current_speed_limit:
                navi.set_yaw_speed(desired_speed_limit)
                current_speed_limit = desired_speed_limit
                self._precise_yaw_speed_limit = current_speed_limit
                logger.info(
                    f"[YAW-PRECISE] speed-limit={current_speed_limit}deg/s "
                    f"error={yaw_error:+.2f}deg "
                    f"rate={filtered_yaw_rate:+.2f}deg/s "
                    f"crossed={target_crossed}"
                )

            if (
                abs_error <= PRECISE_YAW_FINAL_ERROR
                and abs_yaw_rate <= PRECISE_YAW_FINAL_RATE
            ):
                if stable_since is None:
                    stable_since = now
                self._precise_yaw_stable_time = now - stable_since
                if self._precise_yaw_stable_time >= PRECISE_YAW_SETTLE_TIME:
                    self._hold_current_yaw("target-reached", braking=False)
                    logger.info(
                        f"[YAW-PRECISE] reached target={target_yaw:.2f}deg "
                        f"current={current_yaw:.2f}deg "
                        f"error={yaw_error:+.2f}deg "
                        f"rate={filtered_yaw_rate:+.2f}deg/s "
                        f"elapsed={now-start_time:.2f}s"
                    )
                    return True
            else:
                stable_since = None
                self._precise_yaw_stable_time = 0.0

            previous_yaw = current_yaw
            previous_time = now
            previous_error = yaw_error

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
            turn_ok = self.turn_to_yaw_precise(180.0)
            if not turn_ok:
                logger.warning(
                    "[MISSION] Precise yaw turn did not fully settle; "
                    "continue returning to origin for safe landing"
                )

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

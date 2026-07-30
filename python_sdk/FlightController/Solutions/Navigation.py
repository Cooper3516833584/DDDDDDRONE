import threading
import time
from typing import Any, List, Literal, Optional, Tuple, Union

import numpy as np
from attr import dataclass
from FlightController import FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265, T265_Pose_Frame
from loguru import logger
from simple_pid import PID

from .PathPlanner import PFBPP, TrajectoryGenerator
from .SmoothTrajectory import SplineTrajectoryConfig, SplineTrajectoryGenerator

logger_dbg = logger.bind(debug=True)

YAW_AMBIGUITY_BAND_DEG = 2.0
POSE_STALE_TIMEOUT = 0.30
FUSION_ROS_CALIBRATION_INTERVAL = 1.0
FUSION_ROS_MAX_POSITION_CORRECTION_CM = 2.0
FUSION_ROS_MAX_YAW_CORRECTION_DEG = 1.0
NAVIGATION_CONTROL_STALE_TIMEOUT = 0.30
VELOCITY_OVERRIDE_ZERO_FLUSH_FRAMES = 3
VELOCITY_OVERRIDE_ZERO_FLUSH_INTERVAL = 0.05
MOVING_DESCENT_MAX_HORIZONTAL_SPEED = 50.0
MOVING_DESCENT_MAX_VERTICAL_SPEED = 30.0
TOUCHDOWN_MAX_VERTICAL_SPEED_THRESHOLD = 10.0

def _shortest_yaw_error(
    target_yaw: float,
    current_yaw: float,
    preferred_direction: int = 0,
) -> float:
    """Return the signed shortest yaw error in degrees."""
    raw_error = float(target_yaw) - float(current_yaw)
    error = (raw_error + 180.0) % 360.0 - 180.0
    # Around exactly 180 degrees both directions are equally short.  Keep the
    # direction selected when set_yaw() was called so sensor noise cannot flip
    # +180/-180 on successive frames.
    if preferred_direction and abs(abs(error) - 180.0) <= YAW_AMBIGUITY_BAND_DEG:
        return abs(error) * (1 if preferred_direction > 0 else -1)
    if error == -180.0 and raw_error > 0:
        return 180.0
    return error


def _world_to_body_velocity(vel_x: float, vel_y: float, yaw: float) -> Tuple[float, float]:
    """Convert map-frame velocity to forward/left body-frame velocity."""
    yaw_rad = np.deg2rad(float(yaw))
    cos_yaw = float(np.cos(yaw_rad))
    sin_yaw = float(np.sin(yaw_rad))
    body_x = cos_yaw * float(vel_x) - sin_yaw * float(vel_y)
    body_y = sin_yaw * float(vel_x) + cos_yaw * float(vel_y)
    return body_x, body_y


class PARAMS:
    ######## 解算参数 ########
    MAP_SIZE = 1000  # 雷达扫网定位图像大小
    POLYLINE = False  # 雷达扫网定位图像是否导出线框图
    SCALE_RATIO = 0.7  # 雷达扫网定位缩放比例
    LOW_PASS_RATIO = 0.6  # 雷达扫网定位低通滤波系数
    ######## 频率除数 影响PID更新频率 ########
    RADAR_SKIP = 8  # 雷达更新事件频率除数 python管理雷达:400/PACKLEN/RADAR_SKIP ROS管理雷达:1/RS_SKIP
    RS_SKIP = 3  # T265更新事件频率除数 200/RS_SKIP
    MAP_SKIP = 1  # ROS建图更新事件频率除数 5/MAP_SKIP
    FUSION_SKIP = 2  # 雷达融合T265频率除数 python管理雷达:400/PACKLEN/RADAR_SKIP/RS_SKIP/FUSION_SKIP ROS不使用该参数


class Navigation(object):
    """
    闭环导航, 使用realsense T265作为位置闭环, 使用雷达SLAM作为定位校准
    """

    def __init__(self, *args, **kwargs):
        """
        Args:
            fc: 飞控实例(必须) (FC_Controller, FC_Client, FC_Server)
            radar: 雷达实例(必须) (LD_Radar)
            rs: realsense实例(必须) (T265)(这里因为2024/3/5时还没有t265硬件,将初始化的t265默认为None)
            mapper: ROS地图模块实例(可选) (RosMapper)
        """
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        
        # 此处为没有t265的修改
        #self.rs: T265 = kwargs["rs"]
        self.rs: T265 = kwargs.get("rs",None)

        if "mapper" in kwargs:
            from FlightController.Components.RosMapper import RosMapper

            # 解耦:按需导入ROS相关的模块,防止非ROS环境下无法运行
            self.mapper: Optional[RosMapper] = kwargs["mapper"]
        else:
            self.mapper = None
        ############### PID #################
        self.navi_speed = 40  # 导航速度 / cm/s
        self.pid_tunings = {  # PID参数 (仅导航XY使用)
            "default": (0.35, 0.0, 0.08),  # 默认
            "navi": (1.4, 0.0, 0.02),  # 导航
            "hover": (0.65, 0.0, 0.02),  # 悬停
            "land": (0.85, 0.0, 0.02),  # 降落
        }
        self.height_pid = PID(0.8, 0.0, 0.1, setpoint=0, output_limits=(-30, 30), auto_mode=False)
        self.navi_x_pid = PID(
            *self.pid_tunings["default"],
            setpoint=0,
            output_limits=(-self.navi_speed, self.navi_speed),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            *self.pid_tunings["default"],
            setpoint=0,
            output_limits=(-self.navi_speed, self.navi_speed),
            auto_mode=False,
        )
        self.yaw_pid = PID(0.7, 0.0, 0.05, setpoint=0, output_limits=(-30, 30), auto_mode=False)
        self.yaw_target = 0.0
        self._yaw_direction_hint = 0
        #####################################
        self.current_x = 0  # 当前位置X(相对于基地点) / cm
        self.current_y = 0  # 当前位置Y(相对于基地点) / cm
        self.current_yaw = 0  # 当前偏航角(顺时针为正) / deg
        self.current_height = 0  # 当前高度(激光高度) / cm
        self.current_height_rs = 0.0  # 当前高度(realsense高度) / cm
        self.basepoint: Any = np.array([0.0, 0.0])  # 基地点(雷达坐标系)(Note:仅用于雷达扫网定位,建图则不需要) / cm
        #####################################
        self.keep_height_flag = False  # 定高状态
        self.navigation_flag = False  # 导航状态
        self.keep_height_by_rs = False  # 使用realsense定高
        self.stop_event = kwargs.get("stop_event")
        self.running = False
        self._control_lock = threading.Lock()
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self._navigation_control_updated_at = 0.0
        self._velocity_override_active = False
        self._velocity_override_keep_height = False
        self._velocity_override_updated_at = 0.0
        self._velocity_override_faulted = False
        self._velocity_override_horizontal_cancelled = False
        self._last_pose_update = 0.0
        self._last_ros_calibration = 0.0
        self._thread_list: List[threading.Thread] = []
        self.traj_running_event = threading.Event()
        self.traj_progress = 0.0
        self.traj_list_before_stop: Union[List[Tuple[float, ...]], np.ndarray] = []

    def calibrate_basepoint(self, wait=True) -> np.ndarray:
        """
        重置基地点到当前雷达位置 / cm
        """
        if wait and not self.radar.rt_pose_update_event.wait(1):
            logger.error("[NAVI] reset_basepoint(): Radar pose update timeout")
            raise RuntimeError("Radar pose update timeout")
        x, y, _ = self.radar.rt_pose
        self.basepoint = np.array([x, y])
        logger.info(f"[NAVI] Basepoint reset to {self.basepoint}")
        return self.basepoint

    def set_basepoint(self, point):
        """
        设置基地点(雷达坐标系) / cm
        """
        self.basepoint = np.asarray(point)
        logger.info(f"[NAVI] Basepoint set to {self.basepoint}")

    def set_navigation_state(self, state: bool):
        """
        设置导航状态
        """
        self.navigation_flag = state
        if state and self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            logger.debug("[NAVI] Auto set fc mode to HOLD_POS_MODE")

    def set_keep_height_state(self, state: bool):
        """
        设置定高状态
        """
        self.keep_height_flag = state

    def stop(self, join=False):
        """
        停止导航
        """
        self.running = False
        with self._control_lock:
            self._velocity_override_active = False
            self._velocity_override_keep_height = False
            self._velocity_override_faulted = False
            self._velocity_override_horizontal_cancelled = False
        try:
            self.update_realtime_control(vel_x=0, vel_y=0, vel_z=0, yaw=0)
        except Exception:
            logger.exception("[NAVI] Failed to send zero control while stopping")
        self.radar.stop_resolve_pose()
        if join:
            for thread in self._thread_list:
                thread.join()
        logger.info("[NAVI] Navigation stopped")

    def start(self, mode="fusion"):
        """
        启动导航
        mode: 导航模式, "radar"/"rs"/"fusion"/"fusion-ros"
        """
        if self.running:
            logger.warning("[NAVI] Navigation already running, restarting...")
            self.stop(join=True)
        self.running = True
        self.radar.subtask_skip = PARAMS.RADAR_SKIP
        
        # 此处为没有t265的修改
        if self.rs:
            self.rs.event_skip = PARAMS.RS_SKIP

        self._fusion_skip = PARAMS.FUSION_SKIP
        if self.mapper is not None:
            self.mapper.trans_event_skip = PARAMS.MAP_SKIP
        self._fusion_cnt = 0
        self._t265_trans_args = None
        self._velocity_override_active = False
        self._velocity_override_keep_height = False
        self._velocity_override_updated_at = 0.0
        self._velocity_override_faulted = False
        self._velocity_override_horizontal_cancelled = False
        self.switch_navigation_mode(mode)  # type: ignore
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self.update_realtime_control(vel_x=0, vel_y=0, vel_z=0, yaw=0)
        logger.info("[NAVI] Realtime control started")
        self._thread_list.append(threading.Thread(target=self._keep_height_task, daemon=True))
        self._thread_list[-1].start()
        self._thread_list.append(threading.Thread(target=self._navigation_task, daemon=True))
        self._thread_list[-1].start()
        self._thread_list.append(
            threading.Thread(target=self._velocity_override_watchdog_task, daemon=True)
        )
        self._thread_list[-1].start()
        logger.info("[NAVI] Navigation started")

    def update_realtime_control(
        self,
        vel_x: Optional[int] = None,
        vel_y: Optional[int] = None,
        vel_z: Optional[int] = None,
        yaw: Optional[int] = None,
        _source: Literal["external", "navigation", "height"] = "external",
    ) -> None:
        """
        更新实时控制帧
        """
        if _source not in ("external", "navigation", "height"):
            raise ValueError("invalid realtime-control source")
        with self._control_lock:
            now = time.monotonic()
            if not self.running:
                self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
            elif self._velocity_override_active:
                if _source == "external":
                    full_zero = bool(
                        vel_x == 0
                        and vel_y == 0
                        and vel_z == 0
                        and yaw == 0
                    )
                    if not full_zero:
                        raise RuntimeError(
                            "external realtime control is blocked by velocity override"
                        )
                    self._velocity_override_faulted = True
                    self._velocity_override_keep_height = False
                    self.keep_height_flag = False
                    self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
                elif (
                    self._velocity_override_keep_height
                    and _source == "height"
                    and vel_z is not None
                ):
                    self._realtime_control_data_in_xyzYaw[2] = vel_z
            else:
                allow_navigation = not (
                    _source == "navigation" and not self.navigation_flag
                )
                allow_height = not (
                    _source == "height" and not self.keep_height_flag
                )
                if allow_navigation and vel_x is not None:
                    self._realtime_control_data_in_xyzYaw[0] = vel_x
                if allow_navigation and vel_y is not None:
                    self._realtime_control_data_in_xyzYaw[1] = vel_y
                if allow_height and vel_z is not None:
                    self._realtime_control_data_in_xyzYaw[2] = vel_z
                if allow_navigation and yaw is not None:
                    self._realtime_control_data_in_xyzYaw[3] = yaw
                if allow_navigation and (
                    vel_x is not None or vel_y is not None or yaw is not None
                ):
                    self._navigation_control_updated_at = now
            control = list(self._realtime_control_data_in_xyzYaw)
            if (
                self.running
                and self.navigation_flag
                and now - self._navigation_control_updated_at > NAVIGATION_CONTROL_STALE_TIMEOUT
            ):
                control[0] = 0
                control[1] = 0
                control[3] = 0
            if (
                self.running
                and self._velocity_override_active
                and now - self._velocity_override_updated_at
                > NAVIGATION_CONTROL_STALE_TIMEOUT
            ):
                if not self._velocity_override_faulted:
                    logger.error(
                        "[NAVI] Velocity override timed out; "
                        "control revoked and zeroed"
                    )
                self._velocity_override_faulted = True
                self._realtime_control_data_in_xyzYaw[0] = 0
                self._realtime_control_data_in_xyzYaw[1] = 0
                self._realtime_control_data_in_xyzYaw[3] = 0
                if not self._velocity_override_keep_height:
                    self._realtime_control_data_in_xyzYaw[2] = 0
                control = list(self._realtime_control_data_in_xyzYaw)
            self.fc.send_realtime_control_data(*control)

    def _velocity_override_watchdog_task(self):
        """接管租约失效后撤销控制，并持续发送零速度帧直到退出。"""
        while self.running:
            try:
                with self._control_lock:
                    stale_now = bool(
                        self._velocity_override_active
                        and time.monotonic() - self._velocity_override_updated_at
                        > NAVIGATION_CONTROL_STALE_TIMEOUT
                    )
                    if self._velocity_override_active and (
                        stale_now or self._velocity_override_faulted
                    ):
                        first_fault = not self._velocity_override_faulted
                        self._velocity_override_faulted = True
                        self._realtime_control_data_in_xyzYaw[0] = 0
                        self._realtime_control_data_in_xyzYaw[1] = 0
                        self._realtime_control_data_in_xyzYaw[3] = 0
                        if not self._velocity_override_keep_height:
                            self._realtime_control_data_in_xyzYaw[2] = 0
                        self.fc.send_realtime_control_data(
                            *self._realtime_control_data_in_xyzYaw
                        )
                        if first_fault:
                            logger.error(
                                "[NAVI] Velocity override timed out; "
                                "control revoked and zeroed"
                            )
            except Exception:
                logger.exception("[NAVI] Velocity override watchdog error")
            time.sleep(0.05)

    def switch_navigation_mode(self, mode: Literal["radar", "rs", "fusion", "fusion-ros"]):
        """
        切换导航模式
        radar: 仅雷达扫网定位
        rs: 仅T265定位
        fusion: 雷达扫网定位辅助T265定位
        fusion-ros: ROS建图辅助T265定位
        """
        assert mode in ("radar", "rs", "fusion", "fusion-ros"), "Invalid navigation mode"
        if mode == "radar" or mode == "fusion":
            assert self.radar.running, "Radar not running"
            self.radar.start_resolve_pose(
                size=PARAMS.MAP_SIZE,
                scale_ratio=PARAMS.SCALE_RATIO,
                low_pass_ratio=PARAMS.LOW_PASS_RATIO,
                polyline=PARAMS.POLYLINE,
            )
            logger.info("[NAVI] Radar resolve pose started")
        elif self.radar._rtpose_flag:
            self.radar.stop_resolve_pose()
            logger.info("[NAVI] Radar resolve pose stopped")
        if mode == "rs" or mode == "fusion" or mode == "fusion-ros":
            assert self.rs.running, "RealSense not running"
        if mode == "fusion-ros":
            assert self.mapper is not None, "Mapper not initialized"
        self._navigation_mode = mode
        logger.info(f"[NAVI] Navigation mode switched to {mode}")

    def _rs_speed_report_callback(self, pose: T265_Pose_Frame, _, __):
        vel_x = round(-pose.velocity.z * 100)
        vel_y = round(-pose.velocity.x * 100)
        vel_z = round(pose.velocity.y * 100)
        self.fc.send_general_speed(x=vel_x, y=vel_y, z=vel_z)
        # pos_x = round(-pose.position.z * 100)
        # pos_y = round(-pose.position.x * 100)
        # pos_z = round(pose.position.y * 100)
        # fc.send_general_position(x=pos_x, y=pos_y, z=pos_z)

    def set_rs_speed_report(self, state: bool, skip: int = 1):
        """
        设置RealSense速度上报状态
        skip: 速度上报间隔(freq = 200/skip)
        """
        if state:
            self.rs.register_callback(self._rs_speed_report_callback, skip)
        else:
            self.rs.unregister_callback(self._rs_speed_report_callback)

    def switch_pid(self, pid: Union[str, tuple]):
        """
        切换平面导航PID参数

        pid: str:在self.pid_tunings中的键值 / tuple:自定义PID参数
        """
        if isinstance(pid, str):
            tuning = self.pid_tunings.get(pid, self.pid_tunings["default"])
        else:
            tuning = pid  # type: ignore
        self.navi_x_pid.tunings = tuning
        self.navi_y_pid.tunings = tuning
        logger.debug(f"[NAVI] PID Tunings set to {pid}: {tuning}")

    def _keep_height_task(self):
        paused = False
        while self.running:
            try:
                if not self.keep_height_by_rs:
                    if not self.fc.state.update_event.wait(1):
                        logger.warning("[NAVI] FC state update timeout")
                        self.update_realtime_control(vel_z=0, _source="height")
                        continue
                    self.fc.state.update_event.clear()
                    self.current_height = self.fc.state.alt_add.value
                    height = self.current_height
                else:
                    if not self.rs.update_event.wait(1):
                        logger.warning("[NAVI] RealSense height timeout")
                        self.update_realtime_control(vel_z=0, _source="height")
                        continue
                    height = self.current_height_rs
                logger_dbg.debug(f"[NAVI] Current height: {height}")
                if not (
                    self.keep_height_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                    and self.fc.state.unlock.value
                ):
                    if not paused:
                        paused = True
                        self.height_pid.set_auto_mode(False)
                        self.update_realtime_control(vel_z=0, _source="height")
                        logger.info("[NAVI] Keep height paused")
                    continue
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[NAVI] Keep Height resumed")
                out_hei = round(self.height_pid(height))  # type: ignore
                self.update_realtime_control(vel_z=out_hei, _source="height")
                logger_dbg.info(f"[NAVI] Height PID output: {out_hei}")
            except Exception as e:
                logger.exception("[NAVI] Keep height task error")
                self.update_realtime_control(vel_z=0, _source="height")

    def _get_t265_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
        if wait and not self.rs.update_event.wait(POSE_STALE_TIMEOUT):
            logger.warning("[NAVI] RealSense pose timeout")
            return None
        self.rs.update_event.clear()
        pose = self.rs.get_pose_snapshot() if hasattr(self.rs, "get_pose_snapshot") else self.rs.pose
        last_update = float(getattr(self.rs, "last_update_monotonic", 0.0))
        fresh = last_update > 0 and time.monotonic() - last_update <= POSE_STALE_TIMEOUT
        if self._t265_trans_args is None:
            current_x = -pose.translation.z * 100
            current_y = -pose.translation.x * 100
            self.current_height_rs = pose.translation.y * 100
            current_yaw = -self.rs.get_eular_rotation(pose)[2] if hasattr(self.rs, "get_eular_rotation") else -self.rs.eular_rotation[2]
        else:
            position, eular = self.rs.get_pose_in_secondary_frame(self._t265_trans_args, as_eular=True)
            current_x = -position[2] * 100  # type: ignore
            current_y = -position[0] * 100  # type: ignore
            self.current_height_rs = position[1] * 100  # type: ignore
            current_yaw = -eular[2]  # type: ignore
        available = fresh and pose.tracker_confidence >= 2
        if available:
            self._last_pose_update = time.monotonic()
        logger_dbg.debug(f"[NAVI] RealSense pose: {current_x}, {current_y}, {current_yaw}, {available}")
        return current_x, current_y, current_yaw, available  # type: ignore

    def _get_radar_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
        if wait and not self.radar.rt_pose_update_event.wait(1):
            logger.warning("[NAVI] Radar pose timeout")
            return None
        self.radar.rt_pose_update_event.clear()

        current_x, current_y, current_yaw = self.radar.rt_pose
        current_x -= self.basepoint[0]
        current_y -= self.basepoint[1]
        logger_dbg.debug(f"[NAVI] Radar pose: {current_x}, {current_y}, {current_yaw}")


        inited = getattr(self.radar, "_rt_pose_inited", [True, True, True])
        available = bool(getattr(self.radar, "_rtpose_flag", False) and all(inited))
        return float(current_x), float(current_y), float(current_yaw), available

    def _get_fusion_pose(self) -> Optional[Tuple[float, float, float, bool]]:
        if self.radar.rt_pose_update_event.is_set():
            self.radar.rt_pose_update_event.clear()
            self._fusion_cnt += 1
        if self._fusion_cnt >= self._fusion_skip:
            self._fusion_cnt = 0
            self.calibrate_realsense(wait=False)
        return self._get_t265_pose()

    def _get_fusion_ros_pose(self) -> Optional[Tuple[float, float, float, bool]]:
        ret = self._get_t265_pose()
        if not ret:
            return None
        now = time.monotonic()
        if (
            self.mapper.trans_update_event.is_set()  # type: ignore
            and now - self._last_ros_calibration >= FUSION_ROS_CALIBRATION_INTERVAL
            and self.mapper.is_transform_fresh(POSE_STALE_TIMEOUT)  # type: ignore
        ):
            self.mapper.trans_update_event.clear()  # type: ignore
            self.calibrate_realsense_ros(
                wait=False,
                max_position_correction_cm=FUSION_ROS_MAX_POSITION_CORRECTION_CM,
                max_yaw_correction_deg=FUSION_ROS_MAX_YAW_CORRECTION_DEG,
            )
            self._last_ros_calibration = now
            ret = self._get_t265_pose(wait=False) or ret
        x, y, yaw, avai = ret
        return x, y, yaw, avai and self.mapper.is_transform_fresh(POSE_STALE_TIMEOUT)  # type: ignore

    def _navigation_task(self):
        paused = False
        while self.running:
            try:
                if self.stop_event is not None and self.stop_event.is_set():
                    self.update_realtime_control(
                        vel_x=0, vel_y=0, yaw=0, _source="navigation"
                    )
                    time.sleep(0.05)
                    continue
                if self._navigation_mode == "radar":
                    pose = self._get_radar_pose()
                elif self._navigation_mode == "rs":
                    pose = self._get_t265_pose()
                elif self._navigation_mode == "fusion":
                    pose = self._get_fusion_pose()
                elif self._navigation_mode == "fusion-ros":
                    pose = self._get_fusion_ros_pose()
                else:
                    raise ValueError(f"Unknown navigation mode: {self._navigation_mode}")
                if pose is None:
                    self.update_realtime_control(
                        vel_x=0, vel_y=0, yaw=0, _source="navigation"
                    )
                    logger.warning("[NAVI] Navigation pose not available")
                    continue
                self.current_x, self.current_y, self.current_yaw, available = (
                    float(pose[0]),
                    float(pose[1]),
                    float(pose[2]),
                    bool(pose[3]),
                )
                logger_dbg.info(f"[NAVI] Pose: {self.current_x}, {self.current_y}, {self.current_yaw}")
                if not (
                    self.navigation_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                    and self.fc.state.unlock.value
                ):  # 导航需在解锁/定点模式下运行
                    if not paused:
                        paused = True
                        self.navi_x_pid.set_auto_mode(False)
                        self.navi_y_pid.set_auto_mode(False)
                        self.yaw_pid.set_auto_mode(False)
                        self.update_realtime_control(
                            vel_x=0, vel_y=0, yaw=0, _source="navigation"
                        )
                        logger.info("[NAVI] Navigation paused")
                    continue
                if paused:
                    paused = False
                    self.navi_x_pid.set_auto_mode(True, last_output=0)
                    self.navi_y_pid.set_auto_mode(True, last_output=0)
                    self.yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[NAVI] Navigation resumed")
                if not available:
                    logger.warning("[NAVI] Pose not available")
                    self.update_realtime_control(
                        vel_x=0, vel_y=0, yaw=0, _source="navigation"
                    )
                    time.sleep(0.1)
                    continue
                # self.fc.send_general_position(x=self.current_x, y=self.current_y)
                out_x_world = self.navi_x_pid(self.current_x)
                out_y_world = self.navi_y_pid(self.current_y)
                yaw_error = _shortest_yaw_error(
                    self.yaw_target, self.current_yaw, self._yaw_direction_hint
                )
                out_yaw = self.yaw_pid(-yaw_error)
                if out_x_world is None or out_y_world is None or out_yaw is None:
                    continue
                out_x_body, out_y_body = _world_to_body_velocity(
                    out_x_world, out_y_world, self.current_yaw
                )
                out_x_body = round(out_x_body)
                out_y_body = round(out_y_body)
                out_yaw = round(out_yaw)
                self.update_realtime_control(
                    vel_x=out_x_body,
                    vel_y=out_y_body,
                    yaw=out_yaw,
                    _source="navigation",
                )
                logger_dbg.info(
                    f"[NAVI] Pose PID output: world=({out_x_world}, {out_y_world}), "
                    f"body=({out_x_body}, {out_y_body}), yaw={out_yaw}"
                )
            except Exception as e:
                logger.exception(f"[NAVI] Navigation task error")
                self.update_realtime_control(
                    vel_x=0, vel_y=0, yaw=0, _source="navigation"
                )

    def calibrate_realsense(self, wait=True):
        """
        根据雷达扫网定位数据校准T265的坐标系
        """
        if wait and not self.radar.rt_pose_update_event.wait(1):
            raise RuntimeError("Radar pose update timeout")
        x, y, yaw = self.radar.rt_pose
        dx = x - self.basepoint[0]  # -> t265 -z * 100
        dx = -dx / 100.0
        dy = y - self.basepoint[1]  # -> t265 -x * 100
        dy = -dy / 100.0
        dyaw = -yaw
        if not self.keep_height_by_rs:
            dz = self.fc.state.alt_add.value / 100.0
        else:
            dz = self.current_height_rs / 100.0
        logger_dbg.info(f"[NAVI] Calibrate T265: radar={self.radar.rt_pose} dz={dx}, dx={dy}, dy={dz}, dyaw={dyaw}")
        self._t265_trans_args = self.rs.establish_secondary_origin(
            force_level=True, z_offset=dx, x_offset=dy, yaw_offset=dyaw, y_offset=dz
        )

    def calibrate_realsense_ros(
        self,
        wait=True,
        max_position_correction_cm: Optional[float] = None,
        max_yaw_correction_deg: Optional[float] = None,
    ):
        """
        根据ROS建图数据校准T265的坐标系
        """
        if wait and not self.mapper.trans_update_event.wait(1):  # type: ignore
            raise RuntimeError("Mapper transform update timeout")
        x, y, _ = self.mapper.position  # type: ignore
        _, _, yaw = self.mapper.eular_rotation  # type: ignore
        desired_x_cm = float(x) * 100.0
        desired_y_cm = float(y) * 100.0
        desired_nav_yaw = -float(yaw)
        if self._t265_trans_args is not None and max_position_correction_cm is not None:
            correction = np.array(
                [desired_x_cm - self.current_x, desired_y_cm - self.current_y],
                dtype=float,
            )
            correction_norm = float(np.linalg.norm(correction))
            max_correction = max(0.0, float(max_position_correction_cm))
            if correction_norm > max_correction > 0:
                correction *= max_correction / correction_norm
            desired_x_cm = float(self.current_x + correction[0])
            desired_y_cm = float(self.current_y + correction[1])
        if self._t265_trans_args is not None and max_yaw_correction_deg is not None:
            yaw_correction = _shortest_yaw_error(desired_nav_yaw, self.current_yaw)
            yaw_correction = float(
                np.clip(
                    yaw_correction,
                    -abs(float(max_yaw_correction_deg)),
                    abs(float(max_yaw_correction_deg)),
                )
            )
            desired_nav_yaw = float(self.current_yaw + yaw_correction)
        dx = -desired_x_cm / 100.0
        dy = -desired_y_cm / 100.0
        dyaw = -desired_nav_yaw
        if not self.keep_height_by_rs:
            dz = self.fc.state.alt_add.value / 100.0
        else:
            dz = self.current_height_rs / 100.0
        logger_dbg.info(
            f"[NAVI] Calibrate T265: map={self.mapper.position} "  # type: ignore
            f"desired=({desired_x_cm:.1f},{desired_y_cm:.1f},{desired_nav_yaw:.1f}) "
            f"dz={dx}, dx={dy}, dy={dz}, dyaw={dyaw}"
        )
        self._t265_trans_args = self.rs.establish_secondary_origin(
            force_level=True, z_offset=dx, x_offset=dy, yaw_offset=dyaw, y_offset=dz
        )

    def direct_set_waypoint(self, waypoint):
        """
        直接设置水平导航PID目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        self.navi_x_pid.setpoint = waypoint[0]
        self.navi_y_pid.setpoint = waypoint[1]
        if len(waypoint) > 2:
            self.height_pid.setpoint = waypoint[2]

    def navigation_to_waypoint(self, waypoint, wait=True, dt: float = 0.1):
        """
        创建直线航线并导航到指定的目标点

        waypoint: (x, y, [z]) 相对于基地点的坐标 / cm / 匿名(ROS)坐标系 / 基地原点
        wait: 是否阻塞直到到达目标点
        dt: 轨迹精度 / s
        """
        logger.debug(f"[NAVI] Navigation to waypoint: {waypoint}")
        if len(waypoint) == 2:
            waypoint = [waypoint[0], waypoint[1], self.height_pid.setpoint]
        else:
            waypoint = [waypoint[0], waypoint[1], waypoint[2]]
        waypoint_cur = [self.current_x, self.current_y, self.current_height]
        length = np.linalg.norm(np.array(waypoint) - np.array(waypoint_cur))  # type: ignore
        if length <= 1e-6:
            self.direct_set_waypoint(waypoint)
            if wait:
                return self.wait_for_waypoint(time_thres=0.1, timeout=1.0)
            return True
        tT = float(length / self.navi_speed)
        traj = TrajectoryGenerator(start_pos=waypoint_cur, des_pos=waypoint, T=tT)
        traj.solve()
        traj_list = []
        for t in np.arange(0, tT, dt):
            traj_list.append(traj.calc_position_xyz(t))
        traj_list.append(waypoint)
        return self.navigation_follow_trajectory(traj_list, wait=wait)  # type: ignore

    def create_smooth_traj_list(
        self,
        waypoints,
        altitude: Optional[float] = None,
        config: Optional[SplineTrajectoryConfig] = None,
        extended: bool = False,
    ) -> List[Tuple[float, ...]]:
        """根据有序二维航点生成平滑轨迹点列表。

        waypoints: ``[(x, y), ...]``，单位 cm，与当前导航坐标系一致
        altitude: 固定轨迹高度 / cm，默认使用当前定高目标
        config: 平滑与速度规划参数；其中巡航速度始终使用当前 navi_speed
        extended: 是否输出带 ``t/vx/vy/speed_limit`` 的扩展点格式
        """
        if altitude is None:
            altitude = float(self.height_pid.setpoint)
        trajectory_config = (
            config or SplineTrajectoryConfig(navi_speed=self.navi_speed)
        ).with_navi_speed(self.navi_speed)
        generator = SplineTrajectoryGenerator(
            waypoints=waypoints,
            altitude=float(altitude),
            config=trajectory_config,
        )
        return generator.generate_traj_list(extended=extended)

    def navigation_follow_waypoints(
        self,
        waypoints,
        wait=True,
        altitude: Optional[float] = None,
        pos_thres: float = 10.0,
        config: Optional[SplineTrajectoryConfig] = None,
    ):
        """以当前位置为起点生成平滑多航点轨迹，并交给现有执行器。

        输入的第一个航点会作为路径中的第二个点。轨迹起点在调用时读取
        ``current_x/current_y/current_height``；未指定 altitude 时使用当前
        定高 PID 目标。当前执行器仍按位置点逐点推进，因此配置中的时间
        和速度规划决定采样位置，但不等价于按时间戳闭环跟踪。
        """
        planned_waypoints = np.asarray(waypoints, dtype=float)
        if (
            planned_waypoints.ndim != 2
            or planned_waypoints.shape[0] == 0
            or planned_waypoints.shape[1] < 2
        ):
            raise ValueError("waypoints must contain at least one (x, y) point")
        if not np.all(np.isfinite(planned_waypoints[:, :2])):
            raise ValueError("waypoints must contain only finite coordinates")

        start_x = float(self.current_x)
        start_y = float(self.current_y)
        start_height = float(self.current_height)
        if not np.all(np.isfinite([start_x, start_y, start_height])):
            raise ValueError("current navigation position must be finite")

        trajectory_waypoints = np.vstack(
            ([start_x, start_y], planned_waypoints[:, :2])
        )
        traj_list = self.create_smooth_traj_list(
            waypoints=trajectory_waypoints,
            altitude=altitude,
            config=config,
            extended=False,
        )
        first_point = traj_list[0]
        traj_list[0] = (first_point[0], first_point[1], start_height)
        return self.navigation_follow_trajectory(
            traj_list,
            wait=wait,
            pos_thres=pos_thres,
        )

    def navigation_around_waypoint(
            self,
            waypoint,
            wait=True,
            dt: float = 0.2,
            degree: float = 2 * np.pi,
            mode: str = "counterclockwise",
            radius: Optional[float] = None,
            pos_thres: float = 10.0,
    ):
        """
        创建圆形轨迹并让无人机进行圆形巡航
        waypoint: (x, y, [z]) 圆心坐标 / cm / 匿名(ROS)坐标系 / 基地原点
        wait: 是否阻塞直到完成圆形巡航
        dt: 轨迹精度 / s
        degree: 转过的角度 / rad，可为负值，负值表示反向（与mode指定的方向相反）
        mode: 转向 / 默认为俯视逆时针
        """
        center = np.asarray(waypoint[:2], dtype=float)
        cur = np.asarray([float(self.current_x), float(self.current_y)], dtype=float)

        r_meas = float(np.linalg.norm(cur - center))
        r = float(r_meas if radius is None else radius)
        r = max(r, 1e-3)

        start_angle = float(np.arctan2(cur[1] - center[1], cur[0] - center[0]))

        if mode not in ("counterclockwise", "clockwise"):
            raise ValueError("mode must be 'counterclockwise' or 'clockwise'")

        direction = (1.0 if mode == "counterclockwise" else -1.0) * (1.0 if degree >= 0 else -1.0)
        total = float(abs(degree))

        speed = float(max(self.navi_speed, 1e-3))
        dt = float(max(dt, 1e-3))

        # 角步进：v*dt/r
        angle_step = speed * dt / r
        steps = int(np.ceil(total / angle_step))
        steps = max(steps, 1)

        angles = start_angle + direction * np.linspace(0.0, total, steps + 1)

        # 高度保持：优先用定高目标值（更稳），否则用当前高度
        z = float(self.height_pid.setpoint if self.keep_height_flag else self.current_height)

        traj_list = []
        for a in angles:
            x = float(center[0] + r * np.cos(a))
            y = float(center[1] + r * np.sin(a))
            traj_list.append([x, y, z])

        self.navigation_follow_trajectory(traj_list, wait=wait, pos_thres=pos_thres)
 
    def _trajectory_task(
        self,
        traj_list: Union[List[Tuple[float, ...]], np.ndarray],
        pos_thres: float = 10.0,
        timeout_per_point: float = 6.0,
    ):
        """
        轨迹跟随任务（改进版）
        - 改成：先设置目标点 -> 再等待到达（避免“先等旧目标达成再切点”的跳点问题）
        - 用欧式距离判定是否到点（更适合绕圆）
        - 支持pos_thres（绕杆建议8~12cm）
        """
        logger.debug("[NAVI] Trajectory task started")
        if len(traj_list) == 0:
            raise ValueError("trajectory must contain at least one point")
        self.traj_running_event.set()

        pos_thres = float(max(pos_thres, 1.0))
        th2 = pos_thres * pos_thres

        len_t = len(traj_list)

        for n, point in enumerate(traj_list):
            if self.stop_event is not None and self.stop_event.is_set():
                logger.warning("[NAVI] Trajectory stopped by external stop event")
                self.traj_running_event.clear()
                return False
            if not self.pose_is_fresh():
                logger.error("[NAVI] Trajectory aborted because navigation pose is stale")
                self.traj_running_event.clear()
                return False
            if not (self.running and self.navigation_flag):
                logger.debug("[NAVI] Trajectory task forced to stop (nav not running)")
                return

            # 外部请求停止：clear event
            if not self.traj_running_event.is_set():
                logger.debug("[NAVI] Trajectory task forced to stop")
                self.traj_list_before_stop = traj_list[n:]
                return

            x, y = float(point[0]), float(point[1])
            self.navi_x_pid.setpoint = x
            self.navi_y_pid.setpoint = y
            if len(point) > 2:
                self.height_pid.setpoint = float(point[2])

            self.traj_progress = (n + 1) / len_t

            # 等待到达当前点
            t0 = time.perf_counter()
            while True:
                time.sleep(0.02)

                if not self.traj_running_event.is_set():
                    logger.debug("[NAVI] Trajectory task forced to stop")
                    self.traj_list_before_stop = traj_list[n:]
                    return

                if not (self.running and self.navigation_flag):
                    logger.debug("[NAVI] Trajectory task forced to stop (nav not running)")
                    return
                if self.stop_event is not None and self.stop_event.is_set():
                    logger.warning("[NAVI] Trajectory stopped by external stop event")
                    self.traj_running_event.clear()
                    return False
                if not self.pose_is_fresh():
                    logger.error("[NAVI] Trajectory aborted because navigation pose is stale")
                    self.navi_x_pid.setpoint = self.current_x
                    self.navi_y_pid.setpoint = self.current_y
                    self.traj_running_event.clear()
                    return False

                dx = float(self.current_x) - x
                dy = float(self.current_y) - y
                if dx * dx + dy * dy <= th2:
                    break

                if timeout_per_point > 0 and (time.perf_counter() - t0) > timeout_per_point:
                    logger.error("[NAVI] Trajectory point timeout; abort remaining trajectory")
                    self.navi_x_pid.setpoint = self.current_x
                    self.navi_y_pid.setpoint = self.current_y
                    self.traj_running_event.clear()
                    return False

        self.traj_running_event.clear()
        logger.debug("[NAVI] Trajectory task finished")
        return True

    def navigation_follow_trajectory(
        self,
        traj_list: Union[List[Tuple[float, ...]], np.ndarray],
        wait=True,
        pos_thres: float = 10.0,
    ):
        """
        跟随轨迹导航（改进版）
        - 允许传入pos_thres并传递给轨迹任务
        """
        logger.debug(f"[NAVI] Running on trajectory with {len(traj_list)} points")
        self.navi_x_pid.tunings = self.pid_tunings["navi"]
        self.navi_y_pid.tunings = self.pid_tunings["navi"]
        self.navi_x_pid.output_limits = (-self.navi_speed, self.navi_speed)
        self.navi_y_pid.output_limits = (-self.navi_speed, self.navi_speed)

        if wait:
            trajectory_ok = self._trajectory_task(traj_list, pos_thres=pos_thres)
            if not trajectory_ok:
                return False
            # 最后一段再确认一次到点（用更小阈值更贴轨）
            return self.wait_for_waypoint(
                time_thres=0.5,
                pos_thres=max(8, int(pos_thres)),
                timeout=10,
            )
        else:
            t = threading.Thread(
                target=self._trajectory_task,
                args=(traj_list,),
                kwargs={"pos_thres": pos_thres},
                daemon=True,
            )
            t.start()
            self._thread_list.append(t)
            self.traj_running_event.wait()
            return True

    @property
    def navigation_target(self) -> np.ndarray:
        """
        当前导航目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        return np.array([self.navi_x_pid.setpoint, self.navi_y_pid.setpoint])

    @navigation_target.setter
    def navigation_target(self, waypoint: np.ndarray):
        return self.navigation_to_waypoint(waypoint)

    @property
    def current_point(self) -> np.ndarray:
        """
        当前位置 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        return np.array([self.current_x, self.current_y])

    def navigation_stop_here(self) -> np.ndarray:
        """
        原地停止(设置目标点为当前位置)

        return: 原定目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        waypoint = self.navigation_target
        x, y = self.current_x, self.current_y
        if self.traj_running_event.is_set():
            self.traj_running_event.clear()
            self.traj_running_event.wait(0.1)  # 等待轨迹任务停止
            self.traj_running_event.clear()
        self.navi_x_pid.setpoint = x
        self.navi_y_pid.setpoint = y
        self._waypoint_param_switch()
        logger.debug(f"[NAVI] Navigation stopped at {x}, {y}")
        return waypoint

    def set_height(self, height: float):
        """
        设置飞行高度

        height: 激光高度 / cm
        """
        self.height_pid.setpoint = height
        logger.debug(f"[NAVI] Keep height set to {height}")

    def set_yaw(self, yaw: float):
        """
        设置飞行航向

        yaw: 相对于初始状态的航向角 / deg
        """
        if not np.isfinite(yaw):
            raise ValueError("yaw must be finite")
        initial_error = _shortest_yaw_error(float(yaw), self.current_yaw)
        self._yaw_direction_hint = 1 if initial_error > 0 else (-1 if initial_error < 0 else 0)
        self.yaw_target = float(yaw)
        logger.debug(f"[NAVI] Keep yaw set to {self.yaw_target}")

    def pose_is_fresh(self, max_age: float = POSE_STALE_TIMEOUT) -> bool:
        """Return whether a usable navigation pose was received recently."""
        return bool(
            self._last_pose_update > 0
            and time.monotonic() - self._last_pose_update <= float(max_age)
        )

    def navigation_to_waypoint_relative(self, waypoint_rel, *args, **kwargs):
        """
        导航到指定的目标点

        waypoint_rel: (x, y) 坐标 / cm / 匿名(ROS)坐标系 / 当前位置原点
        其余参数参考navigation_to_waypoint
        """
        self.navigation_to_waypoint(self.current_point + np.asarray(waypoint_rel), *args, **kwargs)

    def set_navigation_speed(self, speed):
        """
        设置导航速度

        speed: 速度 / cm/s
        """
        speed = abs(speed)
        self.navi_x_pid.output_limits = (-speed, speed)
        self.navi_y_pid.output_limits = (-speed, speed)
        self.navi_speed = speed
        logger.info(f"[NAVI] Navigation speed set to {speed}")

    def set_vertical_speed(self, speed):
        """
        设置垂直速度

        speed: 速度 / cm/s
        """
        speed = abs(speed)
        self.height_pid.output_limits = (-speed, speed)
        logger.info(f"[NAVI] Vertical speed set to {speed}")

    def set_yaw_speed(self, speed):
        """
        设置偏航速度

        speed: 速度 / deg/s
        """
        speed = abs(speed)
        self.yaw_pid.output_limits = (-speed, speed)
        logger.info(f"[NAVI] Yaw speed set to {speed}")

    def _reached_waypoint(self, pos_thres):
        return (
            abs(self.current_x - self.navi_x_pid.setpoint) < pos_thres
            and abs(self.current_y - self.navi_y_pid.setpoint) < pos_thres
        )

    def adjust_height_and_hover(
        self,
        target_height: float,
        point: Optional[Union[List[float], Tuple[float, float], np.ndarray]] = None,
        height_timeout: float = 15.0,
        pos_timeout: float = 12.0,
        pos_thres: float = 20.0,
        height_thres: float = 8.0,
        lock_pos_time: float = 1.0,
    ) -> None:
        """
        定点调整高度后悬停

        功能：
        1. 在当前位置或指定点调整飞行高度
        2. 调整过程中保持水平位置锁定
        3. 调整完成后稳定悬停

        参数：
        target_height: 目标飞行高度 / cm
        point: 目标水平坐标 (x, y) / cm / 匿名(ROS)坐标系 / 基地原点
            若为 None，则在当前位置调整高度
        height_timeout: 高度调整超时时间 / s
        pos_timeout: 位置锁定超时时间 / s (仅当 point 不为 None 时生效)
        pos_thres: 位置到达阈值 / cm
        height_thres: 高度到达阈值 / cm
        lock_pos_time: 位置稳定时间阈值 / s

        流程：
        1. 确保处于 HOLD_POS_MODE（定点模式）
        2. 开启高度保持与导航闭环
        3. 若指定 point，则锁定水平位置
        4. 设置目标高度并等待到达
        5. 若指定 point，等待位置稳定

        示例：
        # 在当前点爬升到 200cm 高度
        navi.adjust_height_and_hover(200)

        # 移动到 (100, 50) 点并爬升到 150cm
        navi.adjust_height_and_hover(150, point=[100, 50])
        """
        logger.info(f"[NAVI] Adjust height to {target_height}cm at {point if point else 'current position'}")

        # 1) 确保处于定点模式
        if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            time.sleep(0.1)
            logger.debug("[NAVI] Switched to HOLD_POS_MODE")

        # 2) 开启高度保持与导航
        self.keep_height_flag = True
        self.navigation_flag = True

        # 3) 若指定目标点，则锁定水平位置
        if point is not None:
            # 解析坐标点
            if isinstance(point, (list, tuple, np.ndarray)):
                x, y = float(point[0]), float(point[1])
            else:
                raise ValueError(f"Invalid point format: {point}, expected list/tuple/ndarray")

            # 切换到悬停PID参数（更柔和）
            self.switch_pid("hover")
            
            # 设置水平目标点
            self.direct_set_waypoint([x, y])
            logger.debug(f"[NAVI] Lock position to ({x}, {y})")

            # 等待位置初步稳定（避免高度调整时水平漂移过大）
            self.wait_for_waypoint(
                time_thres=lock_pos_time,
                pos_thres=pos_thres,
                timeout=pos_timeout,
            )
        else:
            # 使用当前位置，不改变水平目标
            logger.debug("[NAVI] Keep current horizontal position")

        # 4) 设置目标高度并等待到达
        current_h = float(self.fc.state.alt_add.value) if hasattr(self.fc.state.alt_add, 'value') else self.current_height
        logger.debug(f"[NAVI] Current height: {current_h:.1f}cm, Target: {target_height}cm")
        
        self.set_height(float(target_height))
        self.wait_for_height(
            time_thres=0.5,
            height_thres=height_thres,
            timeout=height_timeout,
        )

        # 5) 若指定了目标点，确保最终位置稳定
        if point is not None:
            self.wait_for_waypoint(
                time_thres=lock_pos_time,
                pos_thres=pos_thres,
                timeout=pos_timeout,
            )
            logger.info(f"[NAVI] Adjusted to height {target_height}cm at ({x}, {y}) and hovering")
        else:
            logger.info(f"[NAVI] Adjusted to height {target_height}cm and hovering at current position")

        # 6) 最终状态确认
        logger.debug(
            f"[NAVI] Final state - Height: {self.current_height:.1f}cm, "
            f"Position: ({self.current_x:.1f}, {self.current_y:.1f})"
        )

    def pointing_landing(
        self,
        point,
        approach_height=35,
        approach_pos_thres=12,
        settle_time_thres=0.5,
        settle_timeout=4,
        height_timeout=5,
        touchdown_alt_thres=8,
        touchdown_timeout=12,
        lock_timeout=4,
    ):
        """
        定点降落(快速版)

        point: (x, y) / cm / 匿名(ROS)坐标系 / 基地原点
        approach_height: 进入自动降落前的对点高度 / cm
        """
        logger.info(f"[NAVI] Landing at {point}")
        x, y = float(point[0]), float(point[1])

        # 阶段1: 在HOLD_POS下先对点并下探到较低高度(减少低空长时间悬停)
        self.navigation_flag = True
        self.keep_height_flag = True
        self.switch_pid("land")
        self.navigation_to_waypoint([x, y], wait=True)
        approach_ok = self.wait_for_waypoint(
            time_thres=max(0.3, float(settle_time_thres)),
            pos_thres=max(8, int(approach_pos_thres)),
            timeout=max(1.0, float(settle_timeout)),
        )

        self.set_height(float(max(10, approach_height)))
        height_ok = self.wait_for_height(
            time_thres=0.3,
            height_thres=10,
            timeout=max(1.0, float(height_timeout)),
        )
        if approach_ok and height_ok and self.pose_is_fresh():
            self.direct_set_waypoint([x, y])
            self.wait_for_waypoint(
                time_thres=0.3,
                pos_thres=max(8, int(approach_pos_thres)),
                timeout=max(1.0, float(settle_timeout)),
            )
        else:
            logger.warning(
                "[NAVI] Landing approach not confirmed; land at current position"
            )

        # 阶段2: 关闭导航闭环，切给飞控一键降落，避免PID慢速“磨地”
        self.navigation_flag = False
        self.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        time.sleep(0.1)
        self.fc.stablize()
        self.fc.land()

        # 等待落地。只有新鲜高度足够低或飞控已自动上锁才允许补发 lock。
        t0 = time.perf_counter()
        landed = False
        alt_thres = float(max(3, touchdown_alt_thres))
        while time.perf_counter() - t0 < max(1.0, float(touchdown_timeout)):
            time.sleep(0.1)
            try:
                alt_now = float(self.fc.state.alt_add.value)
            except Exception:
                alt_now = 999.0
            state_fresh = bool(
                getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
            )
            if (state_fresh and alt_now <= alt_thres) or (not self.fc.state.unlock.value):
                landed = True
                break

        if not landed:
            logger.error("[NAVI] Landing timeout; keep landing command active, refuse airborne force-lock")
            self.fc.land()
            return False

        try:
            ok = self.fc.wait_for_lock(timeout_s=lock_timeout)
        except TypeError:
            ok = self.fc.wait_for_lock(lock_timeout)
        if not ok:
            state_fresh = bool(
                getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
            )
            alt_now = float(self.fc.state.alt_add.value) if state_fresh else 999.0
            if state_fresh and alt_now <= alt_thres:
                self.fc.lock()
            else:
                logger.error("[NAVI] Lock not confirmed; refuse lock without fresh touchdown altitude")
                return False
        return True

    def _flight_state_is_fresh(self, max_age: float = 0.5) -> bool:
        return bool(
            getattr(self.fc.state, "is_fresh", lambda _age: False)(max_age)
        )

    @staticmethod
    def _validate_moving_descent_speeds(
        horizontal_speed: float,
        descent_speed: float,
    ) -> None:
        if horizontal_speed > MOVING_DESCENT_MAX_HORIZONTAL_SPEED:
            raise ValueError(
                "horizontal_speed must not exceed "
                f"{MOVING_DESCENT_MAX_HORIZONTAL_SPEED}cm/s"
            )
        if descent_speed > MOVING_DESCENT_MAX_VERTICAL_SPEED:
            raise ValueError(
                "descent_speed must not exceed "
                f"{MOVING_DESCENT_MAX_VERTICAL_SPEED}cm/s"
            )

    def _start_velocity_override(
        self,
        keep_height: bool = False,
        require_pose: bool = True,
    ) -> bool:
        """暂停导航 PID，并由调用方持续刷新实时速度控制量。"""
        if not self.running:
            logger.error("[NAVI] Velocity override requires navigation to be running")
            return False
        if self.stop_event is not None and self.stop_event.is_set():
            logger.error("[NAVI] Velocity override refused: external stop event is set")
            return False
        if not self._flight_state_is_fresh():
            logger.error("[NAVI] Velocity override refused: FC telemetry is stale")
            return False
        if not self.fc.state.unlock.value:
            logger.error("[NAVI] Velocity override requires an unlocked aircraft")
            return False
        if require_pose and not self.pose_is_fresh():
            logger.error("[NAVI] Velocity override refused: navigation pose is stale")
            return False
        with self._control_lock:
            if self._velocity_override_active:
                logger.error("[NAVI] Another velocity override is already active")
                return False

        if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            mode_deadline = time.perf_counter() + 1.0
            while time.perf_counter() < mode_deadline:
                if (
                    self._flight_state_is_fresh()
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                ):
                    break
                time.sleep(0.05)
            else:
                logger.error("[NAVI] Velocity override refused: HOLD_POS not confirmed")
                return False

        self.navigation_stop_here()
        with self._control_lock:
            if self._velocity_override_active:
                logger.error("[NAVI] Another velocity override is already active")
                return False
            self.navigation_flag = False
            self.keep_height_flag = bool(keep_height)
            self._velocity_override_active = True
            self._velocity_override_keep_height = bool(keep_height)
            self._velocity_override_updated_at = time.monotonic()
            self._velocity_override_faulted = False
            self._velocity_override_horizontal_cancelled = False
            self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
            try:
                self.fc.send_realtime_control_data(0, 0, 0, 0)
            except Exception:
                self._velocity_override_faulted = True
                self._velocity_override_horizontal_cancelled = True
                self.navigation_flag = False
                self.keep_height_flag = False
                logger.exception("[NAVI] Failed to start velocity override")
                return False
        logger.info(
            f"[NAVI] Velocity override started, keep_height={bool(keep_height)}"
        )
        return True

    def _update_velocity_override(
        self,
        vel_x: float,
        vel_y: float,
        vel_z: Optional[float] = None,
        yaw: float = 0,
        frame: Literal["world", "body"] = "world",
    ) -> None:
        """更新速度接管指令；调用方必须以高于失效频率持续刷新。"""
        values = [vel_x, vel_y, yaw]
        if vel_z is not None:
            values.append(vel_z)
        if not np.all(np.isfinite(np.asarray(values, dtype=float))):
            raise ValueError("velocity override values must be finite")
        if frame not in ("world", "body"):
            raise ValueError("frame must be 'world' or 'body'")

        if frame == "world":
            vel_x, vel_y = _world_to_body_velocity(
                vel_x,
                vel_y,
                self.current_yaw,
            )
        with self._control_lock:
            if not self._velocity_override_active:
                raise RuntimeError("velocity override is not active")
            if self._velocity_override_faulted:
                raise RuntimeError("velocity override lease has been revoked")
            self._velocity_override_updated_at = time.monotonic()
            if self._velocity_override_horizontal_cancelled:
                vel_x = 0
                vel_y = 0
            self._realtime_control_data_in_xyzYaw[0] = round(vel_x)
            self._realtime_control_data_in_xyzYaw[1] = round(vel_y)
            if vel_z is not None:
                self._realtime_control_data_in_xyzYaw[2] = round(vel_z)
            self._realtime_control_data_in_xyzYaw[3] = round(yaw)
            self.fc.send_realtime_control_data(
                *self._realtime_control_data_in_xyzYaw
            )

    def _stop_velocity_override(
        self,
        restore_hover: bool = False,
        hover_height: Optional[float] = None,
        zero_flush_frames: int = VELOCITY_OVERRIDE_ZERO_FLUSH_FRAMES,
    ) -> bool:
        """退出速度接管并清零控制；可在条件允许时恢复当前位置悬停。"""
        zero_flush_frames = int(zero_flush_frames)
        if zero_flush_frames < 1:
            raise ValueError("zero_flush_frames must be at least 1")
        with self._control_lock:
            self._velocity_override_active = True
            self._velocity_override_faulted = True
            self._velocity_override_horizontal_cancelled = True
            self.navigation_flag = False
            self.keep_height_flag = False
            self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]

        for frame_index in range(zero_flush_frames):
            with self._control_lock:
                try:
                    self.fc.send_realtime_control_data(0, 0, 0, 0)
                except Exception:
                    logger.exception("[NAVI] Failed to clear velocity override control")
                    return False
            if frame_index + 1 < zero_flush_frames:
                time.sleep(VELOCITY_OVERRIDE_ZERO_FLUSH_INTERVAL)

        with self._control_lock:
            self._velocity_override_active = False
            self._velocity_override_keep_height = False
            self._velocity_override_faulted = False
            self._velocity_override_horizontal_cancelled = False

        if not restore_hover:
            logger.info("[NAVI] Velocity override stopped")
            return True
        if not (
            self.running
            and self.fc.state.unlock.value
            and self._flight_state_is_fresh()
            and self.pose_is_fresh()
            and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
        ):
            logger.error("[NAVI] Cannot restore hover after velocity override")
            return False

        target_height = (
            float(self.fc.state.alt_add.value)
            if hover_height is None
            else float(hover_height)
        )
        self.direct_set_waypoint([self.current_x, self.current_y])
        self.set_height(target_height)
        self.switch_pid("hover")
        self.navigation_flag = True
        self.keep_height_flag = True
        logger.info(
            f"[NAVI] Velocity override stopped; hovering at {target_height}cm"
        )
        return True

    def moving_descent(
        self,
        horizontal_speed: float,
        target_height: float,
        direction_deg: float = 0,
        descent_speed: float = 15,
        height_confirm_time: float = 0.1,
        timeout: float = 15,
    ) -> bool:
        """
        保持水平移动并下降到指定激光高度，成功后恢复当前位置悬停。

        horizontal_speed: 水平速度 / cm/s，取绝对值
        target_height: 目标激光高度 / cm
        direction_deg: 导航坐标系方向 / deg，0 度为 x 正方向，逆时针为正
        descent_speed: 下降速度 / cm/s，取绝对值
        height_confirm_time: 高度不高于目标值的持续确认时间 / s
        timeout: 整个下降阶段超时 / s
        """
        return self._moving_descent(
            horizontal_speed=horizontal_speed,
            target_height=target_height,
            direction_deg=direction_deg,
            descent_speed=descent_speed,
            height_confirm_time=height_confirm_time,
            timeout=timeout,
            hold_after_reaching=True,
        )

    def _moving_descent(
        self,
        horizontal_speed: float,
        target_height: float,
        direction_deg: float = 0,
        descent_speed: float = 15,
        height_confirm_time: float = 0.1,
        timeout: float = 15,
        hold_after_reaching: bool = True,
    ) -> bool:
        """
        执行移动下降；可为 moving_landing 保持速度接管以连续进入接地阶段。

        horizontal_speed: 水平速度 / cm/s，取绝对值
        target_height: 目标激光高度 / cm
        direction_deg: 导航坐标系方向 / deg，0 度为 x 正方向，逆时针为正
        descent_speed: 下降速度 / cm/s，取绝对值
        height_confirm_time: 高度不高于目标值的持续确认时间 / s
        timeout: 整个下降阶段超时 / s
        hold_after_reaching: 成功后是否恢复悬停；False 仅供内部连续控制
        """
        values = np.asarray(
            [
                horizontal_speed,
                target_height,
                direction_deg,
                descent_speed,
                height_confirm_time,
                timeout,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("moving_descent parameters must be finite")
        if not isinstance(hold_after_reaching, bool):
            raise ValueError("hold_after_reaching must be bool")
        horizontal_speed = abs(float(horizontal_speed))
        target_height = float(target_height)
        direction_deg = float(direction_deg)
        descent_speed = abs(float(descent_speed))
        self._validate_moving_descent_speeds(
            horizontal_speed=horizontal_speed,
            descent_speed=descent_speed,
        )
        confirm_time = float(height_confirm_time)
        timeout = float(timeout)
        if target_height < 0:
            raise ValueError("target_height must not be negative")
        if min(descent_speed, confirm_time, timeout) <= 0:
            raise ValueError(
                "descent_speed, height_confirm_time and timeout must be greater than 0"
            )
        if not self._flight_state_is_fresh():
            logger.error("[NAVI] Moving descent refused: FC telemetry is stale")
            return False
        if float(self.fc.state.alt_add.value) < target_height:
            logger.error(
                "[NAVI] Moving descent refused: target height is above current height"
            )
            return False
        if not self._start_velocity_override(keep_height=False, require_pose=True):
            return False

        direction_rad = np.deg2rad(direction_deg)
        vel_x_world = horizontal_speed * float(np.cos(direction_rad))
        vel_y_world = horizontal_speed * float(np.sin(direction_rad))
        start_time = time.perf_counter()
        reached_since = None
        reached = False
        abort_reason = None
        stop_ok = True
        pose_lost_in_target_zone = False
        logger.info(
            f"[NAVI] Moving descent started: speed={horizontal_speed}cm/s, "
            f"direction={direction_deg}deg, target={target_height}cm, "
            f"descent={descent_speed}cm/s"
        )
        try:
            while True:
                now = time.perf_counter()
                if self.stop_event is not None and self.stop_event.is_set():
                    abort_reason = "external stop event is set"
                    break
                if not self.running:
                    abort_reason = "navigation stopped"
                    break
                if not self._flight_state_is_fresh():
                    abort_reason = "flight-controller telemetry became stale"
                    break
                if not self.fc.state.unlock.value:
                    abort_reason = "aircraft became locked before target height"
                    break
                if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
                    abort_reason = "flight mode left HOLD_POS"
                    break
                if now - start_time > timeout:
                    abort_reason = "moving descent timeout"
                    break

                alt_now = float(self.fc.state.alt_add.value)
                pose_fresh = self.pose_is_fresh()
                if not pose_fresh:
                    if hold_after_reaching or alt_now > target_height:
                        abort_reason = "navigation pose became stale"
                        break
                    if not pose_lost_in_target_zone:
                        pose_lost_in_target_zone = True
                        with self._control_lock:
                            self._velocity_override_horizontal_cancelled = True
                        logger.warning(
                            "[NAVI] Pose stale in target-height zone; "
                            "cancel horizontal velocity and continue descent"
                        )
                if alt_now <= target_height:
                    if reached_since is None:
                        reached_since = now
                    elif now - reached_since >= confirm_time:
                        reached = True
                        break
                    vel_z = 0.0 if hold_after_reaching else -descent_speed
                else:
                    reached_since = None
                    vel_z = -descent_speed
                self._update_velocity_override(
                    vel_x=(
                        vel_x_world
                        if pose_fresh and not pose_lost_in_target_zone
                        else 0
                    ),
                    vel_y=(
                        vel_y_world
                        if pose_fresh and not pose_lost_in_target_zone
                        else 0
                    ),
                    vel_z=vel_z,
                    yaw=0,
                    frame="world",
                )
                time.sleep(0.05)
        except Exception:
            logger.exception("[NAVI] Moving descent control error")
            abort_reason = "control exception"
        finally:
            if not reached or hold_after_reaching:
                stop_ok = self._stop_velocity_override(
                    restore_hover=hold_after_reaching if reached else True,
                    hover_height=target_height if reached else None,
                )

        if abort_reason is not None:
            logger.error(f"[NAVI] Moving descent aborted: {abort_reason}")
            return False
        if not reached:
            return False
        if not stop_ok:
            logger.error("[NAVI] Moving descent reached target but control was not cleared")
            return False
        logger.info(f"[NAVI] Moving descent reached {target_height}cm")
        return True

    def moving_landing(
        self,
        horizontal_speed: float,
        direction_deg: float = 0,
        lock_after_landing: bool = True,
        descent_speed: float = 15,
        touchdown_alt_thres: float = 12,
        touchdown_confirm_time: float = 0.3,
        touchdown_timeout: float = 15,
        lock_timeout: float = 4,
        touchdown_vertical_speed_thres: float = 5,
    ) -> bool:
        """
        保持水平移动下降至接地区域，组合确认接地后可选择是否锁桨。

        horizontal_speed: 水平速度 / cm/s，取绝对值
        direction_deg: 导航坐标系方向 / deg，0 度为 x 正方向，逆时针为正
        lock_after_landing: True 为确认接地后锁桨；False 为清零后保持解锁
        descent_speed: 下降速度 / cm/s，取绝对值
        touchdown_alt_thres: 接地高度阈值 / cm，限制为不大于 30 cm
        touchdown_vertical_speed_thres: 接地垂直速度绝对值阈值 / cm/s

        接地条件为激光高度不高于 touchdown_alt_thres，且垂直速度绝对值
        不高于 touchdown_vertical_speed_thres，并持续 touchdown_confirm_time。
        touchdown_timeout 覆盖移动下降和最终接地两个阶段。
        """
        values = np.asarray(
            [
                horizontal_speed,
                direction_deg,
                descent_speed,
                touchdown_alt_thres,
                touchdown_confirm_time,
                touchdown_timeout,
                lock_timeout,
                touchdown_vertical_speed_thres,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("moving_landing parameters must be finite")
        if not isinstance(lock_after_landing, bool):
            raise ValueError("lock_after_landing must be bool")
        horizontal_speed = abs(float(horizontal_speed))
        direction_deg = float(direction_deg)
        descent_speed = abs(float(descent_speed))
        self._validate_moving_descent_speeds(
            horizontal_speed=horizontal_speed,
            descent_speed=descent_speed,
        )
        requested_alt_thres = float(touchdown_alt_thres)
        if requested_alt_thres > 30:
            raise ValueError("touchdown_alt_thres must not exceed 30cm")
        alt_thres = max(3.0, requested_alt_thres)
        confirm_time = float(touchdown_confirm_time)
        timeout = float(touchdown_timeout)
        lock_timeout = float(lock_timeout)
        vertical_speed_thres = abs(float(touchdown_vertical_speed_thres))
        if vertical_speed_thres > TOUCHDOWN_MAX_VERTICAL_SPEED_THRESHOLD:
            raise ValueError(
                "touchdown_vertical_speed_thres must not exceed "
                f"{TOUCHDOWN_MAX_VERTICAL_SPEED_THRESHOLD}cm/s"
            )
        if min(descent_speed, confirm_time, timeout, vertical_speed_thres) <= 0:
            raise ValueError(
                "descent_speed, touchdown_confirm_time, touchdown_timeout, "
                "and touchdown_vertical_speed_thres must be greater than 0"
            )
        if lock_after_landing and lock_timeout <= 0:
            raise ValueError("lock_timeout must be greater than 0 when locking")

        deadline = time.perf_counter() + timeout
        already_in_touchdown_zone = bool(
            self._flight_state_is_fresh()
            and float(self.fc.state.alt_add.value) <= alt_thres
        )
        if already_in_touchdown_zone:
            if not self._start_velocity_override(
                keep_height=False,
                require_pose=False,
            ):
                return False
            if not self.pose_is_fresh():
                with self._control_lock:
                    self._velocity_override_horizontal_cancelled = True
                logger.warning(
                    "[NAVI] Starting in touchdown zone without fresh pose; "
                    "horizontal velocity is cancelled"
                )
        elif not self._moving_descent(
            horizontal_speed=horizontal_speed,
            target_height=alt_thres,
            direction_deg=direction_deg,
            descent_speed=descent_speed,
            height_confirm_time=0.1,
            timeout=timeout,
            hold_after_reaching=False,
        ):
            return False

        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            logger.error("[NAVI] Moving landing aborted: touchdown timeout")
            self._stop_velocity_override(restore_hover=True)
            return False
        with self._control_lock:
            override_active = self._velocity_override_active
        if not override_active:
            logger.error("[NAVI] Moving landing lost velocity override after approach")
            return False

        direction_rad = np.deg2rad(direction_deg)
        vel_x_world = horizontal_speed * float(np.cos(direction_rad))
        vel_y_world = horizontal_speed * float(np.sin(direction_rad))
        touchdown_since = None
        landed = False
        abort_reason = None
        control_cleared = False
        with self._control_lock:
            pose_lost_in_touchdown_zone = (
                self._velocity_override_horizontal_cancelled
            )
        logger.info(
            f"[NAVI] Moving landing final descent: altitude<={alt_thres}cm, "
            f"|vel_z|<={vertical_speed_thres}cm/s, lock={lock_after_landing}"
        )
        try:
            while True:
                now = time.perf_counter()
                if self.stop_event is not None and self.stop_event.is_set():
                    abort_reason = "external stop event is set"
                    break
                if not self.running:
                    abort_reason = "navigation stopped"
                    break
                if not self._flight_state_is_fresh():
                    abort_reason = "flight-controller telemetry became stale"
                    break
                if not self.fc.state.unlock.value:
                    landed = True
                    break
                if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
                    abort_reason = "flight mode left HOLD_POS"
                    break
                if now >= deadline:
                    abort_reason = "touchdown timeout"
                    break

                alt_now = float(self.fc.state.alt_add.value)
                vertical_speed_now = abs(float(self.fc.state.vel_z.value))
                pose_fresh = self.pose_is_fresh()
                if not pose_fresh:
                    if alt_now > alt_thres:
                        abort_reason = "navigation pose became stale"
                        break
                    if not pose_lost_in_touchdown_zone:
                        pose_lost_in_touchdown_zone = True
                        with self._control_lock:
                            self._velocity_override_horizontal_cancelled = True
                        logger.warning(
                            "[NAVI] Pose stale in touchdown zone; "
                            "cancel horizontal velocity and continue vertical landing"
                        )
                touchdown_candidate = bool(
                    alt_now <= alt_thres
                    and vertical_speed_now <= vertical_speed_thres
                )
                if touchdown_candidate:
                    if touchdown_since is None:
                        touchdown_since = now
                    elif now - touchdown_since >= confirm_time:
                        landed = True
                        break
                else:
                    touchdown_since = None

                self._update_velocity_override(
                    vel_x=(
                        vel_x_world
                        if (
                            pose_fresh
                            and not pose_lost_in_touchdown_zone
                            and not touchdown_candidate
                        )
                        else 0
                    ),
                    vel_y=(
                        vel_y_world
                        if (
                            pose_fresh
                            and not pose_lost_in_touchdown_zone
                            and not touchdown_candidate
                        )
                        else 0
                    ),
                    vel_z=-descent_speed,
                    yaw=0,
                    frame="world",
                )
                time.sleep(0.05)
        except Exception:
            logger.exception("[NAVI] Moving landing control error")
            abort_reason = "control exception"
        finally:
            control_cleared = self._stop_velocity_override(
                restore_hover=abort_reason is not None,
                zero_flush_frames=(
                    1
                    if landed and lock_after_landing
                    else VELOCITY_OVERRIDE_ZERO_FLUSH_FRAMES
                ),
            )

        if abort_reason is not None:
            logger.error(f"[NAVI] Moving landing aborted: {abort_reason}")
            return False
        if not landed:
            return False

        logger.info("[NAVI] Moving landing touchdown confirmed")
        if not lock_after_landing:
            if not control_cleared:
                logger.error(
                    "[NAVI] Moving landing cannot finish unlocked because "
                    "zero control was not confirmed"
                )
                return False
            logger.warning("[NAVI] Moving landing finished without locking motors")
            return True
        if not self.fc.state.unlock.value:
            if not control_cleared and not self._stop_velocity_override():
                logger.error(
                    "[NAVI] Motors locked but velocity override cleanup failed"
                )
                return False
            logger.info("[NAVI] Motors already locked after touchdown")
            return True
        if not self._flight_state_is_fresh():
            logger.error("[NAVI] Refuse lock because touchdown telemetry is stale")
            return False
        if (
            float(self.fc.state.alt_add.value) > alt_thres
            or abs(float(self.fc.state.vel_z.value)) > vertical_speed_thres
        ):
            logger.error("[NAVI] Refuse lock because touchdown is not confirmed")
            return False

        self.fc.lock()
        try:
            locked = self.fc.wait_for_lock(timeout_s=lock_timeout)
        except TypeError:
            locked = self.fc.wait_for_lock(lock_timeout)
        if not locked:
            logger.error("[NAVI] Moving landing lock was not confirmed")
            return False
        if not control_cleared and not self._stop_velocity_override():
            logger.error("[NAVI] Motors locked but velocity override cleanup failed")
            return False
        logger.info("[NAVI] Moving landing finished and motors locked")
        return True

    def _waypoint_param_switch(self):
        tuning = self.pid_tunings["hover"]
        self.navi_x_pid.tunings = tuning
        self.navi_y_pid.tunings = tuning
        self.navi_x_pid.output_limits = (-self.navi_speed, self.navi_speed)
        self.navi_y_pid.output_limits = (-self.navi_speed, self.navi_speed)
        logger.debug("[NAVI] Waypoint param switched")

    def wait_for_waypoint(self, time_thres=2, pos_thres=20, timeout=15):
        """
        等待到达目标点

        time_thres: 到达目标点后积累的时间/s
        pos_thres: 到达目标点的距离阈值/cm
        timeout: 超时时间/s
        """
        time_count = 0
        time_start = time.perf_counter()
        param_switched = False
        while True:
            time.sleep(0.05)
            if self.stop_event is not None and self.stop_event.is_set():
                logger.warning("[NAVI] Waypoint wait stopped by external stop event")
                return False
            if self._reached_waypoint(pos_thres):
                time_count += 0.05
                if not param_switched:
                    self._waypoint_param_switch()
                    param_switched = True
            else:
                time_count = 0
            if not self.running or not self.pose_is_fresh():
                time_count = 0
            if time_count >= time_thres:
                logger.info("[NAVI] Reached waypoint")
                return True
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Waypoint overtime")
                return False

    def wait_for_height(self, time_thres=0.5, height_thres=8, timeout=10):
        """
        等待到达目标高度(定高设定值)

        time_thres: 到达目标高度后积累的时间/s
        pos_thres: 到达目标高度的阈值/cm
        timeout: 超时时间/s
        """
        time_start = time.perf_counter()
        time_count = 0
        while True:
            time.sleep(0.05)
            if self.stop_event is not None and self.stop_event.is_set():
                logger.warning("[NAVI] Height wait stopped by external stop event")
                return False
            if abs(self.current_height - self.height_pid.setpoint) < height_thres:
                time_count += 0.05
            else:
                time_count = 0
            state_fresh = bool(
                getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
            )
            if not self.running or not state_fresh:
                time_count = 0
            if time_count >= time_thres:
                logger.info("[NAVI] Reached height")
                return True
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Height overtime")
                return False

    def wait_for_yaw(self, time_thres=0.5, yaw_thres=5, timeout=10):
        """
        等待到达目标偏航角

        time_thres: 到达目标偏航角后积累的时间/s
        pos_thres: 到达目标偏航角的阈值/deg
        timeout: 超时时间/s
        """
        time_start = time.perf_counter()
        time_count = 0
        while True:
            time.sleep(0.05)
            if self.stop_event is not None and self.stop_event.is_set():
                logger.warning("[NAVI] Yaw wait stopped by external stop event")
                return False
            yaw_error = abs(
                _shortest_yaw_error(
                    self.yaw_target, self.current_yaw, self._yaw_direction_hint
                )
            )
            if yaw_error < yaw_thres:
                time_count += 0.05
            else:
                time_count = 0
            if not self.running or not self.pose_is_fresh():
                time_count = 0
            if time_count >= time_thres:
                logger.info("[NAVI] Reached yaw")
                return True
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Yaw overtime")
                return False
    def radar_find_target(self,TARGET_NUM):
        self.radar.get_target_points(TARGET_NUM)

    def pointing_takeoff(self, point, target_height=140):
        """
        定点起飞

        point: (x, y) 坐标 / cm / 匿名(ROS)坐标系 / 基地点
        target_height: 起飞高度 / cm
        """
        logger.info(f"[NAVI] Takeoff at {point}")
        self.navigation_flag = False
        self.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        # inital_yaw = self.fc.state.yaw.value
        time.sleep(2)  # 等待电机启动
        self.fc.take_off(30)
        self.fc.wait_for_takeoff_done(timeout_s=5)
        # self.fc.set_yaw(inital_yaw, 25)
        # self.fc.wait_for_hovering(2)
        ######## 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.set_height(70)
        self.keep_height_flag = True
        self.wait_for_height()
        self.navigation_flag = True
        self.navigation_to_waypoint(point, wait=True)  # 初始化路径点
        time.sleep(0.5)
        self.set_height(target_height)
        self.wait_for_height()
        self.navigation_to_waypoint(point, wait=True)  # 初始化路径点
        self.switch_pid("hover")
        time.sleep(0.1)
        self.navigation_flag = True

        # 原 pointing_takeoff 实现保留如下，仅注释停用：
        # def pointing_takeoff(
        #     self,
        #     point,
        #     target_height=140,
        #     first_lift=60,
        #     lock_pos_thres=15,
        #     lock_pos_time=1.0,
        #     lock_timeout=12,
        #     hover_timeout=12,
        #     height_timeout=15,
        # ):
        #     """
        #     Take off and then enter closed-loop hold/navigation.
        #
        #     A takeoff command is sent only once and only after fresh mode/unlock
        #     feedback.  Ambiguous feedback is treated as a failure, not as a reason
        #     to issue another takeoff command while the aircraft may already be airborne.
        #     """
        #     logger.info(f"[NAVI] Takeoff at {point}")
        #
        #     # 1) Keep navigation loops disabled during raw FC takeoff stage.
        #     self.navigation_flag = False
        #     self.keep_height_flag = False
        #
        #     # 2) PROGRAM mode + unlock.
        #     self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        #     if not self.fc.state.unlock.value:
        #         self.fc.unlock()
        #
        #     # Wait for fresh unlock feedback before sending the one-key takeoff.
        #     t0 = time.perf_counter()
        #     while time.perf_counter() - t0 < 3.0:
        #         state_fresh = bool(
        #             getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
        #         )
        #         if (
        #             state_fresh
        #             and self.fc.state.mode.value == self.fc.PROGRAM_MODE
        #             and self.fc.state.unlock.value
        #         ):
        #             break
        #         time.sleep(0.05)
        #     else:
        #         raise RuntimeError("[NAVI] Fresh PROGRAM/unlock feedback not confirmed")
        #
        #     time.sleep(0.8)  # Buffer for motor/state updates.
        #     lift = int(max(40, first_lift))
        #
        #     try:
        #         alt_before = float(self.fc.state.alt_add.value)
        #     except Exception:
        #         alt_before = 0.0
        #
        #     def _wait_takeoff_done(timeout_s: float) -> bool:
        #         try:
        #             return bool(self.fc.wait_for_takeoff_done(timeout_s=timeout_s))
        #         except TypeError:
        #             # Backward compatibility for older signatures.
        #             return bool(self.fc.wait_for_takeoff_done(4, timeout_s))
        #
        #     def _current_alt() -> float:
        #         try:
        #             return float(self.fc.state.alt_add.value)
        #         except Exception:
        #             return 0.0
        #
        #     self.fc.take_off(lift)
        #     time.sleep(0.8)  # Give command_now / vel_z time to update.
        #     ok = _wait_takeoff_done(timeout_s=8)
        #     alt_now = _current_alt()
        #     state_fresh = bool(
        #         getattr(self.fc.state, "is_fresh", lambda _age: False)(0.5)
        #     )
        #     takeoff_started = bool(
        #         state_fresh and (ok or alt_now >= 10 or (alt_now - alt_before) >= 5)
        #     )
        #     if not takeoff_started:
        #         raise RuntimeError(
        #             "[NAVI] Takeoff was not confirmed by fresh telemetry; "
        #             "command will not be retried"
        #         )
        #
        #     # Ensure FC reports hovering before enabling closed-loop hold.
        #     if not self.fc.wait_for_hovering(hover_timeout):
        #         raise RuntimeError("[NAVI] Hovering was not confirmed after takeoff")
        #
        #     # 3) Switch to HOLD_POS and enable closed-loop control.
        #     self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        #     time.sleep(0.1)
        #
        #     try:
        #         h_now = float(self.fc.state.alt_add.value)
        #     except Exception:
        #         h_now = float(lift)
        #     self.set_height(max(h_now, float(lift)))
        #     self.keep_height_flag = True
        #
        #     self.switch_pid("hover")
        #     self.direct_set_waypoint([float(point[0]), float(point[1])])
        #     self.navigation_flag = True
        #
        #     if not self.wait_for_waypoint(
        #         time_thres=lock_pos_time,
        #         pos_thres=lock_pos_thres,
        #         timeout=lock_timeout,
        #     ):
        #         raise RuntimeError(
        #             "[NAVI] Position hold was not confirmed after takeoff"
        #         )
        #
        #     self.set_height(float(target_height))
        #     if not self.wait_for_height(timeout=height_timeout):
        #         raise RuntimeError(
        #             "[NAVI] Cruise height was not confirmed after takeoff"
        #         )

    def move_by_direction(self, speed: float = 5, direction_deg: float = 0):
        """
        以给定速度沿给定方向移动（临时覆盖导航控制）

        speed: 速度 / cm/s (默认5，适合精调)
        direction_deg: 方向角度 / deg，0度为x轴正方向，逆时针为正
        """
        self.navigation_flag = False  # 关闭水平导航PID
        rad = np.deg2rad(direction_deg)
        vel_x = int(speed * np.cos(rad))
        vel_y = int(speed * np.sin(rad))
        self.update_realtime_control(vel_x=vel_x, vel_y=vel_y)
        logger.info(f"[NAVI] Move by direction: speed={speed}, dir={direction_deg}°, vel=({vel_x},{vel_y})")

    def stop_move(self):
        """
        停止手动移动，重新开启导航并悬停在当前位置
        """
        self.update_realtime_control(vel_x=0, vel_y=0)
        # 设置目标点为当前位置
        self.navi_x_pid.setpoint = self.current_x
        self.navi_y_pid.setpoint = self.current_y
        self.navigation_flag = True  # 重新开启水平导航PID
        logger.info("[NAVI] Stop move, hover at current position")


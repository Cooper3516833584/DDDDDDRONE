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

logger_dbg = logger.bind(debug=True)


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
        self.running = False
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
        self.switch_navigation_mode(mode)  # type: ignore
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self.update_realtime_control(vel_x=0, vel_y=0, vel_z=0, yaw=0)
        logger.info("[NAVI] Realtime control started")
        self._thread_list.append(threading.Thread(target=self._keep_height_task, daemon=True))
        self._thread_list[-1].start()
        self._thread_list.append(threading.Thread(target=self._navigation_task, daemon=True))
        self._thread_list[-1].start()
        logger.info("[NAVI] Navigation started")

    def update_realtime_control(
        self,
        vel_x: Optional[int] = None,
        vel_y: Optional[int] = None,
        vel_z: Optional[int] = None,
        yaw: Optional[int] = None,
    ) -> None:
        """
        更新实时控制帧
        """
        if vel_x is not None:
            self._realtime_control_data_in_xyzYaw[0] = vel_x
        if vel_y is not None:
            self._realtime_control_data_in_xyzYaw[1] = vel_y
        if vel_z is not None:
            self._realtime_control_data_in_xyzYaw[2] = vel_z
        if yaw is not None:
            self._realtime_control_data_in_xyzYaw[3] = yaw
        self.fc.send_realtime_control_data(*self._realtime_control_data_in_xyzYaw)

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
        vel_z = round(pose.velocity.x * 100)
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
                        self.update_realtime_control(vel_z=0)
                        continue
                    self.fc.state.update_event.clear()
                    self.current_height = self.fc.state.alt_add.value
                    height = self.current_height
                else:
                    if not self.rs.update_event.wait(1):
                        logger.warning("[NAVI] RealSense height timeout")
                        self.update_realtime_control(vel_z=0)
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
                        self.update_realtime_control(vel_z=0)
                        logger.info("[NAVI] Keep height paused")
                    continue
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[NAVI] Keep Height resumed")
                out_hei = round(self.height_pid(height))  # type: ignore
                self.update_realtime_control(vel_z=out_hei)
                logger_dbg.info(f"[NAVI] Height PID output: {out_hei}")
            except Exception as e:
                logger.exception("[NAVI] Keep height task error")
                self.update_realtime_control(vel_z=0)

    def _get_t265_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
        if wait and not self.rs.update_event.wait(1):
            logger.warning("[NAVI] RealSense pose timeout")
            return None
        self.rs.update_event.clear()
        if self._t265_trans_args is None:
            current_x = -self.rs.pose.translation.z * 100
            current_y = -self.rs.pose.translation.x * 100
            self.current_height_rs = self.rs.pose.translation.y * 100
            current_yaw = -self.rs.eular_rotation[2]
        else:
            position, eular = self.rs.get_pose_in_secondary_frame(self._t265_trans_args, as_eular=True)
            current_x = -position[2] * 100  # type: ignore
            current_y = -position[0] * 100  # type: ignore
            self.current_height_rs = position[1] * 100  # type: ignore
            current_yaw = -eular[2]  # type: ignore
        available = self.rs.pose.tracker_confidence >= 2
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
        if self.mapper.trans_update_event.is_set():  # type: ignore
            self.mapper.trans_update_event.clear()  # type: ignore
            self.calibrate_realsense_ros(wait=False)
        ret = self._get_t265_pose()
        if not ret:
            return None
        x, y, yaw, avai = ret
        return x, y, yaw, avai and self.mapper._trans_node.transform_established  # type: ignore

    def _navigation_task(self):
        paused = False
        while self.running:
            try:
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
                    self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
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
                        self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
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
                    time.sleep(0.1)
                    continue
                # self.fc.send_general_position(x=self.current_x, y=self.current_y)
                out_x = round(self.navi_x_pid(self.current_x))  # type: ignore
                if out_x is not None:
                    self.update_realtime_control(vel_x=out_x)
                out_y = round(self.navi_y_pid(self.current_y))  # type: ignore
                if out_y is not None:
                    self.update_realtime_control(vel_y=out_y)
                out_yaw = round(self.yaw_pid(self.current_yaw))  # type: ignore
                if out_yaw is not None:
                    self.update_realtime_control(yaw=out_yaw)
                logger_dbg.info(f"[NAVI] Pose PID output: {out_x}, {out_y}, {out_yaw}")
            except Exception as e:
                logger.exception(f"[NAVI] Navigation task error")
                self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)

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

    def calibrate_realsense_ros(self, wait=True):
        """
        根据ROS建图数据校准T265的坐标系
        """
        if wait and not self.mapper.trans_update_event.wait(1):  # type: ignore
            raise RuntimeError("Mapper transform update timeout")
        x, y, _ = self.mapper.position  # type: ignore
        _, _, yaw = self.mapper.eular_rotation  # type: ignore
        dx = -x
        dy = -y
        dyaw = yaw
        if not self.keep_height_by_rs:
            dz = self.fc.state.alt_add.value / 100.0
        else:
            dz = self.current_height_rs / 100.0
        logger_dbg.info(f"[NAVI] Calibrate T265: map={self.mapper.position} dz={dx}, dx={dy}, dy={dz}, dyaw={dyaw}")  # type: ignore
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
        tT = float(length / self.navi_speed)
        traj = TrajectoryGenerator(start_pos=waypoint_cur, des_pos=waypoint, T=tT)
        traj.solve()
        traj_list = []
        for t in np.arange(0, tT, dt):
            traj_list.append(traj.calc_position_xyz(t))
        traj_list.append(waypoint)
        self.navigation_follow_trajectory(traj_list, wait=wait)  # type: ignore

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
        self.traj_running_event.set()

        pos_thres = float(max(pos_thres, 1.0))
        th2 = pos_thres * pos_thres

        len_t = len(traj_list)

        for n, point in enumerate(traj_list):
            if not (self.running and self.navigation_flag):
                logger.debug("[NAVI] Trajectory task forced to stop (nav not running)")
                return

            # 外部请求停止：clear event
            if not self.traj_running_event.is_set():
                self.traj_running_event.set()
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
                    self.traj_running_event.set()
                    logger.debug("[NAVI] Trajectory task forced to stop")
                    self.traj_list_before_stop = traj_list[n:]
                    return

                if not (self.running and self.navigation_flag):
                    logger.debug("[NAVI] Trajectory task forced to stop (nav not running)")
                    return

                dx = float(self.current_x) - x
                dy = float(self.current_y) - y
                if dx * dx + dy * dy <= th2:
                    break

                if timeout_per_point > 0 and (time.perf_counter() - t0) > timeout_per_point:
                    logger.warning("[NAVI] Trajectory point timeout, skipping to next point")
                    break

        self.traj_running_event.clear()
        logger.debug("[NAVI] Trajectory task finished")

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
            self._trajectory_task(traj_list, pos_thres=pos_thres)
            # 最后一段再确认一次到点（用更小阈值更贴轨）
            self.wait_for_waypoint(time_thres=0.5, pos_thres=max(8, int(pos_thres)), timeout=10)
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
        self.yaw_pid.setpoint = yaw
        logger.debug(f"[NAVI] Keep yaw set to {yaw}")

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

    def pointing_takeoff(
            self,
            point,
            target_height=140,
            first_lift=60,
            lock_pos_thres=15,
            lock_pos_time=1.0,
            lock_timeout=12,
            hover_timeout=12,
            height_timeout=15,
    ):
        """
        定点起飞（修复版）
        - 修复问题：在地面时 self.fc.hovering 可能本来就为 True，导致 wait_for_hovering() 立即返回，
          随后切模式/开导航会把 take_off 指令“打断”，表现为电机不转、飞机不抬。
        - 做法：take_off 后先等待“起飞确实开始/完成”（vel_z/alt_add 变化），再等待进入悬停稳定。
        """
        logger.info(f"[NAVI] Takeoff at {point}")

        # 1) 起飞阶段先关掉闭环，避免线程在不合适的模式下干预
        self.navigation_flag = False
        self.keep_height_flag = False

        # 2) 程控模式 + 解锁
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        if not self.fc.state.unlock.value:
            self.fc.unlock()

        # 等解锁状态回传（有些设备回传慢）
        t0 = time.perf_counter()
        while not self.fc.state.unlock.value and (time.perf_counter() - t0) < 3.0:
            time.sleep(0.05)

        time.sleep(0.8)  # 给电机/状态一个缓冲时间

        # 3) 一键起飞抬离地面
        lift = int(max(40, first_lift))
        self.fc.take_off(lift)

        # 关键：等待起飞“确实开始/完成”，避免 hover 状态误判导致立即切模式打断起飞
        time.sleep(0.8)  # 让 command_now / vel_z 有时间更新
        ok = False
        try:
            ok = self.fc.wait_for_takeoff_done(timeout_s=8)
        except TypeError:
            # 兼容旧版接口
            ok = self.fc.wait_for_takeoff_done(4, 8)

        # 兜底：如果 vel_z 阈值没触发，但高度已经抬起来，也算起飞成功
        try:
            alt_now = float(self.fc.state.alt_add.value)
        except Exception:
            alt_now = 0.0
        if (not ok) and alt_now < 10:
            raise RuntimeError("[NAVI] Takeoff did not start (alt_add < 10cm). Check unlock/mode/propellers/FC safety.")

        # 起飞完成后等待进入悬停稳定（此时 hovering 的判断才有意义）
        self.fc.wait_for_hovering(hover_timeout)

        # 4) 切到定点模式，准备开启“水平锁点 + 定高”
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        time.sleep(0.1)

        # 先把当前高度作为定高起点，避免切模式瞬间高度漂
        try:
            h_now = float(self.fc.state.alt_add.value)
        except Exception:
            h_now = float(lift)
        self.set_height(max(h_now, float(lift)))
        self.keep_height_flag = True

        # 5) 立刻锁点到目标 point（只取 x,y；高度由 set_height 控制）
        self.switch_pid("hover")
        self.direct_set_waypoint([float(point[0]), float(point[1])])
        self.navigation_flag = True

        # 低高度先把位置锁回 point，再爬高（解决“先飘很久后才回原点”）
        self.wait_for_waypoint(
            time_thres=lock_pos_time,
            pos_thres=lock_pos_thres,
            timeout=lock_timeout,
        )

        # 6) 在锁点状态下爬升到目标高度（原地竖直爬升）
        self.set_height(float(target_height))
        self.wait_for_height(timeout=height_timeout)

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
        self.wait_for_waypoint(
            time_thres=max(0.3, float(settle_time_thres)),
            pos_thres=max(8, int(approach_pos_thres)),
            timeout=max(1.0, float(settle_timeout)),
        )

        self.set_height(float(max(10, approach_height)))
        self.wait_for_height(time_thres=0.3, height_thres=10, timeout=max(1.0, float(height_timeout)))
        self.direct_set_waypoint([x, y])
        self.wait_for_waypoint(
            time_thres=0.3,
            pos_thres=max(8, int(approach_pos_thres)),
            timeout=max(1.0, float(settle_timeout)),
        )

        # 阶段2: 关闭导航闭环，切给飞控一键降落，避免PID慢速“磨地”
        self.navigation_flag = False
        self.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        time.sleep(0.1)
        self.fc.stablize()
        self.fc.land()

        # 等待落地(高度足够低或飞控已自动上锁)
        t0 = time.perf_counter()
        landed = False
        alt_thres = float(max(3, touchdown_alt_thres))
        while time.perf_counter() - t0 < max(1.0, float(touchdown_timeout)):
            time.sleep(0.1)
            try:
                alt_now = float(self.fc.state.alt_add.value)
            except Exception:
                alt_now = 999.0
            if alt_now <= alt_thres or (not self.fc.state.unlock.value):
                landed = True
                break

        if not landed:
            logger.warning("[NAVI] Landing timeout, force lock")
            self.fc.lock()
            return

        try:
            ok = self.fc.wait_for_lock(timeout_s=lock_timeout)
        except TypeError:
            ok = self.fc.wait_for_lock(lock_timeout)
        if not ok:
            self.fc.lock()

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
            if self._reached_waypoint(pos_thres):
                time_count += 0.05
                if not param_switched:
                    self._waypoint_param_switch()
                    param_switched = True
            if time_count >= time_thres:
                logger.info("[NAVI] Reached waypoint")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Waypoint overtime")
                return

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
            if abs(self.current_height - self.height_pid.setpoint) < height_thres:
                time_count += 0.05
            if time_count >= time_thres:
                logger.info("[NAVI] Reached height")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Height overtime")
                return

    def wait_for_yaw(self, time_thres=0.5, yaw_thres=5, timeout=10):
        """
        等待到达目标偏航角

        time_thres: 到达目标偏航角后积累的时间/s
        pos_thres: 到达目标偏航角的阈值/deg
        timeout: 超时时间/s
        """
        time_start = time.perf_counter()
        time_count = 0
        remap = lambda x: x if x < 180 else 360 - x
        while True:
            time.sleep(0.05)
            if remap(abs(self.current_yaw - self.yaw_pid.setpoint)) < yaw_thres:
                time_count += 0.05
            if time_count >= time_thres:
                logger.info("[NAVI] Reached yaw")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Yaw overtime")
                return
    def radar_find_target(self,TARGET_NUM):
        self.radar.get_target_points(TARGET_NUM)

    def pointing_takeoff(
        self,
        point,
        target_height=140,
        first_lift=60,
        lock_pos_thres=15,
        lock_pos_time=1.0,
        lock_timeout=12,
        hover_timeout=12,
        height_timeout=15,
    ):
        """
        Take off and then enter closed-loop hold/navigation.

        This override is tolerant to FC status-report lag:
        it retries one-key takeoff once and can fallback to `safe_takeoff`
        before treating the attempt as failed.
        """
        logger.info(f"[NAVI] Takeoff at {point}")

        # 1) Keep navigation loops disabled during raw FC takeoff stage.
        self.navigation_flag = False
        self.keep_height_flag = False

        # 2) PROGRAM mode + unlock.
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        if not self.fc.state.unlock.value:
            self.fc.unlock()

        # Wait unlock feedback (some FC firmwares report slowly).
        t0 = time.perf_counter()
        while not self.fc.state.unlock.value and (time.perf_counter() - t0) < 3.0:
            time.sleep(0.05)

        time.sleep(0.8)  # Buffer for motor/state updates.
        lift = int(max(40, first_lift))

        try:
            alt_before = float(self.fc.state.alt_add.value)
        except Exception:
            alt_before = 0.0

        def _wait_takeoff_done(timeout_s: float) -> bool:
            try:
                return bool(self.fc.wait_for_takeoff_done(timeout_s=timeout_s))
            except TypeError:
                # Backward compatibility for older signatures.
                return bool(self.fc.wait_for_takeoff_done(4, timeout_s))

        def _current_alt() -> float:
            try:
                return float(self.fc.state.alt_add.value)
            except Exception:
                return 0.0

        def _takeoff_started(alt_ref: float, alt_now: float, wait_ok: bool) -> bool:
            # Some FCs under-report vel_z, but altitude still rises in reality.
            return bool(wait_ok or alt_now >= 10 or (alt_now - alt_ref) >= 5)

        self.fc.take_off(lift)
        time.sleep(0.8)  # Give command_now / vel_z time to update.
        ok = _wait_takeoff_done(timeout_s=8)
        alt_now = _current_alt()

        # Retry once if first confirmation failed.
        if not _takeoff_started(alt_before, alt_now, ok):
            logger.warning("[NAVI] First takeoff attempt not confirmed, retrying once")
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            if not self.fc.state.unlock.value:
                self.fc.unlock()
                time.sleep(0.5)
            self.fc.take_off(lift)
            time.sleep(1.0)
            ok = _wait_takeoff_done(timeout_s=10)
            alt_now = _current_alt()

        # Last fallback for firmwares with strict one-key timing windows.
        if not _takeoff_started(alt_before, alt_now, ok):
            if hasattr(self.fc, "safe_takeoff"):
                logger.warning("[NAVI] Falling back to FC safe_takeoff")
                try:
                    self.fc.safe_takeoff(
                        target_height=max(int(lift), 80),
                        climb_speed=20,
                        first_lift=lift,
                    )
                except Exception:
                    logger.exception("[NAVI] safe_takeoff fallback failed")
            alt_now = _current_alt()

        if alt_now < 10:
            raise RuntimeError(
                "[NAVI] Takeoff did not start (alt_add < 10cm). Check unlock/mode/propellers/FC safety."
            )

        # Ensure FC reports hovering before enabling closed-loop hold.
        self.fc.wait_for_hovering(hover_timeout)

        # 3) Switch to HOLD_POS and enable closed-loop control.
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        time.sleep(0.1)

        try:
            h_now = float(self.fc.state.alt_add.value)
        except Exception:
            h_now = float(lift)
        self.set_height(max(h_now, float(lift)))
        self.keep_height_flag = True

        self.switch_pid("hover")
        self.direct_set_waypoint([float(point[0]), float(point[1])])
        self.navigation_flag = True

        self.wait_for_waypoint(
            time_thres=lock_pos_time,
            pos_thres=lock_pos_thres,
            timeout=lock_timeout,
        )

        self.set_height(float(target_height))
        self.wait_for_height(timeout=height_timeout)

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


"""
2026 模拟赛 — 空地协同测绘救灾系统

任务流程:
  1. 定点起飞 → Cartographer / TF 就绪
  2. 起飞矩形视觉校准（下视 /dev/video0，检测黑色矩形标记，视觉闭环居中）
  3. 记录校准点为坐标原点
  4. 按预设航点序列巡航（平滑轨迹），期间 10 Hz 持续检测地形环
  5. 识别到 debris_flow（泥石流）时打断轨迹 → 悬停 → 降高 → 关泵 → 等 5s
     → 回升 → 继续航行
  6. 完成全部航点后降落在坐标原点

上位机已自启动 server_ros.py，本程序通过 FC_Client 连接。
ROS 建图组件启动流程参考 base_test.py 与 former_code/2024_D_24.py。
视觉闭环校准流程参考 former_code/2022_24.py 的 vision_approach 模式。
地形检测使用 vision_for_simulation 仿真视觉包（YOLO + 传统图像处理）。
"""
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
from loguru import logger

from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosManager import RosManager
from FlightController.Components.UartScreen import UARTScreen

from python_sdk.vision_for_simulation.takeoff_rectangle import (
    detect_takeoff_rectangle,
)
from python_sdk.vision_for_simulation.terrain_ring import (
    detect_nearest_terrain_ring,
)
from python_sdk.vision_for_simulation.camera_offsets import _center_to_offset

# ============ 可调参数 ============
CRUISE_SPEED = 22            # 水平导航速度 cm/s
CRUISE_HEIGHT = 150          # 巡航高度 cm
VERTICAL_SPEED = 22          # 垂直速度 cm/s

# 起飞矩形视觉校准参数（与 2022_24.py 一致）
CALIB_CLOSE_THRESHOLD_PX = 30   # 像素距离阈值：小于此值认为已居中
CALIB_APPROACH_SPEED = 15       # 逼近速度 cm/s
CALIB_FREQ = 10                 # 控制循环频率 Hz
CALIB_TIMEOUT = 60              # 校准超时 / s

# 摄像头索引（校准和地形环共用 /dev/video0）
CAMERA_INDEX = 0

# 地形环检测频率
RING_DETECT_FREQ = 10         # Hz

# YOLO 仿真模型 7 类地形:
#   0:snow_mountain  1:field  2:river  3:settlements
#   4:lake           5:debris_flow  6:wildfire
#
# 相邻地形在飞机正下方的时间间隔约 3 s，10 Hz 每段可检测约 30 次。

# 地形环触发后冷却时间（避免同一地形重复打断）
RING_COOLDOWN = 4.0           # s
# =================================


def _open_persistent_camera(index: int, width: int = 1280, height: int = 720) -> cv2.VideoCapture:
    """打开摄像头并保持常开，返回 VideoCapture 对象。"""
    if sys.platform.startswith("linux"):
        backends = (cv2.CAP_V4L2, cv2.CAP_ANY)
    elif sys.platform.startswith("win"):
        backends = (cv2.CAP_DSHOW, cv2.CAP_ANY)
    else:
        backends = (cv2.CAP_ANY,)

    for backend in backends:
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            return cap
        cap.release()
    raise RuntimeError(f"Unable to open camera index {index}")


class SimVisionTask:
    """仿真视觉任务，持有一个常开 /dev/video0。

    同时承担起飞矩形检测（起飞后校准用）和地形环检测（巡航中避障/动作触发用）。
    """

    def __init__(self, camera_index: int = CAMERA_INDEX):
        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_index = camera_index
        self._stop_flag = False

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def open(self):
        """打开摄像头。"""
        if self._cap is None:
            self._cap = _open_persistent_camera(self._camera_index)
            logger.info(f"[VISION] Camera {self._camera_index} opened")

    def close(self):
        """释放摄像头。"""
        if self._cap is not None:
            self._cap.release()
            logger.info(f"[VISION] Camera {self._camera_index} released")
        self._cap = None

    def stop(self):
        """标记停止，后台检测线程应检查此标志。"""
        self._stop_flag = True

    # ------------------------------------------------------------------
    # 帧读取
    # ------------------------------------------------------------------

    def _read_frame(self) -> Optional[np.ndarray]:
        """从摄像头读取一帧，失败返回 None。"""
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        return frame

    # ------------------------------------------------------------------
    # 检测 API
    # ------------------------------------------------------------------

    def detect_takeoff_offset(self) -> Optional[Tuple[float, float]]:
        """检测起飞矩形，返回 ``(x_px, y_px)`` 或 None。

        +x = 图像上方（机头前），+y = 图像左侧（机头左）。
        """
        frame = self._read_frame()
        if frame is None:
            return None
        detection = detect_takeoff_rectangle(frame)
        if detection is None:
            return None
        h, w = int(frame.shape[0]), int(frame.shape[1])
        return _center_to_offset(detection.center, (h, w))

    def detect_ring_offset(self) -> Optional[Tuple[float, float, str]]:
        """YOLO 检测最近地形环，返回 ``(x_px, y_px, label)`` 或 None。

        +x = 图像上方（机头前），+y = 图像左侧（机头左）。
        label 为 YOLO 类别名（如 debris_flow、lake 等）。
        """
        frame = self._read_frame()
        if frame is None:
            return None
        detection = detect_nearest_terrain_ring(frame)
        if detection is None:
            return None
        h, w = int(frame.shape[0]), int(frame.shape[1])
        offset_x, offset_y = _center_to_offset(detection.center, (h, w))
        return offset_x, offset_y, detection.class_name


class Mission(object):
    """2026 模拟赛 — 空地协同测绘救灾系统 任务类。

    轨迹巡航期间由后台线程 10 Hz 持续检测地形环。识别到特定地形后通过清除
    Navigation.traj_running_event 打断轨迹 → 悬停 → 执行动作 → 从剩余轨迹恢复航行。
    """

    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.navi: Navigation = kwargs["navi"]
        self.radar: LD_Radar = kwargs["radar"]
        self.rs: T265 = kwargs["rs"]
        self.sim_vision: Optional[SimVisionTask] = kwargs.get("sim_vision", None)
        self.cruise_height = CRUISE_HEIGHT

        # 坐标原点（视觉校准后设置，后续所有航点相对此点）
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0

        # 地形环打断机制
        self._ring_triggered = threading.Event()
        self._ring_label: str = ""
        self._ring_cooldown_until: float = 0.0

        # 线程控制
        self._stop_ring = threading.Event()
        self._ring_thread: Optional[threading.Thread] = None

    def stop(self):
        self._stop_ring.set()
        if self.sim_vision is not None:
            self.sim_vision.stop()
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    # ================================================================
    #  起飞矩形视觉校准
    #
    #  起飞后通过下视摄像头检测黑色矩形起飞标记，以视觉闭环
    #  将无人机移至标记正上方，消除起飞漂移误差。
    # ================================================================

    def _calibrate_to_takeoff_rectangle(
        self,
        close_threshold_px: float = CALIB_CLOSE_THRESHOLD_PX,
        approach_speed: float = CALIB_APPROACH_SPEED,
        freq: float = CALIB_FREQ,
        timeout: float = CALIB_TIMEOUT,
    ) -> bool:
        """视觉闭环校准：将无人机移动到起飞矩形正上方。

        参考 former_code/2022_24.py vision_approach() 的闭环模式。
        以 freq Hz 循环读取摄像头，检测起飞矩形像素偏移，
        用 move_by_direction 逼近直到像素距离小于阈值。
        """
        if self.sim_vision is None:
            logger.error("[CALIB] sim_vision is None")
            return False

        dt = 1.0 / max(freq, 5)
        logger.info(
            f"[CALIB] thresh={close_threshold_px}px, speed={approach_speed}cm/s, "
            f"freq={freq}Hz, timeout={timeout}s"
        )
        t0 = time.perf_counter()

        while True:
            if time.perf_counter() - t0 > timeout:
                self.navi.stop_move()
                logger.warning("[CALIB] Timeout")
                return False

            offset = self.sim_vision.detect_takeoff_offset()
            if offset is None:
                self.navi.stop_move()
                time.sleep(dt)
                continue

            x_px, y_px = offset
            dist_px = float(np.hypot(x_px, y_px))

            if dist_px <= close_threshold_px:
                self.navi.stop_move()
                logger.info(
                    f"[CALIB] Centered "
                    f"(dist={dist_px:.1f}px, x={x_px:.0f}, y={y_px:.0f})"
                )
                return True

            # 像素偏移 → 机体系移动方向
            # +x_px = 图像上方 = 机头前, +y_px = 图像左侧 = 机头左
            angle_deg = float(np.rad2deg(np.arctan2(y_px, x_px)))
            self.navi.move_by_direction(
                speed=approach_speed, direction_deg=angle_deg
            )
            time.sleep(dt)

    # ================================================================
    #  地形环后台检测（10 Hz）
    #
    #  巡航期间持续运行，识别到地形后通过 traj_running_event 打断轨迹。
    # ================================================================

    def _ring_detection_loop(self):
        """后台 daemon 线程：10 Hz 检测地形环，发现后打断轨迹。

        打断机制:
          1. 设置 _ring_triggered 事件通知主线程
          2. 存储 label
          3. 清除 navi.traj_running_event → _trajectory_task 在下一次
             内部检查（最迟 0.02 s）时保存剩余轨迹并返回
        """
        dt = 1.0 / max(RING_DETECT_FREQ, 5)
        logger.info(f"[RING] Loop started ({RING_DETECT_FREQ} Hz)")

        while not self._stop_ring.is_set():
            # 冷却期内跳过
            if time.perf_counter() < self._ring_cooldown_until:
                self._stop_ring.wait(dt)
                continue
            # 上一次触发还未被主线程处理完
            if self._ring_triggered.is_set():
                self._stop_ring.wait(dt)
                continue

            if self.sim_vision is None:
                break

            offset = self.sim_vision.detect_ring_offset()
            if offset is not None:
                x_px, y_px, label = offset
                logger.info(
                    f"[RING] Trigger: label={label}, "
                    f"offset=({x_px:.1f}, {y_px:.1f})px"
                )
                self._ring_label = label
                self._ring_triggered.set()
                # 打断轨迹 — Navigation 内部检查此 event
                self.navi.traj_running_event.clear()

            self._stop_ring.wait(dt)

        logger.info("[RING] Loop stopped")

    # ================================================================
    #  地形环动作
    #
    #  目前仅 debris_flow（泥石流）触发救灾动作，其余地形仅记录。
    # ================================================================

    def _perform_ring_action(self, label: str) -> None:
        """根据检测到的地形类别执行对应动作。

        调用时无人机已通过 stop_move 悬停在当前位置。
        """
        logger.info(
            f"[ACTION] Ring action for '{label}' "
            f"at ({self.navi.current_x:.0f}, {self.navi.current_y:.0f})"
        )

        if label == "debris_flow":
            # 泥石流: 降高 → 关泵 → 等待 → 回升 → 继续航行
            self._action_debris_flow()
        else:
            logger.info(f"[ACTION] Label '{label}' has no dedicated action, skipping")
            time.sleep(0.5)

    def _action_debris_flow(self) -> None:
        """泥石流救灾动作序列。

        流程: 黄灯 → 降至 90cm → 关水泵(digital output 0) → 等待 5s
              → 回升至巡航高度 150cm → 绿灯 → 继续航行。
        """
        fc = self.fc
        navi = self.navi

        logger.info("[ACTION:debris_flow] Starting debris-flow action sequence")

        # 1. 指示灯 → 黄色（警告状态）
        fc.set_indicator_led(255, 255, 0)
        logger.info("[ACTION:debris_flow] Indicator LED → yellow")

        # 2. 降低高度至 90cm（接近地面以便水泵作业）
        target_low = 90.0
        logger.info(f"[ACTION:debris_flow] Descending to {target_low}cm")
        navi.set_height(target_low)
        height_ok = navi.wait_for_height(
            time_thres=0.5,
            height_thres=10,
            timeout=10,
        )
        if not height_ok:
            logger.warning(
                f"[ACTION:debris_flow] Height not reached: "
                f"current={navi.current_height:.1f}cm, target={target_low}cm"
            )

        # 3. 关闭水泵（数字输出通道 0 → False）
        fc.set_digital_output(0, False)
        logger.info("[ACTION:debris_flow] Digital output 0 → OFF (pump stopped)")

        # 4. 在低空等待 5 秒
        logger.info("[ACTION:debris_flow] Waiting 5s at low altitude")
        time.sleep(5.0)

        # 5. 回升至巡航高度
        logger.info(f"[ACTION:debris_flow] Ascending to {self.cruise_height}cm")
        navi.set_height(self.cruise_height)
        height_ok = navi.wait_for_height(
            time_thres=0.5,
            height_thres=10,
            timeout=10,
        )
        if not height_ok:
            logger.warning(
                f"[ACTION:debris_flow] Height not reached: "
                f"current={navi.current_height:.1f}cm, target={self.cruise_height}cm"
            )

        # 6. 指示灯 → 绿色（恢复正常航行状态）
        fc.set_indicator_led(0, 255, 0)
        logger.info("[ACTION:debris_flow] Indicator LED → green, action complete")

    # ================================================================
    #  平滑轨迹生成
    # ================================================================

    def _build_smooth_traj(
        self, waypoints: List[Tuple[float, float]]
    ) -> List[Tuple[float, ...]]:
        """以当前位置为起点，生成经过所有 waypoints 的平滑轨迹。"""
        start_x = float(self.navi.current_x)
        start_y = float(self.navi.current_y)
        start_height = float(self.navi.current_height)
        trajectory_waypoints = np.vstack(
            ([[start_x, start_y]], np.asarray(waypoints, dtype=float))
        )
        traj_list = self.navi.create_smooth_traj_list(
            waypoints=trajectory_waypoints,
            altitude=self.cruise_height,
        )
        first_point = traj_list[0]
        traj_list[0] = (first_point[0], first_point[1], start_height)
        logger.info(
            f"[TRAJ] {len(traj_list)} points from "
            f"({start_x:.0f}, {start_y:.0f}) through {len(waypoints)} waypoints"
        )
        return traj_list

    # ================================================================
    #  主任务
    #
    #  完整流程:
    #    起飞 → Cartographer 就绪 → 起飞矩形校准 → 记录原点
    #    → 生成平滑轨迹 → 启动地形环检测
    #    → 轨迹巡航（可被地形环打断）→ 降落
    # ================================================================

    def run(self):
        fc = self.fc
        navi = self.navi

        # ---- 航点（相对原点，cm，匿名 ROS 坐标系）----
        raw_waypoints = [
            (100, -40),
            (240, -40),
            (240, -320),
            (100, -320),
            (100, -110),
            (170, -110),
            (170, -250),
            (0, 0),
        ]

        # ---- 导航参数 ----
        navi.set_navigation_speed(CRUISE_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)

        # ---- 启动导航（ROS fusion 模式，Cartographer + T265）----
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started (fusion-ros)")
        navi.set_rs_speed_report(True, 2)

        # ---- 初始化 ----
        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        # ---- 定点起飞 ----
        logger.info(f"[MISSION] Takeoff to {self.cruise_height}cm")
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        # ---- Cartographer 初始化等待 ----
        # 在线 SLAM 需要一定运动量（起飞 + 漂移）完成首个 Submap。
        CART_TIMEOUT = 30.0
        logger.info(f"[MISSION] Waiting for Cartographer TF ({CART_TIMEOUT}s)...")
        t0 = time.perf_counter()
        while True:
            time.sleep(1)
            logger.info(f"[MISSION] current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
            if time.perf_counter() - t0 > CART_TIMEOUT:
                raise RuntimeError(
                    f"Cartographer TF timeout ({CART_TIMEOUT}s)"
                )
        logger.info(
            f"[MISSION] Cartographer TF ok ({time.perf_counter() - t0:.1f}s)"
        )
        fc.set_indicator_led(0, 0, 0)

        # ---- 起飞矩形视觉校准（消除起飞漂移）----
        if self.sim_vision is not None:
            if not self._calibrate_to_takeoff_rectangle():
                raise RuntimeError("Takeoff calibration failed")
        else:
            logger.warning("[MISSION] sim_vision unavailable")

        # ---- 记录坐标原点（校准后当前位置即为原点）----
        self._origin_x = float(navi.current_x)
        self._origin_y = float(navi.current_y)
        logger.info(
            f"[MISSION] Origin = ({self._origin_x:.1f}, {self._origin_y:.1f})"
        )

        # ---- 航点坐标变换（相对原点 → 绝对坐标）----
        waypoints: List[Tuple[float, float]] = [
            (x + self._origin_x, y + self._origin_y)
            for (x, y) in raw_waypoints
        ]

        # ---- 生成平滑轨迹 ----
        traj_list = self._build_smooth_traj(waypoints)

        # ---- 启动地形环后台检测（10 Hz，巡航期间持续运行）----
        if self.sim_vision is not None:
            self._stop_ring.clear()
            self._ring_thread = threading.Thread(
                target=self._ring_detection_loop, daemon=True
            )
            self._ring_thread.start()

        # ============================================================
        #  轨迹巡航 + 地形环打断循环
        #
        #  Navigation._trajectory_task 每 0.02 s 检查 traj_running_event。
        #  后台线程检测到地形后清除该 event → 轨迹在下一个内部循环
        #  退出，同时保存剩余轨迹到 traj_list_before_stop。
        #  主线程据此区分「正常完成」与「打断」，执行动作后重新接续。
        # ============================================================
        while len(traj_list) > 0:
            logger.info(
                f"[TRAJ] Starting segment: {len(traj_list)} points remaining"
            )
            success = navi.navigation_follow_trajectory(traj_list)

            if self._ring_triggered.is_set():
                # ---- 地形环打断 ----
                label = self._ring_label
                logger.info(
                    f"[MISSION] Interrupted by ring: '{label}'"
                )
                navi.stop_move()       # 悬停在当前位置
                time.sleep(0.2)         # 等待悬停稳定

                self._perform_ring_action(label)

                # 设置冷却期（避免原地重复触发）
                self._ring_cooldown_until = time.perf_counter() + RING_COOLDOWN
                self._ring_triggered.clear()

                # 从剩余轨迹接续航行
                traj_list = list(navi.traj_list_before_stop)
                if len(traj_list) == 0:
                    logger.info("[TRAJ] No remaining points, mission done")
                    break
                logger.info(
                    f"[TRAJ] Resuming: {len(traj_list)} points remaining"
                )

            elif not success:
                # ---- 非打断性失败（pose 过期、超时等）----
                raise RuntimeError("Navigation trajectory failed unexpectedly")

            else:
                # ---- 正常完成 ----
                logger.info("[TRAJ] Segment completed successfully")
                break

        # ---- 停止地形环检测 ----
        self._stop_ring.set()

        # ---- 降落在坐标原点 ----
        logger.info("[MISSION] Landing at origin")
        navi.pointing_landing((self._origin_x, self._origin_y))


# ================================================================
#  __main__: 初始化 ROS 组件 → 创建 Mission → 执行任务
# ================================================================
if __name__ == "__main__":
    # ---- 1. 权限配置 ----
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")   # CP2102 雷达
    rm.chmod("/dev/ttyACM0")   # LX 飞控
    rm.chmod("/dev/video0")    # USB 摄像头（起飞矩形校准 + 地形环检测）

    # ---- 2. 启动 ROS 建图组件 ----
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package(
        "tf2_ros", "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link"
    )

    # ---- 3. 连接飞控（上位机已运行 server_ros.py，使用 FC_Client）----
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)

    # ---- 4. 初始化传感器 ----
    t265 = T265("ros")
    t265.start()

    radar = LD_Radar()
    radar.start("ros")

    screen = UARTScreen(fc)

    # ---- 5. 初始化仿真视觉（/dev/video0 常开）----
    sim_vision = SimVisionTask(camera_index=CAMERA_INDEX)
    sim_vision.open()

    # ---- 6. 桥梁层 ----
    mapper = RosMapper()

    # ---- 7. 导航层 ----
    navi = Navigation(
        fc=fc,
        rs=t265,
        radar=radar,
        mapper=mapper,
    )

    # ---- 8. 启动 ROS Python 节点执行器 ----
    RosNodeRunner().add_nodes().run()

    # ---- 9. 创建 Mission ----
    mission = Mission(
        fc=fc,
        rs=t265,
        radar=radar,
        navi=navi,
        sim_vision=sim_vision,
    )

    # TODO: 地面站起飞命令等待
    #   1. fc.set_digital_output(0, True)  — 打开水泵 / 数字输出通道 0
    #   2. 参考 former_code/2024_D_24.py __main__ 中 FCWirelessTransport /
    #      start_ground_station / enable_ground_command_reception 的完整流程，
    #      等待地面站通过飞控 UT2/HC-14 无线链路发送 START_MISSION 命令
    #   3. 收到 START_MISSION 后等待 5 秒再执行 mission.run()
    #      (可复用 2024_D_24.py 的 fc.receive_ground_command / accept_ground_command /
    #       prepare_ground_mission 调用链)
    #   4. mission.run() 完成后向地面站上报 COMPLETED / FAILED 并 complete/fail
    #      ground_command

    try:
        mission.run()
    except Exception as e:
        logger.exception(f"[MANAGER] Mission Failed: {e}")
    finally:
        mission.stop()
        if fc.state.unlock.value:
            logger.warning("[MANAGER] Auto Landing (Emergency)")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            ret = fc.wait_for_lock()
            if not ret:
                fc.lock()
        sim_vision.close()

    logger.info("[MANAGER] Mission finished")
    fc.close()

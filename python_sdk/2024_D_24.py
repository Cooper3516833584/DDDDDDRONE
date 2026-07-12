"""
2024_D_24: QR码巡检任务
使用 ROS (fusion-ros) 作为位置闭环
基于 2025_嵌赛.py 模板框架
"""
import time
import numpy as np
from loguru import logger
from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosManager import RosManager

# ============ 可调参数 ============
CRUISE_SPEED = 22           # 水平导航速度 cm/s
CRUISE_HEIGHT = 120         # 巡航高度 cm (待定)
VERTICAL_SPEED = 22         # 垂直速度 cm/s
QR_SEARCH_STEP = 50         # QR 搜索步长 cm
QR_SEARCH_MAX = 300         # QR 搜索最大距离 cm
# =================================


class Mission(object):
    """QR 码巡检任务"""

    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.navi: Navigation = kwargs["navi"]
        self.rs: T265 = kwargs["rs"]
        self.mapper: RosMapper = kwargs.get("mapper", None)
        # [TODO-4] 串口屏 — 取消注释下面两行, 并从参数接收 screen
        # self.screen: UARTScreen = kwargs.get("screen", None)
        self.cruise_height = CRUISE_HEIGHT
        # [TODO-2] 视觉位置闭环 — QR 码偏移量
        self.qr_offset: tuple = (0.0, 0.0)

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    # ================================================================
    #  TODO 汇总 (按优先级排列)
    # ================================================================
    #  [TODO-1] 障碍物板距离判定
    #    利用雷达实时扫描数据，在飞行过程中持续监控无人机到
    #    前方/侧方障碍物的距离。当距离低于安全阈值时自动悬停
    #    或绕行，防止碰撞。
    #    涉及方法: check_barrier_distance()
    #
    #  [TODO-2] 视觉识别位置闭环
    #    将占位 detect_qr_code() 替换为真实的二维码检测，并实现
    #    视觉精调闭环: 检测到 QR 后根据其在画面中的位置偏移量
    #    (dx, dy) 微调无人机位姿，使 QR 码居中后再执行动作。
    #    参考: 2022_24_noscreen_nomotor.py 的 vision_approach()
    #    涉及方法: detect_qr_code(), vision_qr_approach()
    #
    #  [TODO-3] 激光笔控制
    #    通过飞控的数字输出或 PWM 通道控制激光笔开关。
    #    在检测到目标后点亮激光笔指示位置。
    #    涉及方法: laser_on(), laser_off()
    #
    #  [TODO-4] 串口屏通信
    #    集成 UARTScreen 实现串口屏状态上报:
    #    - 任务进度/当前步骤显示
    #    - 传感器状态 (雷达/ROS/T265) 指示灯
    #    - 电池电压显示
    #    - 远程紧急停止指令监听
    #    参考: 2025_嵌赛.py 的 UARTScreen 用法
    #    涉及位置: Mission.__init__(), __main__ 初始化段
    # ================================================================

    # ================================================================
    #  [TODO-1] 障碍物板距离判定
    # ================================================================
    def check_barrier_distance(self, min_safe_distance: float = 80.0) -> bool:
        """
        [TODO-1] 利用雷达数据检查前方/侧方障碍物距离是否安全

        Args:
            min_safe_distance: 最小安全距离 / cm

        Returns:
            True 表示安全，False 表示距离过近需要避让

        实现思路:
          1. 从 radar.map 读取当前航向前方 ±30° 范围的最短距离
          2. 从 radar.map 读取当前机头左侧 (y正) ±30° 范围的最短距离
          3. 若任一方向距离 < min_safe_distance → 触发避让
        """
        # ---- 占位实现: 始终返回安全 ----
        # radar = self.radar
        # forward_deg = (self.navi.current_yaw) % 360          # 机头在雷达坐标系中的角度
        # left_deg    = (self.navi.current_yaw + 90) % 360     # 左侧
        # search_range = 30                                     # ±30°
        #
        # forward_dist = min(
        #     radar.map[deg] for deg in range(forward_deg - search_range,
        #                                     forward_deg + search_range)
        #     if radar.map[deg] != -1
        # )
        # if forward_dist < min_safe_distance:
        #     logger.warning(f"[BARRIER] Forward obstacle at {forward_dist}cm!")
        #     return False
        #
        # left_dist = min(...) ...
        #
        # return True
        return True

    # ================================================================
    #  [TODO-3] 激光笔控制
    # ================================================================
    def laser_on(self, channel: int = 1):
        """
        [TODO-3] 打开激光笔

        Args:
            channel: 飞控数字输出通道号 (需根据实际接线确认)

        实现思路:
          fc.set_digital_output(channel, True)
          或 fc.set_PWM_output(channel, 100)
        """
        logger.info(f"[LASER] Laser ON (channel={channel}) — placeholder")
        # self.fc.set_digital_output(channel, True)

    def laser_off(self, channel: int = 1):
        """
        [TODO-3] 关闭激光笔
        """
        logger.info(f"[LASER] Laser OFF (channel={channel}) — placeholder")
        # self.fc.set_digital_output(channel, False)

    # ================================================================
    #  占位视觉函数 — 待实际视觉模块编写后替换
    # ================================================================

    def detect_qr_code(self) -> bool:
        """
        [TODO-2] 二维码识别  (占位阶段 → 待视觉模块编写后替换)

        Returns:
            True 表示当前画面中检测到二维码

        待实现:
          - 从摄像头读取帧
          - 运行 QR 码检测器 (如 OpenCV QRCodeDetector / zbar / pyzbar)
          - 若检测到, 将二维码中心偏移 (dx, dy) 保存到 self.qr_offset
        """
        # 实际实现示例:
        #   ret, frame = self.cam.read()
        #   if not ret: return False
        #   result = qr_detector.detectAndDecode(frame)
        #   if result[0]:
        #       # 计算二维码在画面中的位置偏移
        #       self.qr_offset = (center_x - 320, center_y - 240)
        #       return True
        #   return False
        return False

    def vision_qr_approach(self, timeout: float = 30.0):
        """
        [TODO-2] 视觉位置闭环 — QR 码视觉精调

        检测到 QR 码后, 根据 QR 在画面中的偏移量微调无人机位置,
        使 QR 码居中, 实现精确对准。

        Args:
            timeout: 精调超时时间 / s

        实现思路 (参考 2022_24_noscreen_nomotor.py vision_approach):
          1. 持续读取 detect_qr_code() 的偏移结果
          2. 用 move_by_direction() 以小速度 (3-5 cm/s) 沿偏移方向逼近
          3. 当偏移量小于阈值时停止, stop_move()
          4. 超时后也停止, 记录警告
        """
        logger.info(f"[VISION] QR vision approach started (timeout={timeout}s) — placeholder")
        # ---- 占位: 跳过视觉精调 ----
        # t0 = time.perf_counter()
        # while time.perf_counter() - t0 < timeout:
        #     found = self.detect_qr_code()
        #     if not found:
        #         time.sleep(0.1)
        #         continue
        #     dx, dy = self.qr_offset
        #     if abs(dx) < 20 and abs(dy) < 20:
        #         self.navi.stop_move()
        #         logger.info("[VISION] QR centered")
        #         return
        #     direction = np.rad2deg(np.arctan2(-dy, -dx))  # offset → 移动方向
        #     self.navi.move_by_direction(speed=5, direction_deg=direction)
        #     time.sleep(0.1)
        # logger.warning("[VISION] QR approach timeout")
        # self.navi.stop_move()

    def qr_code_action(self):
        """
        [TODO-2][TODO-3] 检测到二维码后执行的动作

        待实现:
          1. [TODO-2] 先调用 vision_qr_approach() 视觉精调对准 QR 码
          2. [TODO-3] 打开激光笔指示
          3. 执行任务动作 (拍照记录 / 投放物品 / 悬停记录坐标等)
          4. [TODO-3] 关闭激光笔
        """
        logger.info("[MISSION] >>> QR code action executed (placeholder) <<<")
        # self.vision_qr_approach(timeout=30)
        # self.laser_on()
        time.sleep(1.0)
        # self.laser_off()

    def detect_landing_spot(self) -> bool:
        """
        [TODO-2] 落点识别  (占位阶段 → 待视觉模块编写后替换)

        Returns:
            True 表示识别到可降落区域

        待实现:
          - 下视摄像头拍摄地面
          - 通过颜色/纹理/标记检测安全降落区域 (如"H"标记)
          - 若未检测到安全区域 → 使用预设的默认降落点
        """
        return False

    # ================================================================
    #  f1: QR 码扫描 → 搜索 → 动作 → 升回巡航高度
    # ================================================================

    def scan_qr_code_and_act(self):
        """
        扫描二维码:
          1. 尝试识别二维码
          2. 若未识别到，向当前机头左侧 (y正方向) 移动 QR_SEARCH_STEP cm
          3. 重复直至识别到或超过最大搜索距离
          4. 识别到后执行动作
          5. 升回巡航高度

        注意: y正方向 = 机头左侧, 机头朝向由当前 yaw 决定
        """
        navi = self.navi
        logger.info("[MISSION] ┌─ f1: QR scan sequence START ─────────────────")

        qr_found = False
        total_searched = 0.0

        while not qr_found and total_searched < QR_SEARCH_MAX:
            # 尝试识别
            qr_found = self.detect_qr_code()

            if qr_found:
                break

            # 未识别到 → 向机头左侧 (y正方向) 移动 QR_SEARCH_STEP cm
            #
            # 坐标系说明:
            #   absolute x+ = north,  absolute y+ = west
            #   drone forward (= x正) = yaw 指向
            #   drone left    (= y正) = yaw + 90°
            #
            # 设 yaw_rad = 当前 yaw 角 (rad), 向左移动 step cm:
            #   abs_dx = -step · sin(yaw_rad)
            #   abs_dy =  step · cos(yaw_rad)
            #
            yaw_rad = np.deg2rad(navi.current_yaw)
            abs_dx = -QR_SEARCH_STEP * np.sin(yaw_rad)
            abs_dy = QR_SEARCH_STEP * np.cos(yaw_rad)

            target = navi.current_point + np.array([abs_dx, abs_dy])
            logger.info(
                f"[MISSION]   QR not found (yaw={navi.current_yaw:.1f}°), "
                f"moving y+ {QR_SEARCH_STEP}cm → {np.round(target, 1)}"
            )
            navi.navigation_to_waypoint(target, wait=True)
            total_searched += QR_SEARCH_STEP

        # 识别到 → 执行动作
        if qr_found:
            logger.info("[MISSION]   QR code detected!")
            self.qr_code_action()
        else:
            logger.warning(
                f"[MISSION]   QR not found after {total_searched:.0f}cm search, "
                f"skipping action"
            )

        # 升回巡航高度 (如果之前因动作降低了高度)
        logger.info(f"[MISSION]   Ascending to cruise height {self.cruise_height}cm")
        navi.set_height(self.cruise_height)
        navi.wait_for_height()

        logger.info("[MISSION] └─ f1: QR scan sequence END ───────────────────")

    # ================================================================
    #  主任务流程
    # ================================================================

    def run(self):
        fc = self.fc
        navi = self.navi

        # ---------- 导航参数 ----------
        navi.set_navigation_speed(CRUISE_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)

        # ---------- 启动导航 ----------
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started (fusion-ros)")

        fc.set_action_log(False)
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")

        # ---------- Cartographer 初始化等待 ----------
        # 在线 SLAM 模式下等待 TF 变换建立
        while True:
            time.sleep(1)
            logger.info(f"[MISSION] current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
        logger.info("[MISSION] Cartographer TF established")

        # ---------- 定点起飞 ----------
        logger.info(f"[MISSION] Taking off to {self.cruise_height}cm")
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        # ================================================================
        #  Step A:  f1 (yaw=0°)
        #           机头朝北, y正 = 绝对西
        # ================================================================
        self.scan_qr_code_and_act()

        # ================================================================
        #  Step B:  yaw=0°, y正方向 100cm, x正方向 150cm
        #           y正(dir 0°=left=west) = abs y+100
        #           x正(dir 0°=forward=north) = abs x+150
        # ================================================================
        target = navi.current_point + np.array([150.0, 100.0])
        logger.info(f"[MISSION] Step B: fly x+150 y+100 (yaw=0) → {np.round(target, 1)}")
        navi.navigation_to_waypoint(target, wait=True)

        # ================================================================
        #  Step C:  yaw=180°, 运行 f1
        #           机头朝南, y正(左) = 绝对东 (abs y-)
        # ================================================================
        logger.info("[MISSION] Step C: set yaw=180°")
        navi.set_yaw(180)
        navi.wait_for_yaw()
        time.sleep(0.5)
        self.scan_qr_code_and_act()

        # ================================================================
        #  Step D:  yaw=0°, 运行 f1
        #           机头朝北, y正(左) = 绝对西 (abs y+)
        # ================================================================
        logger.info("[MISSION] Step D: set yaw=0°")
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)
        self.scan_qr_code_and_act()

        # ================================================================
        #  Step E:  yaw=0°, y正方向 100cm, x正方向 150cm
        # ================================================================
        target = navi.current_point + np.array([150.0, 100.0])
        logger.info(f"[MISSION] Step E: fly x+150 y+100 (yaw=0) → {np.round(target, 1)}")
        navi.navigation_to_waypoint(target, wait=True)

        # ================================================================
        #  Step F:  yaw=180°, 运行 f1
        # ================================================================
        logger.info("[MISSION] Step F: set yaw=180°")
        navi.set_yaw(180)
        navi.wait_for_yaw()
        time.sleep(0.5)
        self.scan_qr_code_and_act()

        # ================================================================
        #  Step G:  yaw=0°, x正方向 70cm, y正方向 200cm
        # ================================================================
        logger.info("[MISSION] Step G: set yaw=0°")
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        target = navi.current_point + np.array([70.0, 200.0])
        logger.info(f"[MISSION] Step G: fly x+70 y+200 (yaw=0) → {np.round(target, 1)}")
        navi.navigation_to_waypoint(target, wait=True)

        # ================================================================
        #  Step H:  落点识别
        # ================================================================
        logger.info("[MISSION] Step H: Landing spot detection")
        landing_spot_found = self.detect_landing_spot()
        if landing_spot_found:
            logger.info("[MISSION]   Landing spot confirmed")
        else:
            logger.info("[MISSION]   Landing spot not detected (placeholder), "
                        "landing at basepoint")

        # ================================================================
        #  Step I:  定点降落
        # ================================================================
        logger.info("[MISSION] Step I: Landing at basepoint (0, 0)")
        navi.pointing_landing((0, 0))
        logger.info("[MISSION] ========== Mission Complete ==========")


# ================================================================
#  __main__: 初始化 → 启动 ROS → 运行任务
#  完全遵循 2025_嵌赛.py 的 ROS 启动框架
# ================================================================
if __name__ == "__main__":
    # ---- 步骤 1: 权限配置 ----
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")   # 雷达
    rm.chmod("/dev/ttyACM0")   # 飞控
    rm.chmod("/dev/video1")    # T265

    # ---- 步骤 2: 启动 ROS 包 (tmux 后台) ----
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package(
        "tf2_ros", "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link"
    )

    # ---- 步骤 3: 连接飞控 ----
    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)

    # ---- 步骤 4: 初始化传感器 Python 包装层 ----
    t265 = T265("ros")
    t265.start()

    radar = LD_Radar()
    radar.start("ros")

    # ---- 步骤 5: 初始化桥梁层 ----
    mapper = RosMapper()

    # ---- 步骤 5.5: [TODO-4] 串口屏初始化 (占位) ----
    # from FlightController.Components.UartScreen import UARTScreen
    # screen = UARTScreen(fc)

    # ---- 步骤 6: 初始化导航层 ----
    navi = Navigation(
        fc=fc,
        rs=t265,
        radar=radar,
        mapper=mapper,
    )

    # ---- 步骤 7: 启动 ROS Python 节点执行器 ----
    RosNodeRunner().add_nodes().run()

    # ---- 步骤 8: 创建 Mission 并运行 ----
    mission = Mission(
        fc=fc,
        rs=t265,
        radar=radar,
        navi=navi,
        mapper=mapper,
    )

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

    logger.info("[MANAGER] Mission finished")
    fc.close()

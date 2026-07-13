"""
2024_D_24: QR码巡检任务
使用 ROS (fusion-ros) 作为位置闭环
基于 2025_嵌赛.py 模板框架
"""
import json
import os
import time
from typing import Optional
import numpy as np
from loguru import logger
from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosManager import RosManager
from FlightController.Components.GroundStationLink import (
    CommandId,
    MissionState,
    RejectReason,
)
import vision_of_tf
from vision_of_tf import (
    _detect_qrcodes,
    _open_usb_camera,
    _select_nearest_to_image_center,
    _center_to_first_y_offset,
)

# ============ 可调参数 ============
CRUISE_SPEED = 22            # 水平导航速度 cm/s
CRUISE_HEIGHT = 150          # 巡航高度 cm (待定)
QR_UPPER_HEIGHT = 150.0      # 上层二维码扫描高度 cm
QR_LOWER_HEIGHT = 90.0       # 下层二维码扫描高度 cm
LANDING_SCAN_HEIGHT = 30.0   # 黑色圆形降落标记扫描高度 cm
VERTICAL_SPEED = 22          # 垂直速度 cm/s
QR_SEARCH_STEP = 50          # QR 搜索步长 cm
QR_SEARCH_MAX = 250          # QR 搜索最大距离 cm
BARRIER_TARGET_DIST = 75.0   # 障碍物板目标距离 cm
BARRIER_TOLERANCE = 5.0      # 距离允许误差 cm (±)
BARRIER_APPROACH_SPEED = 5  # 逼近速度 cm/s
BARRIER_ANOMALY_THRESH = 20  # 连续两次测距差超过此值判定为异常
QR_CAMERA_INDEX = 2          # 前视 USB 摄像头索引 (0bda:3035, 二维码识别)
LANDING_CAMERA_INDEX = 0     # 下视 USB 摄像头索引 (0c45:636b, 落点识别)
VISION_APPROACH_SPEED = 5    # 视觉精调水平速度 cm/s
VISION_PX_THRESH = 30        # 水平居中像素阈值
VISION_Z_PX_THRESH = 40      # QR 垂直居中像素阈值
VISION_HEIGHT_STEP = 5      # QR 视觉精调单步高度调整量 cm
LANDING_APPROACH_SPEED = 8   # 落点视觉精调速度 cm/s
QR_GRID_Z_STEP = 60          # QR 网格纵向 (z) 间距 cm
QR_GRID_Y_STEP = 40          # QR 网格横向 (y) 间距 cm
QR_SCAN_TOTAL_ROUNDS = 4     # 总巡检轮数
QR_SCAN_PER_ROUND = 6        # 每轮 QR 数量
INVENTORY_TOTAL = QR_SCAN_TOTAL_ROUNDS * QR_SCAN_PER_ROUND
INVENTORY_RECORD_FILENAME = "2024_D_24_inventory.json"
LASER_CHANNEL = 1            # 沿用当前任务代码预留的数字输出通道
GROUND_LED_WHITE = ((255, 255, 255),) * 7
GROUND_LED_OFF = ((0, 0, 0),) * 7
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
        # 摄像头索引 (可通过 kwargs 覆盖)
        self.qr_camera_index: int = kwargs.get("qr_camera_index", QR_CAMERA_INDEX)
        self.landing_camera_index: int = kwargs.get(
            "landing_camera_index", LANDING_CAMERA_INDEX
        )
        self._last_qr_offset: Optional[tuple] = None
        # QR 多轮扫描状态
        self._qr_round: int = 0           # 当前轮次 (0-based, 每轮结束后递增)
        self._qr_positions: dict = {}      # {货物编号: "A1", ...}
        self._inventory_record_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            INVENTORY_RECORD_FILENAME,
        )
        self._save_inventory_results()

    def stop(self):
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    @property
    def inventory_results(self) -> dict:
        return dict(self._qr_positions)

    def _save_inventory_results(self):
        """将已识别的物理槽位和二维码数字原子写入任务同目录。"""
        records = [
            {
                "position": position_name,
                "qr_number": qr_number,
            }
            for qr_number, position_name in sorted(
                self._qr_positions.items(), key=lambda item: item[1]
            )
        ]
        temporary_path = self._inventory_record_path + ".tmp"
        with open(temporary_path, "w", encoding="utf-8", newline="\n") as file:
            json.dump(records, file, ensure_ascii=False, indent=2)
            file.write("\n")
        os.replace(temporary_path, self._inventory_record_path)
        logger.info(
            f"[INVENTORY] Saved {len(records)} records to "
            f"{INVENTORY_RECORD_FILENAME}"
        )

    # ================================================================
    #  TODO 汇总 (按优先级排列)
    # ================================================================
    #  [TODO-1] 障碍物板距离判定 — ✅ 已实现
    #    barrier_distance_align() 利用雷达 285°~359° 扇区点云拟合直线，
    #    10Hz 闭环将机身到障碍物的距离调整到 75±5cm。
    #
    #  [TODO-2] 视觉识别位置闭环 — ✅ 已接入 vision_of_tf
    #    detect_qr_code() → vision_of_tf.detect_qrcode_offset()
    #    detect_landing_spot() → vision_of_tf.detect_black_circle_offset()
    #    vision_qr_approach() → 根据 y_px 偏移量左右精调使 QR 居中
    #
    #  [TODO-3] 激光笔控制
    #    通过飞控的数字输出或 PWM 通道控制激光笔开关。
    #    涉及方法: laser_on(), laser_off()
    #
    #  [TODO-4] 串口屏通信
    #    集成 UARTScreen 实现串口屏状态上报。
    #    涉及位置: Mission.__init__(), __main__ 初始化段
    # ================================================================

    # ================================================================
    #  [TODO-1] 障碍物板距离判定与位置闭环 (已实现)
    # ================================================================

    def _measure_barrier_distance(self) -> "Optional[float]":
        """截取雷达 0°~75° CCW（机头→左侧）扇区点云，拟合直线后
        返回无人机原点到该直线的垂直距离。

        雷达坐标系: 0°=机头正前方, 角度顺时针增加(右转)
        0°~75° CCW → 在雷达坐标中对应 285°~359°

        Returns:
            距离 / cm，点云不足 (≤5 点) 时返回 None
        """
        radar_map = self.radar.map
        acc = radar_map.ACC            # 3，每度 3 个 bin
        from_idx = int(285 * acc)      # 855
        to_idx_excl = int(360 * acc)   # 1080 → range(855, 1080) 共 225 bin

        pts_cm = []
        for idx in range(from_idx, to_idx_excl):
            d_mm = radar_map.data[idx]
            if d_mm == -1:
                continue
            # 雷达坐标系 (0=fwd, cw) → 匿名坐标系 (x=fwd, y=left)
            deg = idx / acc
            rad = np.deg2rad(deg)
            x_cm = d_mm * np.cos(rad) / 10.0    # 前向分量 cm
            y_cm = -d_mm * np.sin(rad) / 10.0   # 左侧分量 cm
            pts_cm.append([x_cm, y_cm])

        pts = np.array(pts_cm)
        if len(pts) < 5:
            return None

        # SVD 总最小二乘直线拟合
        mean = pts.mean(axis=0)
        _, _, vh = np.linalg.svd(pts - mean)
        normal = vh[1]  # 拟合直线的法向量 (与直线垂直)

        # 原点到直线的垂直距离 = |mean · normal|
        return float(abs(np.dot(mean, normal)))

    def barrier_distance_align(
        self,
        target_distance: float = BARRIER_TARGET_DIST,
        tolerance: float = BARRIER_TOLERANCE,
        speed: float = BARRIER_APPROACH_SPEED,
        anomaly_threshold: float = BARRIER_ANOMALY_THRESH,
        timeout: float = 60.0,
    ) -> bool:
        """障碍物板距离判定与位置闭环。

        识别到 QR 码后调用，以 10Hz 频率检测到障碍物板的距离，
        通过前/后飞行将距离调整到 target_distance ± tolerance 内。

        Args:
            target_distance: 目标距离 / cm (默认 75)
            tolerance: 允许误差 / cm (默认 ±5)
            speed: 逼近速度 / cm/s (默认 5)
            anomaly_threshold: 连续两次测距差阈值 / cm，
                               超过则丢弃后一次数据，飞行方向延续
            timeout: 闭环超时 / s

        控制逻辑:
            distance < target → 机头反方向 (后退, 远离障碍物)
            distance > target → 机头正方向 (前进, 靠近障碍物)
            |distance - target| < tolerance → 悬停, 退出
        """
        navi = self.navi
        logger.info(
            f"[BARRIER] Align start: target={target_distance}±{tolerance}cm, "
            f"speed={speed}cm/s"
        )

        prev_distance: Optional[float] = None
        prev_direction: float = 0.0  # 仅在 prev_distance 非 None 时有效
        t0 = time.perf_counter()

        while True:
            if time.perf_counter() - t0 > timeout:
                navi.stop_move()
                final_distance = (
                    "unavailable"
                    if prev_distance is None
                    else f"{prev_distance:.1f}cm"
                )
                logger.warning(
                    f"[BARRIER] Timeout after {timeout}s, "
                    f"last valid distance={final_distance}"
                )
                return False

            # ---- 1. 雷达测距 ----
            distance = self._measure_barrier_distance()

            if distance is None:
                logger.warning("[BARRIER] No valid radar points, hovering")
                navi.stop_move()
                time.sleep(0.1)
                continue

            logger.debug(f"[BARRIER] measured distance = {distance:.1f} cm")

            # ---- 2. 到达目标区域 → 退出 ----
            if abs(distance - target_distance) < tolerance:
                navi.stop_move()
                logger.info(
                    f"[BARRIER] Converged: {distance:.1f}cm within "
                    f"±{tolerance}cm of {target_distance}cm"
                )
                return True

            # ---- 3. 异常数据过滤 ----
            if prev_distance is not None and abs(distance - prev_distance) > anomaly_threshold:
                logger.warning(
                    f"[BARRIER] Anomalous jump: {prev_distance:.1f}→{distance:.1f}cm "
                    f"(>{anomaly_threshold}cm), discarding new reading"
                )
                # 丢弃新数据 (distance 回退), 延续上一周期的方向
                distance = prev_distance
            else:
                prev_distance = distance
                # 确定飞行方向
                if distance < target_distance:
                    # 太近 → 远离障碍物 → 机头反方向
                    direction = (navi.current_yaw + 180) % 360
                    logger.debug(
                        f"[BARRIER] Too close ({distance:.1f} < {target_distance}), "
                        f"backward dir={direction:.0f}°"
                    )
                else:
                    # 太远 → 靠近障碍物 → 机头正方向
                    direction = navi.current_yaw
                    logger.debug(
                        f"[BARRIER] Too far ({distance:.1f} > {target_distance}), "
                        f"forward dir={direction:.0f}°"
                    )
                prev_direction = direction

            # ---- 4. 发送速度指令 ----
            navi.move_by_direction(speed=speed, direction_deg=prev_direction)

            # ---- 5. 10Hz 循环 ----
            time.sleep(0.1)

    # ================================================================
    #  [TODO-3] 激光笔控制
    # ================================================================
    def laser_on(self, channel: int = LASER_CHANNEL):
        """
        打开激光笔

        Args:
            channel: 飞控数字输出通道号 (需根据实际接线确认)

        """
        logger.info(f"[LASER] Laser ON (channel={channel})")
        self.fc.set_digital_output(channel, True)

    def laser_off(self, channel: int = LASER_CHANNEL):
        """
        关闭激光笔
        """
        logger.info(f"[LASER] Laser OFF (channel={channel})")
        self.fc.set_digital_output(channel, False)

    # ================================================================
    #  视觉函数 — 封装 vision_of_tf
    # ================================================================

    def detect_qr_code(self) -> bool:
        """调用 vision_of_tf.detect_qrcode_offset 检测 QR 码。

        每次调用打开/关闭摄像头，读取若干帧后返回。

        Returns:
            True 表示当前画面中检测到二维码
        """
        result = vision_of_tf.detect_qrcode_offset(self.qr_camera_index)
        if result is not None:
            # 存储偏移量供 vision_qr_approach 初始参考
            self._last_qr_offset = result
            return True
        return False

    def _detect_qr_number_and_offset(self) -> "Optional[tuple]":
        """打开摄像头逐帧检测 QR 码，返回 (number, z_px, y_px) 或 None。

        使用 vision_of_tf 内部的 _detect_qrcodes 获取解码文本，
        从中解析出整数编号。一次调用完成打开/检测/关闭。
        """
        cap = _open_usb_camera(self.qr_camera_index, 1280, 720)
        try:
            # 预热
            for _ in range(3):
                cap.read()
            # 最多 30 帧 (~3s)
            for _ in range(30):
                ok, frame = cap.read()
                if not ok or frame is None:
                    continue
                detections = _detect_qrcodes(frame)
                if not detections:
                    continue
                image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                selected = _select_nearest_to_image_center(
                    detections, image_center
                )
                z_px, y_px = _center_to_first_y_offset(
                    selected.center, image_center
                )
                try:
                    number = int(selected.text.strip())
                except (ValueError, AttributeError):
                    continue
                self._last_qr_offset = (z_px, y_px)
                return number, z_px, y_px
            return None
        finally:
            cap.release()

    def vision_qr_approach(
        self,
        timeout: float = 30.0,
        speed: float = VISION_APPROACH_SPEED,
        px_thresh: float = VISION_PX_THRESH,
        z_px_thresh: float = VISION_Z_PX_THRESH,
        height_step: float = VISION_HEIGHT_STEP,
    ) -> bool:
        """QR 码双轴视觉位置闭环。

        坐标系约定 (前视摄像头):
            画面水平向左 → 飞机 y正方向
            画面竖直向上 → 飞机 z正方向

        - y_px → 水平偏移: move_by_direction 左/右飞行
        - z_px → 垂直偏移: navi.set_height 上/下调整

        同时满足 |y_px| < px_thresh 且 |z_px| < z_px_thresh 时退出。

        Args:
            timeout: 精调超时 / s
            speed: 水平逼近速度 cm/s
            px_thresh: 水平像素阈值
            z_px_thresh: 垂直像素阈值
            height_step: 单步高度调整量 cm
        """
        navi = self.navi
        logger.info(
            f"[VISION] QR approach: h-speed={speed}cm/s, "
            f"h-thresh={px_thresh}px, v-thresh={z_px_thresh}px, "
            f"v-step={height_step}cm, timeout={timeout}s"
        )

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            # ---- 1. 检测 QR 偏移 ----
            result = vision_of_tf.detect_qrcode_offset(self.qr_camera_index)

            if result is None:
                logger.debug("[VISION] QR lost, hovering")
                navi.stop_move()
                time.sleep(0.3)
                continue

            z_px, y_px = result
            logger.debug(f"[VISION] QR: z={z_px:.0f}px  y={y_px:.0f}px")

            # ---- 2. 判断是否已居中 ----
            y_ok = abs(y_px) < px_thresh
            z_ok = abs(z_px) < z_px_thresh

            if y_ok and z_ok:
                navi.stop_move()
                logger.info(
                    f"[VISION] QR centered: y={y_px:.0f}px, z={z_px:.0f}px"
                )
                return True

            # ---- 3. 水平方向校正 (y_px → y正) ----
            # y_px > 0: QR 在画面左侧 → 飞机向左 (yaw + 90°)
            # y_px < 0: QR 在画面右侧 → 飞机向右 (yaw - 90°)
            if not y_ok:
                if y_px > 0:
                    direction = (navi.current_yaw + 90) % 360
                else:
                    direction = (navi.current_yaw - 90) % 360
                navi.move_by_direction(speed=speed, direction_deg=direction)
            else:
                navi.stop_move()

            # ---- 4. 垂直方向校正 (z_px → z正) ----
            # z_px > 0: QR 在画面上方 → 飞机上升
            # z_px < 0: QR 在画面下方 → 飞机下降
            if not z_ok:
                delta_h = height_step * (1 if z_px > 0 else -1)
                new_h = navi.current_height + delta_h
                # 钳制在合理范围 (不低于 30cm)
                new_h = max(30.0, new_h)
                logger.debug(f"[VISION] QR height adjust: {navi.current_height:.0f} → {new_h:.0f}cm")
                navi.set_height(new_h)

            # ---- 5. 循环间隔 ----
            time.sleep(0.2)

        # 超时
        navi.stop_move()
        logger.warning(f"[VISION] QR approach timeout after {timeout}s")
        return False

    # ================================================================
    #  QR 多轮扫描: 每轮 6 个 QR, 共 4 轮
    #
    #  扫描顺序固定 (2列 × 3行网格):
    #    右上→右下→中下→中上→左上→左下
    #
    #  QR 内容是货物编号 1~24，位置由实际扫描格位决定:
    #    第1轮格位 → A1~A6      第2轮格位 → B1~B6
    #    第3轮格位 → C1~C6      第4轮格位 → D1~D6
    # ================================================================

    def _report_inventory_item(self, qr_number: int, position_name: str):
        """向地面站上报盘点结果，并让 7 颗 LED 全白闪烁约 1 秒。"""
        progress = min(100, round(len(self._qr_positions) * 100 / INVENTORY_TOTAL))
        self.fc.send_ground_status(
            MissionState.RUNNING,
            progress=progress,
            message=f"INV:ITEM:{qr_number}:{position_name}",
        )
        self.fc.set_ground_led_pixels(GROUND_LED_WHITE, brightness=4)
        laser_started = False
        try:
            self.laser_on()
            laser_started = True
            time.sleep(0.5)
        finally:
            try:
                if laser_started:
                    self.laser_off()
            finally:
                time.sleep(0.5)
                self.fc.set_ground_led_pixels(GROUND_LED_OFF, brightness=0)

    def _single_qr_action(
        self, position_name: str, label: str, scan_height: float
    ) -> int:
        """单个 QR 码格位的完整动作序列:
          1. barrier_distance_align()   — 障碍物板距离闭环 75±5cm
          2. vision_qr_approach()      — 双轴视觉精调 (z高低 + y左右)
          3. 解码 QR 货物编号
          4. 记录货物编号对应的实际格位，并上报地面站
          5. 激光笔指示 + 动作

        Returns:
            解码出的 QR 编号 (number), 解码失败则返回 0
        """
        navi = self.navi
        logger.info(
            f"[QR-{label}] height={scan_height:.0f}cm + barrier + vision + laser"
        )

        # Step 0: 每个格位都先回到该层的固定扫描高度，避免视觉精调累积漂移
        navi.set_height(scan_height)
        navi.wait_for_height()
        if abs(navi.current_height - scan_height) >= 8.0:
            logger.error(
                f"[QR-{label}] scan height not reached: "
                f"current={navi.current_height:.1f}cm, target={scan_height:.1f}cm"
            )
            return 0

        # Step 1: 障碍物板距离闭环
        if not self.barrier_distance_align():
            logger.error(f"[QR-{label}] barrier distance not aligned, skipping QR")
            return 0

        # Step 2: 双轴视觉精调 (z + y)
        if not self.vision_qr_approach():
            logger.error(f"[QR-{label}] QR not centered, skipping decode/action")
            return 0

        # Step 3: 解码 QR 编号 (打开摄像头读一帧)
        verified = self._detect_qr_number_and_offset()
        qr_number = verified[0] if verified is not None else 0

        if qr_number == 0:
            logger.warning(f"[QR-{label}] QR decode failed, cannot record")
            return 0

        if not 1 <= qr_number <= INVENTORY_TOTAL:
            logger.warning(f"[QR-{label}] cargo number out of range: {qr_number}")
            return 0
        previous_position = self._qr_positions.get(qr_number)
        if previous_position is not None and previous_position != position_name:
            logger.warning(
                f"[QR-{label}] duplicate cargo #{qr_number}: "
                f"already recorded at {previous_position}"
            )
            return 0

        # Step 4: 货物编号跟随二维码内容，位置跟随当前实际扫描格位
        self._qr_positions[qr_number] = position_name
        self._save_inventory_results()
        logger.info(
            f"[QR-{label}] decoded cargo #{qr_number} → {position_name}, "
            f"map=({navi.current_x:.1f}, {navi.current_y:.1f})"
        )

        # Step 5: 激光笔 0.5 秒 + 地面站全白灯 1 秒
        self._report_inventory_item(qr_number, position_name)

        return qr_number

    def qr_code_action(self):
        """单轮 QR 码扫描 (6 个格位)。

        板面槽位布局（从左到右、从上到下）:
           A1  A2  A3
           A4  A5  A6

        实际蛇形扫描顺序:
           右上(3) → 右下(6) → 中下(5) → 中上(2) → 左上(1) → 左下(4)

        坐标系 (前视摄像头画面 → 飞机):
           画面左 → 飞机 y正    (move_by_direction yaw+90°)
           画面上 → 飞机 z正    (set_height +)

        移动规则:
           ①→②: z负 60cm (下降)     ③→④: z正 60cm (上升)
           ②→③: y正 40cm (左移)     ④→⑤: y正 40cm (左移)
           ⑤→⑥: z负 60cm (下降)

        QR 编号不由扫描顺序决定；二维码文本是货物编号，位置名由当前
        扫描轮次和格位决定。
        """
        navi = self.navi
        round_index = self._qr_round
        round_letter = chr(ord("A") + round_index)

        logger.info(
            f"[MISSION] ╔══ QR Round {round_letter} START ══╗"
        )

        # ---- 扫描点 G1: 右上 → 本面槽位 3 ----
        self._single_qr_action(
            f"{round_letter}3", f"{round_letter}-G1(右上)", QR_UPPER_HEIGHT
        )

        # ---- 扫描点 G2: 右下 → 本面槽位 6 ----
        logger.info(f"[MISSION]   ── z- {QR_GRID_Z_STEP}cm → G2(右下)")
        self._single_qr_action(
            f"{round_letter}6", f"{round_letter}-G2(右下)", QR_LOWER_HEIGHT
        )

        # ---- 扫描点 G3: 中下 → 本面槽位 5 ----
        logger.info(f"[MISSION]   ── y+ {QR_GRID_Y_STEP}cm → G3(中下)")
        yaw_rad = np.deg2rad(navi.current_yaw)
        abs_dx = -QR_GRID_Y_STEP * np.sin(yaw_rad)
        abs_dy = QR_GRID_Y_STEP * np.cos(yaw_rad)
        target = navi.current_point + np.array([abs_dx, abs_dy])
        navi.navigation_to_waypoint(target, wait=True)
        self._single_qr_action(
            f"{round_letter}5", f"{round_letter}-G3(中下)", QR_LOWER_HEIGHT
        )

        # ---- 扫描点 G4: 中上 → 本面槽位 2 ----
        logger.info(f"[MISSION]   ── z+ {QR_GRID_Z_STEP}cm → G4(中上)")
        self._single_qr_action(
            f"{round_letter}2", f"{round_letter}-G4(中上)", QR_UPPER_HEIGHT
        )

        # ---- 扫描点 G5: 左上 → 本面槽位 1 ----
        logger.info(f"[MISSION]   ── y+ {QR_GRID_Y_STEP}cm → G5(左上)")
        yaw_rad = np.deg2rad(navi.current_yaw)
        abs_dx = -QR_GRID_Y_STEP * np.sin(yaw_rad)
        abs_dy = QR_GRID_Y_STEP * np.cos(yaw_rad)
        target = navi.current_point + np.array([abs_dx, abs_dy])
        navi.navigation_to_waypoint(target, wait=True)
        self._single_qr_action(
            f"{round_letter}1", f"{round_letter}-G5(左上)", QR_UPPER_HEIGHT
        )

        # ---- 扫描点 G6: 左下 → 本面槽位 4 ----
        logger.info(f"[MISSION]   ── z- {QR_GRID_Z_STEP}cm → G6(左下)")
        self._single_qr_action(
            f"{round_letter}4", f"{round_letter}-G6(左下)", QR_LOWER_HEIGHT
        )

        # 轮次结束, 递增计数器
        self._qr_round += 1

        # 打印本轮收集到的位置
        round_positions = {
            position_name: cargo_number
            for cargo_number, position_name in self._qr_positions.items()
            if position_name.startswith(round_letter)
        }
        logger.info(
            f"[MISSION] ╚══ QR Round {round_letter} COMPLETE, "
            f"positions: {round_positions} ══╝"
        )

        # 完整性检查: 每轮必须有 6 个不同编号
        if len(round_positions) < QR_SCAN_PER_ROUND:
            logger.warning(
                f"[MISSION] Round {round_letter}: only "
                f"{len(round_positions)}/{QR_SCAN_PER_ROUND} positions recorded!"
            )

    def detect_landing_spot(self) -> bool:
        """调用 vision_of_tf 检测下视画面中的黑色圆形。

        若检测到，同时将偏移量缓存到 self._last_landing_offset，
        供 landing_vision_approach 初始参考。

        Returns:
            True 表示检测到可降落标记
        """
        # 先尝试黑色圆
        result = vision_of_tf.detect_black_circle_offset(self.landing_camera_index)
        if result is not None:
            logger.info(f"[LANDING] Black circle: offset={result}")
            self._landing_offset_type = "circle"
            self._last_landing_offset = result
            return True

        return False

    # ---- landing 视觉字段初始化 ----
    def _ensure_landing_fields(self):
        if not hasattr(self, "_landing_offset_type"):
            self._landing_offset_type: Optional[str] = None
        if not hasattr(self, "_last_landing_offset"):
            self._last_landing_offset: Optional[tuple] = None

    def _detect_landing_offset(self) -> "Optional[tuple]":
        """单次调用检测落地标记偏移, 返回 (x_px, y_px) 或 None。
        内部更新 _last_landing_offset 缓存。"""
        self._ensure_landing_fields()
        result = vision_of_tf.detect_black_circle_offset(self.landing_camera_index)
        if result is not None:
            self._last_landing_offset = result
            return result
        return None

    def landing_vision_approach(
        self,
        timeout: float = 30.0,
        speed: float = LANDING_APPROACH_SPEED,
        px_thresh: float = VISION_PX_THRESH,
    ) -> bool:
        """落点视觉位置闭环。

        下视摄像头坐标系约定:
            画面水平向左  → 飞机 y正方向
            画面竖直向上  → 飞机 x正方向

        vision_of_tf 返回 (x_px, y_px):
            x_px = center_y - target_y → 画面上为正 → 对应飞机前 (x+)
            y_px = center_x - target_x → 画面左为正 → 对应飞机左 (y+)

        合成移动方向:
            direction = yaw + arctan2(y_px, x_px)

        参考: 2022_24_noscreen_nomotor.py vision_approach 的闭环模式。
        """
        navi = self.navi
        logger.info(
            f"[LANDING] Vision approach: speed={speed}cm/s, "
            f"px_thresh={px_thresh}px, timeout={timeout}s"
        )

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            # ---- 1. 检测落点标记 ----
            result = self._detect_landing_offset()

            if result is None:
                logger.debug("[LANDING] Target lost, hovering")
                navi.stop_move()
                time.sleep(0.3)
                continue

            x_px, y_px = result
            logger.debug(f"[LANDING] offset: x={x_px:.0f}px  y={y_px:.0f}px")

            # ---- 2. 已居中 → 退出 ----
            if abs(x_px) < px_thresh and abs(y_px) < px_thresh:
                navi.stop_move()
                logger.info(
                    f"[LANDING] Centered over mark: x={x_px:.0f}px, y={y_px:.0f}px"
                )
                return True

            # ---- 3. 合成移动方向 ----
            # x_px > 0 → 画面上方(机头正前) → 向前分量
            # y_px > 0 → 画面左侧(机头左)   → 向左分量
            # direction = yaw + arctan2(y_px, x_px)
            angle_from_forward = np.rad2deg(np.arctan2(y_px, x_px))
            direction = (navi.current_yaw + angle_from_forward) % 360

            logger.debug(
                f"[LANDING] moving: direction={direction:.0f}° "
                f"(yaw={navi.current_yaw:.0f} + {angle_from_forward:.0f})"
            )

            navi.move_by_direction(speed=speed, direction_deg=direction)
            time.sleep(0.2)

        # 超时
        navi.stop_move()
        logger.warning(f"[LANDING] Vision approach timeout after {timeout}s")
        return False

    def land_after_visual_alignment(self):
        """停止导航输出并直接交给飞控执行降落和锁桨兜底。"""
        navi = self.navi
        fc = self.fc

        navi.stop_move()
        navi.set_navigation_state(False)
        navi.set_keep_height_state(False)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        time.sleep(0.1)
        fc.stablize()
        fc.land()
        if not fc.wait_for_lock():
            logger.warning("[LANDING] Auto lock timeout, forcing lock")
            fc.lock()

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

        # ---------- 定点起飞 ----------
        # 起飞使用飞控内置程控 (PROGRAM_MODE → take_off → HOLD_POS_MODE)，
        # 不依赖 Cartographer / T265 闭环，因此必须放在 Cartographer 初始
        # 化等待之前。起飞过程本身（上升 + 悬停漂移）为在线 SLAM 提供了足
        # 够的运动来完成首个 Submap 构建。
        # 起飞结束后 navigation_flag=True，但 Cartographer 未就绪时 PID
        # 会自动暂停 (available=False)，飞控 HOLD_POS_MODE 维持悬停。
        logger.info(f"[MISSION] Taking off to {self.cruise_height}cm")
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        # ---------- Cartographer 初始化等待 ----------
        # 在线 SLAM 模式下，Cartographer 需要一定运动量（上升 + 漂移）才能
        # 完成首个 Submap 并开始发布 TF 变换。轮询 navi.current_point 直到
        # 位姿非零，表示 transform_established=True 且定位已收敛。
        CART_TIMEOUT = 30.0  # 初始化超时 / s
        logger.info(f"[MISSION] Waiting for Cartographer TF (timeout={CART_TIMEOUT}s)...")
        t0 = time.perf_counter()
        while True:
            time.sleep(1)
            logger.info(f"[MISSION] current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
            if time.perf_counter() - t0 > CART_TIMEOUT:
                raise RuntimeError(
                    "Cartographer TF not established within "
                    f"{CART_TIMEOUT}s. Consider adding small initial "
                    "movement (e.g., rotate or translate 1m) to help "
                    "scan matching converge."
                )
        logger.info(f"[MISSION] Cartographer TF established "
                    f"({time.perf_counter() - t0:.1f}s)")

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
        #  Step H:  下降到 30cm，识别黑色圆形并视觉精调
        # ================================================================
        logger.info(
            f"[MISSION] Step H: descend to {LANDING_SCAN_HEIGHT:.0f}cm, "
            "detect black circle and center"
        )
        navi.set_height(LANDING_SCAN_HEIGHT)
        navi.wait_for_height()
        if abs(navi.current_height - LANDING_SCAN_HEIGHT) >= 8.0:
            raise RuntimeError(
                "Unable to reach landing scan height: "
                f"current={navi.current_height:.1f}cm, "
                f"target={LANDING_SCAN_HEIGHT:.1f}cm"
            )

        landing_spot_found = self.detect_landing_spot()
        if not landing_spot_found:
            logger.info("[MISSION]   Black circle not found in initial scan, retrying")
        if not self.landing_vision_approach():
            raise RuntimeError("Black landing circle was not centered before timeout")

        # ================================================================
        #  Step I:  视觉居中后直接调用飞控降落
        # ================================================================
        logger.info("[MISSION] Step I: Black circle centered, FC landing")
        self.land_after_visual_alignment()
        logger.info("[MISSION] ========== Mission Complete ==========")


# ================================================================
#  __main__: 初始化 → 启动 ROS → 运行任务
#  完全遵循 2025_嵌赛.py 的 ROS 启动框架
# ================================================================
if __name__ == "__main__":
    # ---- 步骤 1: 权限配置 ----
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")   # CP2102 雷达
    rm.chmod("/dev/ttyACM0")   # LX 飞控
    rm.chmod("/dev/video0")    # 下视 USB 摄像头采集节点 (落点识别)
    rm.chmod("/dev/video2")    # 前视 USB 摄像头采集节点 (QR 识别)

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

    # ---- 步骤 9: 等待地面站通过 HC-14 发送 START_MISSION ----
    fc.start_ground_station()
    fc.enable_ground_command_reception()
    logger.info("[MANAGER] Waiting for ground-station start command")
    ground_command = None
    while ground_command is None:
        command = fc.receive_ground_command(timeout=0.5)
        if command is None:
            continue
        try:
            if command.command.command_id == CommandId.START_MISSION:
                fc.prepare_ground_mission()
                fc.accept_ground_command(command)
                ground_command = command
                logger.info("[MANAGER] Ground-station start accepted")
            elif command.command.command_id == CommandId.STOP_MISSION:
                fc.complete_ground_command(command)
            else:
                fc.reject_ground_command(command, RejectReason.UNKNOWN_COMMAND)
        finally:
            fc.ground_command_done()

    fc.enable_ground_telemetry()
    fc.send_ground_status(
        MissionState.RUNNING,
        progress=0,
        message="INV:START",
    )

    mission_error = None
    try:
        mission.run()
    except Exception as e:
        mission_error = e
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

        # 只有降落并锁桨后才向地面站报告最终结果。
        results = mission.inventory_results
        progress = min(100, round(len(results) * 100 / INVENTORY_TOTAL))
        try:
            fc.set_ground_led_pixels(GROUND_LED_OFF, brightness=0)
            if mission_error is not None:
                fc.send_ground_status(
                    MissionState.FAILED,
                    progress=progress,
                    error_code=1,
                    message=f"INV:FAILED:{type(mission_error).__name__}",
                )
                fc.fail_ground_command(ground_command, RejectReason.FC_OFFLINE)
            elif len(results) != INVENTORY_TOTAL:
                fc.send_ground_status(
                    MissionState.FAILED,
                    progress=progress,
                    error_code=2,
                    message=f"INV:INCOMPLETE:{len(results)}",
                )
                fc.fail_ground_command(
                    ground_command, RejectReason.CAMERA_UNAVAILABLE
                )
            else:
                fc.send_ground_status(
                    MissionState.COMPLETED,
                    progress=100,
                    message=f"INV:COMPLETE:{len(results)}",
                )
                fc.complete_ground_command(ground_command)
        except Exception as report_error:
            logger.exception(
                f"[MANAGER] Ground-station final report failed: {report_error}"
            )

    logger.info("[MANAGER] Mission finished")
    fc.close()

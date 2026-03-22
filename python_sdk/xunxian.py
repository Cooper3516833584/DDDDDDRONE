import os, time, threading

os.chdir(os.path.dirname(os.path.abspath(__file__)))

import glob
from datetime import datetime
from typing import Optional, Tuple
import numpy as np
import cv2
from loguru import logger
from FlightController.Solutions import Vision
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation, PARAMS

BASE_POINT = np.array([0.0, 0.0], dtype=float)
LANDING_POINT = np.array([0.0, 0.0], dtype=float)


def get_desktop_dir() -> str:
    """兼容中文“桌面”和英文“Desktop”目录名"""
    home = os.path.expanduser("~")
    desktop_cn = os.path.join(home, "桌面")
    desktop_en = os.path.join(home, "Desktop")
    if os.path.isdir(desktop_cn):
        return desktop_cn
    return desktop_en


def ensure_photos_dir() -> str:
    desktop = get_desktop_dir()
    photos_dir = os.path.join(desktop, "photos")
    os.makedirs(photos_dir, exist_ok=True)
    return photos_dir


def try_open_camera(prefer_size=(800, 600)) -> Tuple[Optional[cv2.VideoCapture], Optional[str]]:
    """尝试打开摄像头"""
    w, h = prefer_size
    for idx in range(4):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            for _ in range(3):
                cap.read()
                time.sleep(0.03)
            return cap, "index:%d" % idx
        try:
            cap.release()
        except Exception:
            pass
    devs = sorted(glob.glob("/dev/video*"))
    for dev in devs:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            for _ in range(3):
                cap.read()
                time.sleep(0.03)
            return cap, dev
        try:
            cap.release()
        except Exception:
            pass
    return None, None


# =========================================================
#  视觉任务线程类 (VisionTask)
#  保障主线程流畅运行，防止雷达/飞控超时
# =========================================================
class VisionTask:
    def __init__(self):
        self.cam = None
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None
        self.is_detected = False
        self.offset_x = 0
        self.offset_y = 0

    def start(self, camera_device):
        self.cam = camera_device
        if self.cam is None or not self.cam.isOpened():
            logger.warning("[VISION] Camera not ready.")
            return
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        logger.info("[VISION] Thread Started.")

    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1)
        logger.info("[VISION] Thread Stopped.")

    def _worker(self):
        while self.running:
            if self.cam is None:
                time.sleep(0.1)
                continue
            ret, frame = self.cam.read()
            if not ret:
                time.sleep(0.1)
                continue

            # 视觉运算
            found, x, y = Vision.find_yellow_code(frame)

            with self.lock:
                self.latest_frame = frame.copy()
                self.is_detected = found
                self.offset_x = x
                self.offset_y = y

            # 限制帧率，给CPU喘息时间
            time.sleep(0.08)

    def get_result(self):
        with self.lock:
            if self.latest_frame is None:
                return False, 0, 0, None
            return self.is_detected, self.offset_x, self.offset_y, self.latest_frame.copy()


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        self.navi = Navigation(fc=fc, radar=radar)
        self.vision_task = VisionTask()
        self.cam = None
        self.cam_name = None
        self.photos_dir = ensure_photos_dir()

    def stop(self):
        self.vision_task.stop()
        try:
            self.navi.stop()
        except Exception:
            pass
        try:
            if self.cam is not None:
                self.cam.release()
        except Exception:
            pass
        logger.info("[MISSION] Mission stopped")

    def setup_camera(self):
        cap, name = try_open_camera(prefer_size=(800, 600))
        if cap is None:
            logger.warning("[CAM] No camera opened.")
            self.cam = None
            self.cam_name = None
            return
        self.cam = cap
        self.cam_name = name
        logger.info("[CAM] Camera opened: %s" % name)

        try:
            self.cam.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)
            logger.info("[CAM] WB Locked (Temp: 4600)")
        except Exception as e:
            logger.warning(f"[CAM] Failed to set WB: {e}")

    def wait_for_radar_pose_ready(self, timeout_sec: float = 8.0):
        """
        Wait until radar pose stream has at least one update.
        This gives a clear failure reason before basepoint calibration.
        """
        deadline = time.perf_counter() + timeout_sec
        while time.perf_counter() < deadline:
            if self.radar.rt_pose_update_event.wait(0.2):
                self.radar.rt_pose_update_event.clear()
                logger.info("[MISSION] Radar pose stream ready")
                return

        if not self.radar.connected:
            raise RuntimeError(
                "Radar data not received from FC. Check radar wiring/forwarding on flight controller."
            )
        raise RuntimeError(
            "Radar connected but pose is not updated. Check radar packet stream and solver parameters."
        )

    def snap_from_frame(self, frame, tag: str):
        if frame is None:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        save_path = os.path.join(self.photos_dir, "%s_%s.jpg" % (tag, ts))
        cv2.imwrite(save_path, frame)
        logger.info("[CAM] Snapshot saved: %s" % save_path)

    def snap_at_point(self, tag: str):
        if self.cam is None:
            return
        frame_to_save = None
        if self.vision_task.running:
            _, _, _, frame_to_save = self.vision_task.get_result()
        else:
            for _ in range(5):
                self.cam.grab()
                time.sleep(0.01)
            ret, tmp = self.cam.read()
            if ret:
                frame_to_save = tmp

        if frame_to_save is not None:
            self.snap_from_frame(frame_to_save, tag)
        else:
            logger.warning(f"[CAM] Failed to snap {tag}")

    def run(self):
        fc = self.fc
        radar = self.radar
        navi = self.navi

        navigation_speed = 25
        circle_speed = 15
        cruise_height = 110
        vertical_speed = 20
        R = 77   # 安全半径（cm）

        # 绕杆参数（配合 Navigation.py 的修复：更密轨迹 + 更严到点阈值）
        CIRCLE_DT = 0.2
        CIRCLE_POS_THRES = 10  # cm，建议 8~12

        # ---------- 启动导航 ----------
        # Diagnostic setting for low-rate radar streams: trigger map/pose resolve on every radar update.
        PARAMS.RADAR_SKIP = 1
        logger.warning("[MISSION] PARAMS.RADAR_SKIP overridden to 1 for radar diagnostics")

        navi.set_navigation_speed(navigation_speed)
        navi.set_vertical_speed(vertical_speed)

        try:
            navi.start(mode="radar")
        except TypeError:
            navi.start("radar")

        logger.info("[MISSION] Navigation started (radar)")

        # Wait for real radar pose updates instead of blind sleeping.
        self.wait_for_radar_pose_ready(timeout_sec=8.0)

        # ---------- 校准基地点 ----------
        navi.calibrate_basepoint(wait=False)

        # ---------- 起飞 ----------
        fc.set_action_log(True)
        logger.info("[MISSION] Mission Started")
        time.sleep(3.0)

        # 注意：pointing_takeoff 已在 Navigation.py 中修复为“低高度先锁点再爬高”
        navi.pointing_takeoff(BASE_POINT, cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(1)

        # 启动相机和视觉线程
        self.setup_camera()
        if self.cam is not None:
            self.vision_task.start(self.cam)

        # ---------- 雷达扫杆 ----------
        logger.info("[MISSION] Scanning poles...")
        radar.register_map_func(
            radar.map.find_nearest_with_ext_point_opt,
            from_=260, to_=359, num=2
        )
        time.sleep(5)

        if (not radar.map_func_results) or (len(radar.map_func_results[0]) < 2):
            raise RuntimeError("Radar did not find 2 poles!")

        p1 = radar.map_func_results[0][0]
        p2 = radar.map_func_results[0][1]
        p1.distance /= 10.0
        p2.distance /= 10.0

        xy1 = p1.to_xy()
        xy2 = p2.to_xy()
        p1x, p1y = float(xy1[0]), float(xy1[1])
        p2x, p2y = float(xy2[0]), float(xy2[1])

        logger.info("[MISSION] Pole A xy(cm): %s" % (xy1,))
        logger.info("[MISSION] Pole B xy(cm): %s" % (xy2,))

        # 定义 A/B 的中心点
        Pole_A_Center = np.array([p1x, p1y], dtype=float)
        Pole_B_Center = np.array([p2x, p2y], dtype=float)

        # 计算 A/B 的 前方安全点 (y - R)
        A_safe = np.array([p1x, p1y - R], dtype=float)
        B_safe = np.array([p2x, p2y - R], dtype=float)

        # 计算 A/B 的 后方对称点 (y + R) - 用于回程
        A_back = np.array([p1x, p1y + R], dtype=float)
        B_back = np.array([p2x, p2y + R], dtype=float)

        # ==========================================
        # 1. 前往 A 安全点 (起点)
        # ==========================================
        logger.info(f"[MISSION] Go A_safe: {A_safe}")
        navi.navigation_to_waypoint(A_safe, wait=True)
        logger.info("[MISSION] Reach A_safe")
        self.snap_at_point("A_safe")

        # ==========================================
        # 2. 前往 B 安全点 (带视觉检测)
        # ==========================================
        navi.set_navigation_speed(navigation_speed)
        logger.info(f"[MISSION] Go B_safe with detection: {B_safe}")

        start_pos_of_leg = np.array([navi.current_x, navi.current_y])
        navi.navigation_to_waypoint(B_safe, wait=False)
        has_snapped_yellow = False

        while True:
            curr_pos = np.array([navi.current_x, navi.current_y])
            dist_from_start = np.linalg.norm(curr_pos - start_pos_of_leg)
            dist_to_B = np.linalg.norm(curr_pos - B_safe)

            if dist_to_B < 15:
                logger.info("[MISSION] Arrived at B_safe area")
                break

            # 起飞保护（避免刚出发时误触发中途停）
            if dist_from_start < 15:
                time.sleep(0.1)
                continue

            found, x_off, y_off, current_frame = self.vision_task.get_result()

            if found and not has_snapped_yellow and (current_frame is not None):
                h, w = current_frame.shape[:2]
                x_limit = w * 0.1
                y_limit = h * 0.3

                if abs(x_off) < x_limit and abs(y_off) < y_limit:
                    logger.info("[MISSION] Yellow object detected! Hovering...")
                    navi.navigation_stop_here()
                    self.snap_from_frame(current_frame, "Yellow_Mid_Target")
                    has_snapped_yellow = True
                    time.sleep(1.0)
                    logger.info("[MISSION] Resuming...")
                    navi.navigation_to_waypoint(B_safe, wait=False)

            time.sleep(0.1)

        navi.navigation_to_waypoint(B_safe, wait=True)
        logger.info("[MISSION] Reach B_safe")
        self.snap_at_point("B_safe")

        # ==========================================
        # 3. 绕 B 半圈 (外侧) -> 到达 B_back
        # ==========================================
        navi.set_navigation_speed(circle_speed)
        time.sleep(0.5)

        logger.info("[MISSION] Circle B Half (to Back side)")
        navi.navigation_around_waypoint(
            waypoint=Pole_B_Center,
            wait=True,
            degree=np.pi,                  # 半圈
            mode="counterclockwise",        # 下 -> 右 -> 上
            radius=R,                      # 固定安全半径，避免贴杆
            dt=CIRCLE_DT,                  # 更密轨迹点
            pos_thres=CIRCLE_POS_THRES,    # 更严到点阈值，减少切弦
        )
        logger.info("[MISSION] Reached B Back side")

        # ==========================================
        # 4. 直线飞行: B后方 -> A后方
        # ==========================================
        navi.set_navigation_speed(navigation_speed)
        logger.info(f"[MISSION] Fly linear to A Back: {A_back}")

        navi.navigation_to_waypoint(A_back, wait=True)
        logger.info("[MISSION] Reached A Back side")

        # ==========================================
        # 5. 绕 A 半圈 (外侧) -> 回到 A_safe
        # ==========================================
        navi.set_navigation_speed(circle_speed)
        time.sleep(0.5)

        logger.info("[MISSION] Circle A Half (to Front side)")
        navi.navigation_around_waypoint(
            waypoint=Pole_A_Center,
            wait=True,
            degree=np.pi,                  # 半圈
            mode="counterclockwise",        # 上 -> 左 -> 下
            radius=R,
            dt=CIRCLE_DT,
            pos_thres=CIRCLE_POS_THRES,
        )
        logger.info("[MISSION] Returned to A_safe")

        # ==========================================
        # 6. 返回起飞点并降落
        # ==========================================
        navi.set_navigation_speed(navigation_speed)
        time.sleep(0.5)

        logger.info("[MISSION] Return to Base and Landing")
        navi.pointing_landing(LANDING_POINT)


if __name__ == "__main__":
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyACM0", print_state=False)
    fc.wait_for_connection()

    radar = LD_Radar()
    radar.start()
    time.sleep(0.5)

    mission = Mission(fc, radar)

    try:
        mission.run()
    except Exception:
        logger.exception("[MANAGER] Mission Failed")
    finally:
        mission.stop()
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] Auto Landing (Emergency)")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                for _ in range(100):
                    if fc.state.alt_add.value < 10:
                        break
                    time.sleep(0.1)
                fc.lock()
        except Exception:
            logger.exception("[MANAGER] Auto Landing Failed")

    logger.info("[MANAGER] Mission finished")
    fc.close()

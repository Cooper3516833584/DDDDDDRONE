import os, sys, time, threading, re

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
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation

BASE_POINT = np.array([0.0,0.0], dtype = float)
LANDING_POINT = np.array([0.0,0.0], dtype=float)

target_points = {
    "1": np.array([50.0, 275.0], dtype = float),
    "2": np.array([200.0, 125.0], dtype = float),
    "3": np.array([275.0, 200.0], dtype = float),
    "4": np.array([350.0, -25.0], dtype = float),
    "5": np.array([350.0, 275.0], dtype = float),
    "6": np.array([275.0, 50.0], dtype = float),
    "7": np.array([125.0, 50.0], dtype = float),
    "8": np.array([125.0, 200.0], dtype = float),
    "9": np.array([50.0, 125.0], dtype = float),
    "10": np.array([200.0, -25.0], dtype = float),
    "11": np.array([200.0, 275.0], dtype = float),
    "12": np.array([350.0, 125.0], dtype = float)
}

target_features = {
    "1": {"name": "1 red tri",  "color": "#FFFF0000"},
    "2": {"name": "2 red tri",  "color": "#FFFF0000"},
    "3": {"name": "3 blu tri",  "color": "#FF0000FF"},
    "4": {"name": "4 blu tri",  "color": "#FF0000FF"},
    "5": {"name": "5 red cir",  "color": "#FFFF0000"},
    "6": {"name": "6 red cir",  "color": "#FFFF0000"},
    "7": {"name": "7 blu cir",  "color": "#FF0000FF"},
    "8": {"name": "8 blu cir",  "color": "#FF0000FF"},
    "9": {"name": "9 red squ",  "color": "#FFFF0000"},
    "10": {"name": "10 red squ",  "color": "#FFFF0000"},
    "11": {"name": "11 blu squ",  "color": "#FF0000FF"},
    "12": {"name": "12 blu squ",  "color": "#FF0000FF"}
}

def try_open_camera(prefer_size = (800, 600)) -> Tuple[Optional[cv2.VideoCapture], Optional[str]]:
    """尝试打开摄像头，优先使用prefer_size指定的分辨率 """
    w, h = prefer_size
    
    #索引方式探测
    for i in range(4):
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            continue
        #尝试设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        for _ in range(3):
            cap.read()
            time.sleep(0.03)
        return cap, f"index:{i}"
    
    #Linux专有路径
    devs = sorted(glob.glob("/dev/video*"))
    for dev in devs:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            continue
        #尝试设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        for _ in range(3):
            cap.read()
            time.sleep(0.03)
        return cap, dev
    
    raise Exception("Failed to open camera")

class VisionTask():
    def __init__(self):
        self.cam = None
        self._stop_flag = False

    def get_target_angle(self) -> Optional[float]:
        """
        返回目标点在匿名坐标系x,y平面上与坐标原点连线的角度 / deg
        0度为x轴正方向，顺时针为正
        返回None表示未检测到目标
        """
        # TODO: 实现视觉检测目标角度
        return None

    def is_close_to_target(self) -> bool:
        """
        判断是否足够靠近目标点
        返回True表示已足够靠近
        """
        # TODO: 实现视觉判断是否靠近目标
        return False

    def get_target_by_vision(self, time_out = 120) -> Optional[Tuple[str, str]]:
        """
        通过视觉检测目标点，返回目标点编号字符串，如 ("3", "10")
        time_out: 超时时间/s，超过后返回None
        """
        # TODO: 实现视觉检测目标点
        return None

    def stop(self):
        self._stop_flag = True

class Mission(object):
    def __init__(self, fc:FC_Controller, radar:LD_Radar):
        self.fc = fc
        self.radar = radar
        self.navi = Navigation(fc = fc, radar = radar)
        self.vision_task = VisionTask()
        self.cam = None 
        self.cam_name = None
        self._emergency_stop = threading.Event()

        
    def stop(self):
        self._emergency_stop.set()
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

    def _check_emergency(self):
        """检查是否收到紧急停止信号，如果收到则抛出异常中断任务"""
        if self._emergency_stop.is_set():
            raise RuntimeError("Emergency stop triggered")

    def _vision_approach_thread(self, modify_speed=15, freq=10):
        """
        视觉逼近线程：持续调用get_target_angle获取方向，调用move_by_direction移动
        freq: 调用频率/Hz，不小于5Hz
        """
        dt = 1.0 / max(freq, 5)
        while not self.vision_task._stop_flag:
            angle = self.vision_task.get_target_angle()
            if angle is not None:
                self.navi.move_by_direction(speed=modify_speed, direction_deg=angle)
            else:
                # 未检测到目标时悬停
                self.navi.stop_move()
                logger.debug("[VISION] No target detected, hovering")
            time.sleep(dt)

    def _vision_check_thread(self, freq=20, timeout=60):
        """
        视觉判断线程：持续调用is_close_to_target判断是否足够靠近
        一旦靠近，停止逼近线程，转为悬停
        freq: 检测频率/Hz（与视觉帧率一致）
        timeout: 超时时间/s
        """
        dt = 1.0 / freq
        t0 = time.perf_counter()
        while not self.vision_task._stop_flag:
            if self.vision_task.is_close_to_target():
                logger.info("[VISION] Close enough to target, stopping approach")
                self.vision_task._stop_flag = True
                self.navi.stop_move()
                return
            if time.perf_counter() - t0 > timeout:
                logger.warning("[VISION] Vision approach timeout, stopping")
                self.vision_task._stop_flag = True
                self.navi.stop_move()
                return
            time.sleep(dt)

    def vision_approach(self, modify_speed=15, approach_freq=10, check_freq=20, timeout=60):
        """
        视觉精调位置：两个线程并行
        - 逼近线程：以approach_freq频率获取目标方向并移动
        - 判断线程：以check_freq频率判断是否靠近目标
        靠近后自动停止逼近，转为悬停
        """
        self.vision_task._stop_flag = False
        logger.info("[MISSION] Starting vision approach")

        t_approach = threading.Thread(
            target=self._vision_approach_thread,
            args=(modify_speed, approach_freq),
            daemon=True,
        )
        t_check = threading.Thread(
            target=self._vision_check_thread,
            args=(check_freq, timeout),
            daemon=True,
        )

        t_approach.start()
        t_check.start()

        # 等待判断线程结束（无论成功或超时）
        t_check.join()
        # 确保逼近线程也结束
        self.vision_task._stop_flag = True
        t_approach.join(timeout=2.0)

        logger.info("[MISSION] Vision approach finished")
        
    def setup_camera(self):
        try:
            cap, name = try_open_camera()
            self.cam = cap
            self.cam_name = name
            logger.info(f"[CAM] Camera opened: {name}")
            
        except Exception as e:
            logger.warning(f"[CAM] Failed to open camera: {e}")
            self.cam = None
            self.cam_name = None
            return False
        
        return True
    
    def _motor_step_task(self, revolutions: float):
            self.fc.step_motor_rotate(revolutions, 1)
            time.sleep(6.0)

    def _pwm_output_task(self, channel: int, value: int, stop_event: threading.Event, interval: float = 0.05):
        """
        持续发送PWM输出，直到stop_event被设置

        channel: PWM通道号
        value: PWM输出值
        stop_event: 停止事件，set()后停止发送
        interval: 发送间隔/s
        """
        while not stop_event.is_set():
            self.fc.set_PWM_output(channel, value)
            stop_event.wait(interval)
    
    def run(self, target1: str, target2: str):
        fc = self.fc
        radar = self.radar
        navi = self.navi
        
        navi_speed = 25
        vertical_speed = 20
        cruise_height = 150
        target_height = 80
        modify_speed = 15
        revolutions_60cm = 0.5
        
        # --------启动导航--------
        self._check_emergency()
        navi.set_navigation_speed(navi_speed)
        navi.set_vertical_speed(vertical_speed)
        
        try:
            navi.start(mode = "radar")
        except TypeError:
            navi.start("radar")
        
        logger.info("[MISSION] Navigation started (radar)")
        
        # 给雷达 5-6 秒预热防止超时
        for _ in range(60):
            self._check_emergency()
            time.sleep(0.1)
        
        # --------设置基准点--------
        self._check_emergency()
        navi.calibrate_basepoint()
        
        # --------起飞--------
        self._check_emergency()
        fc.set_action_log(True)
        logger.info("[MISSION] Taking off")
        
        navi.pointing_takeoff(BASE_POINT, cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(1.0)
        
        #--------飞往目标点1--------
        self._check_emergency()
        logger.info(f"[MISSION] Flying to target point 1 ({target_points[target1][0]}, {target_points[target1][1]})")
        navi.navigation_to_waypoint(target_points[target1], wait = True)
        logger.info("[MISSION] Arrived at target point 1 roughly")
        
        #--------悬停，调整高度至80cm--------
        self._check_emergency()
        navi.adjust_height_and_hover(target_height)
        
        #--------视觉精调位置--------
        self._check_emergency()
        logger.info("[MISSION] Starting vision-based fine approach for target 1")
        self.vision_approach(modify_speed=modify_speed, approach_freq=10, check_freq=20, timeout=60)
        logger.info("[MISSION] Vision approach done, hovering at target 1")

        #--------并行执行电机控制和蜂鸣器发声--------
        self._check_emergency()
        logger.info("[MISSION] Starting parallel motor control for target 1")

        t_motor = threading.Thread(target=self._motor_step_task, args=(revolutions_60cm,), daemon=True)
        _pwm_stop = threading.Event()
        t_pwm = threading.Thread(target=self._pwm_output_task, args=(3, 100, _pwm_stop), daemon=True)
        t_motor.start()
        t_pwm.start()
        logger.info("[MISSION] Buzzer on")
        t_motor.join()
        _pwm_stop.set()
        t_pwm.join(timeout=2.0)
        logger.info("[MISSION] Parallel motor control done for target 1")
        logger.info("[MISSION] Buzzer off")
        
        #--------悬停，调整高度至150cm--------
        self._check_emergency()
        navi.adjust_height_and_hover(cruise_height)
        navi.set_navigation_speed(navi_speed)
        time.sleep(0.5)
        
        #--------飞往目标点2--------
        self._check_emergency()
        logger.info(f"[MISSION] Flying to target point 2 ({target_points[target2][0]}, {target_points[target2][1]})")
        navi.navigation_to_waypoint(target_points[target2], wait=True)
        logger.info("[MISSION] Arrived at target point 2 roughly")

        #--------悬停，调整高度至80cm--------
        self._check_emergency()
        navi.adjust_height_and_hover(target_height)

        #--------视觉精调位置--------
        self._check_emergency()
        logger.info("[MISSION] Starting vision-based fine approach to target 2")
        self.vision_approach(modify_speed=modify_speed, approach_freq=10, check_freq=20, timeout=60)
        logger.info("[MISSION] Vision approach done, hovering at target 2")

        #--------并行执行电机控制和蜂鸣器发声--------
        self._check_emergency()
        logger.info("[MISSION] Starting parallel motor control for target 2")

        t_motor2 = threading.Thread(target=self._motor_step_task, args=(revolutions_60cm,), daemon=True)
        _pwm_stop2 = threading.Event()
        t_pwm2 = threading.Thread(target=self._pwm_output_task, args=(3, 100, _pwm_stop2), daemon=True)
        t_motor2.start()
        t_pwm2.start()
        logger.info("[MISSION] Buzzer on")
        t_motor2.join()
        _pwm_stop2.set()
        t_pwm2.join(timeout=2.0)
        logger.info("[MISSION] Parallel motor control done for target 2")
        logger.info("[MISSION] Buzzer off")
        
        #--------悬停，调整高度至150cm--------
        self._check_emergency()
        navi.adjust_height_and_hover(cruise_height)
        navi.set_navigation_speed(navi_speed)
        time.sleep(0.5)
        
        #--------返航--------
        self._check_emergency()
        logger.info("[MISSION] Returning to landing point")
        navi.navigation_to_waypoint(LANDING_POINT, wait=True)
        logger.info("[MISSION] Arrived at landing point")
        
        #--------降落--------
        self._check_emergency()
        logger.info("[MISSION] Landing")
        navi.pointing_landing(LANDING_POINT)
        logger.info("[MISSION] Landed")
        
def wait_for_targets(screen: UARTScreen, count: int = 2, timeout: float = 120.0) -> list:
    """
    监控串口屏数据，提取格式为 "targetN" 的字符串中的数字N (1-12)
    返回提取到的数字字符串列表，如 ["3", "10"]
    count: 需要提取的目标数量
    timeout: 总超时秒数
    """
    targets = []
    t0 = time.perf_counter()
    pattern = re.compile(r"target(\d+)")
    
    while len(targets) < count:
        remaining = timeout - (time.perf_counter() - t0)
        if remaining <= 0:
            logger.warning(f"[SCREEN] 等待串口屏目标数据超时，已获取: {targets}")
            break
        
        result = screen.wait_for_data(timeout=min(remaining, 5.0))
        if result is None:
            continue
        
        dtype, value = result
        if dtype == "str" and isinstance(value, str):
            match = pattern.search(value)
            if match:
                num = match.group(1)
                n = int(num)
                if n == 0:
                    return []
                elif 1 <= n <= 12:
                    targets.append(num)
                    logger.info(f"[SCREEN] 检测到目标: target{num} (第{len(targets)}/{count}个)")
                else:
                    logger.debug(f"[SCREEN] target数字超出范围1-12: {num}")
            else:
                logger.debug(f"[SCREEN] 忽略非目标字符串: {value}")
        else:
            logger.debug(f"[SCREEN] 忽略非字符串数据: {dtype}={value}")
    
    return targets

def main():
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyAMA0", print_state=False)
    fc.wait_for_connection()
    
    screen = UARTScreen(fc)
    radar = None
    mission = None
    mission_thread = None
    acquire_thread = None
    
    # 串口屏指令正则
    pat_tarset = re.compile(r"tarset(\d+)")
    pat_target = re.compile(r"target(\d+)")
    EMERGENCY_STOP_CMD = "mission stop"
    
    # 目标点获取状态
    acquiring_targets = False          # 是否正在获取目标点
    acquire_mode = None                # "screen" 或 "vision"
    collected_targets = []             # 已收集的目标点
    vision_result = None               # 视觉获取的结果
    _stop_acquire = threading.Event()  # 用于停止目标点获取过程
    
    def emergency_land():
        """紧急降落"""
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] Emergency landing")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                for _ in range(100):
                    if fc.state.alt_add.value < 10:
                        break
                    time.sleep(0.1)
                fc.lock()
        except Exception as e:
            logger.exception(f"[MANAGER] Emergency landing failed: {e}")
    
    def run_mission_in_thread(mission: Mission, t1: str, t2: str):
        """在线程中执行 mission，结束后自动清理"""
        nonlocal mission_thread
        try:
            mission.run(t1, t2)
        except RuntimeError as e:
            if "Emergency stop" in str(e):
                logger.warning("[MANAGER] Mission aborted by emergency stop")
            else:
                raise
        except Exception as e:
            logger.exception(f"[MANAGER] Mission failed with exception: {e}")
        finally:
            mission.stop()
            try:
                if fc.state.unlock.value:
                    logger.warning("[MANAGER] Auto landing after mission")
                    fc.set_flight_mode(fc.PROGRAM_MODE)
                    fc.stablize()
                    fc.land()
                    for _ in range(100):
                        if fc.state.alt_add.value < 10:
                            break
                        time.sleep(0.1)
                    fc.lock()
            except Exception as e:
                logger.exception(f"[MANAGER] Auto landing failed: {e}")
        logger.info("[MANAGER] Mission thread finished")
    
    def acquire_targets_by_screen():
        """阻塞式通过串口屏收集目标点，结果存入 collected_targets"""
        nonlocal collected_targets
        collected_targets = []
        logger.info("[MANAGER] 进入串口屏目标收集模式，请发送 targetN (N=1~12)")
        
        while len(collected_targets) < 2 and not _stop_acquire.is_set():
            result = screen.wait_for_data(timeout=1.0)
            if result is None:
                continue
            
            dtype, value = result
            if dtype != "str" or not isinstance(value, str):
                continue
            
            value = value.strip()
            
            # 在收集过程中也能响应紧急停止
            if value == EMERGENCY_STOP_CMD:
                logger.warning("[MANAGER] 目标收集过程中收到紧急停止")
                _stop_acquire.set()
                break
            
            match = pat_target.search(value)
            if match:
                num = match.group(1)
                n = int(num)
                if n == 0:
                    collected_targets = []
                    logger.info("[MANAGER] 清空已收集目标点")
                elif 1 <= n <= 12:
                    collected_targets.append(num)
                    logger.info(f"[MANAGER] 收集到目标: target{num} (第{len(collected_targets)}/2个)")
                else:
                    logger.debug(f"[MANAGER] target数字超出范围1-12: {num}")
    
    def acquire_targets_by_vision():
        """阻塞式通过视觉获取目标点，结果存入 vision_result"""
        nonlocal vision_result
        vision_result = None
        logger.info("[MANAGER] 进入视觉目标识别模式...")
        
        vt = VisionTask()
        
        def _vision_worker():
            nonlocal vision_result
            try:
                result = vt.get_target_by_vision(time_out=120)
                if result is not None:
                    vision_result = result
            except Exception as e:
                logger.exception(f"[MANAGER] 视觉目标识别异常: {e}")
        
        t = threading.Thread(target=_vision_worker, daemon=True)
        t.start()
        
        # 等待视觉结果或停止信号
        while t.is_alive() and not _stop_acquire.is_set():
            t.join(timeout=0.5)
        
        if _stop_acquire.is_set():
            vt.stop()
            t.join(timeout=3.0)
            logger.warning("[MANAGER] 视觉目标识别被停止")
            return
        
        if vision_result is not None:
            logger.info(f"[MANAGER] 视觉识别到目标: {list(vision_result)}")
        else:
            logger.warning("[MANAGER] 视觉目标识别超时或未识别到目标")
    
    def start_mission_with_targets(t1: str, t2: str):
        """使用已收集的目标点启动 mission，启动前有20秒缓冲期"""
        nonlocal mission, mission_thread, radar
        
        if t1 == t2:
            logger.error(f"[MANAGER] 两个目标相同(t1={t1}, t2={t2})，请重新选择")
            return
        
        # 20秒缓冲期，期间监听串口屏是否收到 mission stop
        screen.send_command("page 2")  # 串口屏切换到已找到目标点的页面
        screen.set_widget_value("text0.txt", target_features[t1]["name"]) # 设置目标1名称
        screen.set_widget_value("text1.txt", target_features[t2]["name"]) # 设置目标2名称
        screen.set_widget_value("text0.bgColor", target_features[t1]["color"]) # 设置目标1颜色
        screen.set_widget_value("text1.bgColor", target_features[t2]["color"]) # 设置目标2颜色
        
        logger.info("[MANAGER] 20秒倒计时，发送 'mission stop' 可取消任务...")
        t0 = time.perf_counter()
        aborted = False
        last_logged = -1
        while time.perf_counter() - t0 < 20.0:
            remaining = 20.0 - (time.perf_counter() - t0)
            result = screen.wait_for_data(timeout=min(remaining, 2.0))
            if result is not None:
                dtype, value = result
                if dtype == "str" and isinstance(value, str):
                    value = value.strip()
                    if value == EMERGENCY_STOP_CMD:
                        logger.warning("[MANAGER] 缓冲期内收到停止指令，取消任务启动")
                        aborted = True
                        break
            elapsed = int(time.perf_counter() - t0)
            if elapsed % 5 == 0 and elapsed > 0 and elapsed != last_logged:
                last_logged = elapsed
                logger.info(f"[MANAGER] 倒计时: {20 - elapsed}秒后启动任务")
        
        if aborted:
            return
        
        screen.send_command("page 1")  # 切回主页面
        logger.info(f"[MANAGER] 启动 Mission: target1={t1}, target2={t2}")
        
        if radar is None:
            radar = LD_Radar()
            radar.start()
            time.sleep(0.5)
        
        mission = Mission(fc, radar)
        mission_thread = threading.Thread(
            target=run_mission_in_thread,
            args=(mission, t1, t2),
            daemon=True,
        )
        mission_thread.start()
    
    logger.info("[MANAGER] 等待串口屏指令...")
    logger.info(f"[MANAGER] 支持的指令: 'tarset1'(串口屏选目标) / 'tarset2'(视觉选目标) / '{EMERGENCY_STOP_CMD}'")
    
    while True:
        # 如果正在通过串口屏获取目标点，由子线程自己处理串口数据，主线程等待
        if acquiring_targets and acquire_mode == "screen":
            time.sleep(0.1)
            continue
        
        result = screen.wait_for_data(timeout=5.0)
        if result is None:
            continue
        
        dtype, value = result
        if dtype != "str" or not isinstance(value, str):
            continue
        
        value = value.strip()
        
        # ---- 紧急停止 ----
        if value == EMERGENCY_STOP_CMD:
            logger.warning("[MANAGER] 收到紧急停止指令!")
            
            # 停止目标获取过程
            if acquiring_targets:
                _stop_acquire.set()
                if acquire_thread is not None and acquire_thread.is_alive():
                    acquire_thread.join(timeout=5.0)
                acquiring_targets = False
                acquire_mode = None
                collected_targets.clear()
                logger.info("[MANAGER] 目标获取过程已停止并清空")
            
            # 停止运行中的任务
            if mission is not None:
                mission.stop()
                if mission_thread is not None and mission_thread.is_alive():
                    mission_thread.join(timeout=10.0)
                emergency_land()
                mission = None
                mission_thread = None
            
            _stop_acquire.clear()
            continue
        
        # ---- 选择目标获取模式 ----
        match = pat_tarset.search(value)
        if match:
            mode = match.group(1)
            
            if mission_thread is not None and mission_thread.is_alive():
                logger.warning("[MANAGER] 当前有任务正在运行，请先发送 'mission stop'")
                continue
            
            if acquiring_targets:
                logger.warning("[MANAGER] 正在获取目标点中，请先发送 'mission stop' 取消")
                continue
            
            _stop_acquire.clear()
            collected_targets.clear()
            vision_result = None
            
            if mode == "1":
                # 串口屏模式：在子线程中阻塞收集
                acquire_mode = "screen"
                acquiring_targets = True
                
                def _screen_acquire_wrapper():
                    nonlocal acquiring_targets
                    try:
                        acquire_targets_by_screen()
                    finally:
                        acquiring_targets = False
                
                acquire_thread = threading.Thread(target=_screen_acquire_wrapper, daemon=True)
                acquire_thread.start()
                
                # 定时检查收集是否完成
                def _check_screen_acquire_done():
                    if acquire_thread is not None and acquire_thread.is_alive():
                        threading.Timer(0.5, _check_screen_acquire_done).start()
                    else:
                        if _stop_acquire.is_set():
                            return
                        if len(collected_targets) >= 2:
                            start_mission_with_targets(collected_targets[0], collected_targets[1])
                            collected_targets.clear()
                        else:
                            logger.warning(f"[MANAGER] 目标点不足(需2个，获{len(collected_targets)}个)")
                
                _check_screen_acquire_done()
            
            elif mode == "2":
                # 视觉模式：在子线程中阻塞获取
                acquire_mode = "vision"
                acquiring_targets = True
                
                def _vision_acquire_wrapper():
                    nonlocal acquiring_targets, collected_targets
                    try:
                        acquire_targets_by_vision()
                        if not _stop_acquire.is_set() and vision_result is not None:
                            collected_targets = list(vision_result)
                            if len(collected_targets) >= 2:
                                start_mission_with_targets(collected_targets[0], collected_targets[1])
                            else:
                                logger.warning(f"[MANAGER] 视觉目标点不足(需2个，获{len(collected_targets)}个)")
                    finally:
                        acquiring_targets = False
                
                acquire_thread = threading.Thread(target=_vision_acquire_wrapper, daemon=True)
                acquire_thread.start()
            
            else:
                logger.warning(f"[MANAGER] 未知目标获取模式: tarset{mode} (仅支持 tarset1 或 tarset2)")
            
            continue
        
        logger.debug(f"[MANAGER] 忽略未知指令: {value}")

if __name__ == "__main__":
    main()
"""
串口屏管理模块
从 2022_24.py 中提取的串口屏相关功能，包括：
- 目标点收集（串口屏模式）
- 任务启动与倒计时缓冲
- 指令解析与分发
- 紧急停止处理
"""

import re
import time
import threading
from typing import Optional, Tuple, List, Callable

from loguru import logger
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from FlightController.Components.UartScreen import UARTScreen


# ---------- 目标特征数据 ----------

target_features = {
    "1":  {"name": "1 red tri",  "color": "#FFFF0000"},
    "2":  {"name": "2 red tri",  "color": "#FFFF0000"},
    "3":  {"name": "3 blu tri",  "color": "#FF0000FF"},
    "4":  {"name": "4 blu tri",  "color": "#FF0000FF"},
    "5":  {"name": "5 red cir",  "color": "#FFFF0000"},
    "6":  {"name": "6 red cir",  "color": "#FFFF0000"},
    "7":  {"name": "7 blu cir",  "color": "#FF0000FF"},
    "8":  {"name": "8 blu cir",  "color": "#FF0000FF"},
    "9":  {"name": "9 red squ",  "color": "#FFFF0000"},
    "10": {"name": "10 red squ",  "color": "#FFFF0000"},
    "11": {"name": "11 blu squ",  "color": "#FF0000FF"},
    "12": {"name": "12 blu squ",  "color": "#FF0000FF"},
}


class ScreenManager:
    """
    串口屏管理器，负责：
    - 监听串口屏指令
    - 通过串口屏收集目标点
    - 显示任务信息与倒计时
    - 紧急停止信号处理
    """

    EMERGENCY_STOP_CMD = "mission stop"

    def __init__(self, screen: "UARTScreen"):
        self.screen = screen

        # 指令正则
        self._pat_tarset = re.compile(r"tarset(\d+)")
        self._pat_target = re.compile(r"target(\d+)")

        # 目标收集状态
        self.acquiring_targets = False
        self.acquire_mode: Optional[str] = None   # "screen" 或 "vision"
        self.collected_targets: List[str] = []
        self._stop_acquire = threading.Event()

        # 外部回调
        self.on_targets_collected: Optional[Callable[[str, str], None]] = None
        self.on_vision_acquire_requested: Optional[Callable[[], None]] = None
        self.on_emergency_stop: Optional[Callable[[], None]] = None

        # 任务状态（由外部设置）
        self.mission_running = False

    # ==================== 目标收集（串口屏模式） ====================

    def acquire_targets_by_screen(self) -> List[str]:
        """
        阻塞式通过串口屏收集目标点。
        串口屏发送 "targetN" (N=1~12)，收集满2个后返回。
        发送 "target0" 清空已收集目标。
        可被 stop_acquiring() 中断。
        """
        self.collected_targets = []
        logger.info("[SCREEN_MGR] 进入串口屏目标收集模式，请发送 targetN (N=1~12)")

        while len(self.collected_targets) < 2 and not self._stop_acquire.is_set():
            result = self.screen.wait_for_data(timeout=1.0)
            if result is None:
                continue

            dtype, value = result
            if dtype != "str" or not isinstance(value, str):
                continue

            value = value.strip()

            # 收集过程中也响应紧急停止
            if value == self.EMERGENCY_STOP_CMD:
                logger.warning("[SCREEN_MGR] 目标收集过程中收到紧急停止")
                self._stop_acquire.set()
                break

            match = self._pat_target.search(value)
            if match:
                num = match.group(1)
                n = int(num)
                if n == 0:
                    self.collected_targets = []
                    logger.info("[SCREEN_MGR] 清空已收集目标点")
                elif 1 <= n <= 12:
                    self.collected_targets.append(num)
                    logger.info(f"[SCREEN_MGR] 收集到目标: target{num} (第{len(self.collected_targets)}/2个)")
                else:
                    logger.debug(f"[SCREEN_MGR] target数字超出范围1-12: {num}")

        return self.collected_targets

    def stop_acquiring(self):
        """停止目标收集过程"""
        self._stop_acquire.set()

    def reset_acquire_state(self):
        """重置收集状态（在开始新一轮收集前调用）"""
        self._stop_acquire.clear()
        self.collected_targets = []
        self.acquiring_targets = False
        self.acquire_mode = None

    # ==================== 倒计时与任务启动 ====================

    def show_target_info_and_countdown(self, t1: str, t2: str, countdown_sec: float = 20.0) -> bool:
        """
        在串口屏上显示目标信息并进行倒计时缓冲。
        倒计时期间可通过串口屏发送 "mission stop" 取消。
        返回 True 表示确认启动，False 表示被取消。
        """
        if t1 == t2:
            logger.error(f"[SCREEN_MGR] 两个目标相同(t1={t1}, t2={t2})，请重新选择")
            return False

        # 串口屏切换到目标信息页面
        self.screen.send_command("page 2")
        self.screen.set_widget_value("text0.txt", target_features[t1]["name"])
        self.screen.set_widget_value("text1.txt", target_features[t2]["name"])
        self.screen.set_widget_value("text0.bgColor", target_features[t1]["color"])
        self.screen.set_widget_value("text1.bgColor", target_features[t2]["color"])

        logger.info(f"[SCREEN_MGR] {countdown_sec:.0f}秒倒计时，发送 '{self.EMERGENCY_STOP_CMD}' 可取消任务...")
        t0 = time.perf_counter()
        last_logged = -1

        while time.perf_counter() - t0 < countdown_sec:
            remaining = countdown_sec - (time.perf_counter() - t0)
            result = self.screen.wait_for_data(timeout=min(remaining, 2.0))
            if result is not None:
                dtype, value = result
                if dtype == "str" and isinstance(value, str):
                    value = value.strip()
                    if value == self.EMERGENCY_STOP_CMD:
                        logger.warning("[SCREEN_MGR] 缓冲期内收到停止指令，取消任务启动")
                        return False

            elapsed = int(time.perf_counter() - t0)
            if elapsed % 5 == 0 and elapsed > 0 and elapsed != last_logged:
                last_logged = elapsed
                logger.info(f"[SCREEN_MGR] 倒计时: {int(countdown_sec) - elapsed}秒后启动任务")

        return True

    def switch_to_main_page(self):
        """切换串口屏回主页面"""
        self.screen.send_command("page 1")

    # ==================== 指令解析与分发 ====================

    def parse_command(self, raw_value: str) -> Optional[Tuple[str, Optional[str]]]:
        """
        解析串口屏指令字符串，返回 (指令类型, 参数) 或 None。
        指令类型：
          - ("emergency_stop", None)
          - ("tarset", "1" 或 "2")
          - ("target", "N")
          - ("unknown", raw_value)
        """
        value = raw_value.strip()

        if value == self.EMERGENCY_STOP_CMD:
            return ("emergency_stop", None)

        match = self._pat_tarset.search(value)
        if match:
            return ("tarset", match.group(1))

        match = self._pat_target.search(value)
        if match:
            return ("target", match.group(1))

        return ("unknown", value)

    # ==================== 主循环 ====================

    def run_command_loop(self):
        """
        主循环：监听串口屏数据并分发指令。
        该方法会阻塞，需在线程中调用。
        """
        logger.info("[SCREEN_MGR] 等待串口屏指令...")
        logger.info(
            f"[SCREEN_MGR] 支持的指令: "
            f"'tarset1'(串口屏选目标) / 'tarset2'(视觉选目标) / '{self.EMERGENCY_STOP_CMD}'"
        )

        while True:
            # 串口屏模式下主线程空等
            if self.acquiring_targets and self.acquire_mode == "screen":
                time.sleep(0.1)
                continue

            result = self.screen.wait_for_data(timeout=5.0)
            if result is None:
                continue

            dtype, value = result
            if dtype != "str" or not isinstance(value, str):
                continue

            parsed = self.parse_command(value)
            if parsed is None:
                continue

            cmd_type, cmd_param = parsed

            if cmd_type == "emergency_stop":
                self._handle_emergency_stop()

            elif cmd_type == "tarset":
                self._handle_tarset(cmd_param)

            elif cmd_type == "target":
                logger.debug(f"[SCREEN_MGR] 收到游离target指令: target{cmd_param}（非收集模式下忽略）")

            else:
                logger.debug(f"[SCREEN_MGR] 忽略未知指令: {cmd_param}")

    def _handle_emergency_stop(self):
        """处理紧急停止"""
        logger.warning("[SCREEN_MGR] 收到紧急停止指令!")

        # 停止目标获取
        if self.acquiring_targets:
            self.stop_acquiring()
            self.acquiring_targets = False
            self.acquire_mode = None
            self.collected_targets.clear()
            logger.info("[SCREEN_MGR] 目标获取过程已停止并清空")

        # 通知外部处理紧急停止（如降落等）
        if self.on_emergency_stop is not None:
            self.on_emergency_stop()

    def _handle_tarset(self, mode: Optional[str]):
        """处理 tarset 指令"""
        if self.mission_running:
            logger.warning("[SCREEN_MGR] 当前有任务正在运行，请先发送 'mission stop'")
            return

        if self.acquiring_targets:
            logger.warning("[SCREEN_MGR] 正在获取目标点中，请先发送 'mission stop' 取消")
            return

        self.reset_acquire_state()

        if mode == "1":
            # 串口屏模式
            self.acquire_mode = "screen"
            self.acquiring_targets = True

            def _screen_acquire_wrapper():
                try:
                    targets = self.acquire_targets_by_screen()
                    if not self._stop_acquire.is_set() and len(targets) >= 2:
                        if self.on_targets_collected is not None:
                            self.on_targets_collected(targets[0], targets[1])
                    elif not self._stop_acquire.is_set():
                        logger.warning(f"[SCREEN_MGR] 目标点不足(需2个，获{len(targets)}个)")
                finally:
                    self.acquiring_targets = False

            t = threading.Thread(target=_screen_acquire_wrapper, daemon=True)
            t.start()

            # 定时检查收集是否完成
            def _check_done():
                if t.is_alive():
                    threading.Timer(0.5, _check_done).start()
                else:
                    if self._stop_acquire.is_set():
                        return
                    if len(self.collected_targets) >= 2:
                        if self.on_targets_collected is not None:
                            self.on_targets_collected(
                                self.collected_targets[0], self.collected_targets[1]
                            )
                    else:
                        logger.warning(
                            f"[SCREEN_MGR] 目标点不足(需2个，获{len(self.collected_targets)}个)"
                        )

            _check_done()

        elif mode == "2":
            # 视觉模式
            self.acquire_mode = "vision"
            self.acquiring_targets = True

            if self.on_vision_acquire_requested is not None:
                self.on_vision_acquire_requested()
            else:
                logger.warning("[SCREEN_MGR] 视觉模式未注册回调")
                self.acquiring_targets = False

        else:
            logger.warning(f"[SCREEN_MGR] 未知目标获取模式: tarset{mode} (仅支持 tarset1 或 tarset2)")

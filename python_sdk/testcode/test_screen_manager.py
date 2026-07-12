"""
串口屏管理器测试程序
使用 Mock 替代真实硬件（FC_Controller、UARTScreen），模拟串口屏交互。
"""

import time
import threading
from unittest.mock import MagicMock, PropertyMock, patch
from loguru import logger

from screen_manager import ScreenManager, target_features


class MockUARTScreen:
    """
    模拟 UARTScreen，通过 inject_data() 注入数据来模拟串口屏发送指令。
    """

    def __init__(self):
        self._queue = []
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._commands_sent = []  # 记录发送过的指令
        self._widget_values = {}  # 记录设置的控件值

    def inject_data(self, dtype: str, value):
        """模拟串口屏发送数据，如 inject_data("str", "tarset1")"""
        with self._lock:
            self._queue.append((dtype, value))
            self._event.set()

    def wait_for_data(self, timeout=None):
        """模拟 UARTScreen.wait_for_data"""
        deadline = None if timeout is None else time.perf_counter() + timeout
        while True:
            with self._lock:
                if self._queue:
                    return self._queue.pop(0)
            remaining = None
            if deadline is not None:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None
            self._event.wait(timeout=remaining if remaining is not None else 0.1)
            self._event.clear()

    def send_command(self, cmd: str):
        """模拟发送指令到串口屏"""
        self._commands_sent.append(cmd)
        logger.debug(f"[MOCK SCREEN] 发送指令: {cmd}")

    def set_widget_value(self, widget: str, *args):
        """模拟设置控件值"""
        self._widget_values[widget] = args
        logger.debug(f"[MOCK SCREEN] 设置控件: {widget} = {args}")

    def get_widget_value(self, widget: str):
        """模拟获取控件值"""
        return self._widget_values.get(widget)

    def send_string(self, string: str):
        self._commands_sent.append(string)


# ==================== 测试用例 ====================

def test_parse_command():
    """测试指令解析"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    assert mgr.parse_command("tarset1") == ("tarset", "1")
    assert mgr.parse_command("tarset2") == ("tarset", "2")
    assert mgr.parse_command("mission stop") == ("emergency_stop", None)
    assert mgr.parse_command("target3") == ("target", "3")
    assert mgr.parse_command("target12") == ("target", "12")
    assert mgr.parse_command("hello") == ("unknown", "hello")
    assert mgr.parse_command("  tarset1  ") == ("tarset", "1")
    logger.info("[TEST] parse_command 测试通过")


def test_acquire_targets_by_screen():
    """测试串口屏目标收集"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    collected = []

    def on_collected(t1, t2):
        collected.append((t1, t2))

    mgr.on_targets_collected = on_collected

    # 在另一个线程中运行收集
    def run_acquire():
        targets = mgr.acquire_targets_by_screen()
        if len(targets) >= 2 and mgr.on_targets_collected:
            mgr.on_targets_collected(targets[0], targets[1])

    t = threading.Thread(target=run_acquire, daemon=True)
    t.start()

    # 模拟串口屏发送目标指令
    time.sleep(0.2)
    screen.inject_data("str", "target3")
    time.sleep(0.2)
    screen.inject_data("str", "target7")

    t.join(timeout=5.0)

    assert len(collected) == 1
    assert collected[0] == ("3", "7")
    logger.info("[TEST] acquire_targets_by_screen 测试通过")


def test_acquire_targets_with_clear():
    """测试目标收集中清空"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    collected = []

    def run_acquire():
        return mgr.acquire_targets_by_screen()

    t = threading.Thread(target=run_acquire, daemon=True)
    t.start()

    time.sleep(0.2)
    screen.inject_data("str", "target5")
    time.sleep(0.2)
    screen.inject_data("str", "target0")  # 清空
    time.sleep(0.2)
    screen.inject_data("str", "target1")
    time.sleep(0.2)
    screen.inject_data("str", "target9")

    t.join(timeout=5.0)

    # 清空后重新收集，最终应该是 1 和 9
    assert len(mgr.collected_targets) == 2
    assert mgr.collected_targets[0] == "1"
    assert mgr.collected_targets[1] == "9"
    logger.info("[TEST] acquire_targets_with_clear 测试通过")


def test_acquire_targets_stop():
    """测试中途停止目标收集"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    result = [None]

    def run_acquire():
        result[0] = mgr.acquire_targets_by_screen()

    t = threading.Thread(target=run_acquire, daemon=True)
    t.start()

    time.sleep(0.2)
    screen.inject_data("str", "target3")
    time.sleep(0.2)
    mgr.stop_acquiring()

    t.join(timeout=3.0)

    # 只收集到1个就被停止
    assert len(result[0]) == 1
    logger.info("[TEST] acquire_targets_stop 测试通过")


def test_countdown_confirm():
    """测试倒计时确认启动"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    # 用3秒倒计时测试
    def run_countdown():
        return mgr.show_target_info_and_countdown("1", "5", countdown_sec=3.0)

    t = threading.Thread(target=lambda results: results.append(run_countdown()), args=([None],), daemon=True)
    results = [None]
    t = threading.Thread(target=lambda: results.__setitem__(0, mgr.show_target_info_and_countdown("1", "5", countdown_sec=3.0)), daemon=True)
    t.start()

    # 不发送取消指令，等待倒计时结束
    t.join(timeout=8.0)

    assert results[0] == True
    # 验证串口屏设置了正确的控件值
    assert screen._widget_values.get("text0.txt") == (target_features["1"]["name"],)
    assert screen._widget_values.get("text1.txt") == (target_features["5"]["name"],)
    assert "page 2" in screen._commands_sent
    logger.info("[TEST] countdown_confirm 测试通过")


def test_countdown_cancel():
    """测试倒计时取消"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    results = [None]

    def run_countdown():
        results[0] = mgr.show_target_info_and_countdown("3", "8", countdown_sec=10.0)

    t = threading.Thread(target=run_countdown, daemon=True)
    t.start()

    # 1秒后发送取消指令
    time.sleep(1.0)
    screen.inject_data("str", "mission stop")

    t.join(timeout=5.0)

    assert results[0] == False
    logger.info("[TEST] countdown_cancel 测试通过")


def test_same_target_rejected():
    """测试相同目标被拒绝"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    result = mgr.show_target_info_and_countdown("3", "3", countdown_sec=3.0)
    assert result == False
    logger.info("[TEST] same_target_rejected 测试通过")


def test_emergency_stop_callback():
    """测试紧急停止回调"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    stopped = [False]

    def on_stop():
        stopped[0] = True

    mgr.on_emergency_stop = on_stop
    mgr.acquiring_targets = True
    mgr.acquire_mode = "screen"

    # 直接调用内部处理
    mgr._handle_emergency_stop()

    assert stopped[0] == True
    assert mgr.acquiring_targets == False
    assert mgr.acquire_mode is None
    logger.info("[TEST] emergency_stop_callback 测试通过")


def test_tarset_while_mission_running():
    """测试任务运行时拒绝 tarset"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)
    mgr.mission_running = True

    # 不应触发任何回调
    callback_called = [False]

    def on_collected(t1, t2):
        callback_called[0] = True

    mgr.on_targets_collected = on_collected
    mgr._handle_tarset("1")

    assert callback_called[0] == False
    logger.info("[TEST] tarset_while_mission_running 测试通过")


def test_vision_mode_callback():
    """测试视觉模式回调触发"""
    screen = MockUARTScreen()
    mgr = ScreenManager(screen)

    vision_requested = [False]

    def on_vision():
        vision_requested[0] = True
        mgr.acquiring_targets = False  # 模拟外部处理完成

    mgr.on_vision_acquire_requested = on_vision
    mgr._handle_tarset("2")

    assert vision_requested[0] == True
    assert mgr.acquire_mode == "vision"
    logger.info("[TEST] vision_mode_callback 测试通过")


# ==================== 运行所有测试 ====================

if __name__ == "__main__":
    logger.info("=" * 50)
    logger.info("串口屏管理器测试开始")
    logger.info("=" * 50)

    tests = [
        test_parse_command,
        test_acquire_targets_by_screen,
        test_acquire_targets_with_clear,
        test_acquire_targets_stop,
        test_countdown_confirm,
        test_countdown_cancel,
        test_same_target_rejected,
        test_emergency_stop_callback,
        test_tarset_while_mission_running,
        test_vision_mode_callback,
    ]

    passed = 0
    failed = 0

    for test_fn in tests:
        try:
            test_fn()
            passed += 1
        except Exception as e:
            logger.exception(f"[TEST] {test_fn.__name__} 失败: {e}")
            failed += 1

    logger.info("=" * 50)
    logger.info(f"测试完成: {passed} 通过, {failed} 失败, 共 {len(tests)} 项")
    logger.info("=" * 50)

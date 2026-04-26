"""
串口屏通信诊断测试程序
在数据流链路的多个关键点注入诊断日志，定位串口屏数据丢失位置。
不执行任何飞机运动指令。

诊断层次：
  [RAW]      飞控串口原始帧数据 (cmd byte)
  [SCREEN]   UARTScreen._callback 收到的原始字节
  [PARSE]    _callback 内部解析结果
  [QUEUE]    入队数据
  [REPORT]   串口屏主动上报
"""

import os, sys, time, threading

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from loguru import logger
from queue import Empty, Queue

# 设置 loguru 显示 DEBUG 级别
logger.remove()
logger.add(sys.stderr, level="DEBUG")

from FlightController import FC_Controller
from FlightController.Components.UartScreen import UARTScreen
from screen_manager import ScreenManager


# ==================== 诊断补丁 ====================

def patch_fc_for_diag(fc):
    """给飞控 _update_fc_data 加诊断日志，拦截所有 cmd 分支"""
    original_update = fc._update_fc_data

    def diag_update(data: bytes):
        cmd = data[0]
        payload = data[1:]
        # 只记录串口屏相关帧和非状态帧
        if cmd != 0x01:
            logger.debug(f"[RAW] FC帧 cmd=0x{cmd:02X}, payload={payload.hex()} ({len(payload)}B)")
            if cmd == 0x05:
                logger.info(f"[RAW] >>> 串口屏帧! payload={payload.hex()}")
        original_update(data)

    fc._update_fc_data = diag_update
    logger.info("[DIAG] FC _update_fc_data 已补丁")


def patch_screen_for_diag(screen):
    """给 UARTScreen._callback 加诊断日志，记录收到的原始字节和解析路径"""
    original_callback = screen._callback

    def diag_callback(data: bytes):
        hex_str = data.hex()
        logger.info(f"[SCREEN] _callback 收到原始数据: {hex_str} ({len(data)}B)")

        # 尝试解析第一个字节
        if len(data) == 0:
            logger.warning("[PARSE] 空数据!")
            return

        first_byte = data[0]

        if first_byte == ord("\\"):
            logger.info(f"[PARSE] 检测为报告类型 (0x5C)")
        elif first_byte == 0x00:
            logger.info(f"[PARSE] 检测为指令结果 (0x00), result=0x{data[1]:02X}" if len(data) > 1 else f"[PARSE] 检测为指令结果 (0x00), 数据过短")
        elif first_byte == 0x01:
            logger.info(f"[PARSE] 检测为整数类型 (0x01)")
        elif first_byte == 0x02:
            logger.info(f"[PARSE] 检测为浮点类型 (0x02)")
        elif first_byte == 0x03:
            logger.info(f"[PARSE] 检测为字符串类型 (0x03)")
            # 尝试解码
            try:
                raw_str = data[1:-2].decode("utf-8")
                logger.info(f"[PARSE] 字符串内容: {raw_str!r}")
            except Exception as e:
                logger.warning(f"[PARSE] 字符串解码失败: {e}, raw={data[1:-2].hex()}")
        else:
            logger.warning(f"[PARSE] 未知类型 0x{first_byte:02X}! 完整数据: {hex_str}")
            # 尝试作为原始UTF-8解码看看
            try:
                raw_text = data.decode("utf-8")
                logger.info(f"[PARSE] 作为原始文本解码: {raw_text!r}")
            except Exception:
                pass

        # 调用原始回调
        try:
            original_callback(data)
        except Exception as e:
            logger.exception(f"[PARSE] 原始回调异常: {e}")

    screen._callback = diag_callback
    # 重新注册到飞控
    screen._fc.register_uart_screen_callback(diag_callback)
    logger.info("[DIAG] UARTScreen._callback 已补丁")


def patch_queue_for_diag(screen):
    """给数据队列加诊断"""
    original_queue = screen._data_queue
    diag_queue = Queue()  # noqa: F841 - 保留旧队列引用以防GC

    class DiagQueue(Queue):
        def put(self, item, block=True, timeout=None):
            logger.info(f"[QUEUE] 入队: type={item[0]}, value={item[1]!r}")
            super().put(item, block, timeout)

    # 替换队列
    screen._data_queue = DiagQueue()
    # 将旧队列中的数据转移
    while not original_queue.empty():
        try:
            screen._data_queue.put(original_queue.get_nowait())
        except Empty:
            break
    logger.info("[DIAG] 数据队列已补丁")


# ==================== 主动探测函数 ====================

def probe_screen(screen):
    """主动向串口屏发送探测指令，检查双向通信"""
    logger.info("=" * 50)
    logger.info("[PROBE] 开始主动探测串口屏通信")
    logger.info("=" * 50)

    # 1. 发送页面切换指令
    logger.info("[PROBE] 发送: page 0")
    screen.send_command("page 0")
    time.sleep(1.0)

    # 2. 尝试读取系统参数
    logger.info("[PROBE] 发送: sget system.version")
    result = screen.get_system_value("system.version")
    logger.info(f"[PROBE] system.version 返回: {result}")
    time.sleep(0.5)

    # 3. 尝试发送自定义字符串看有无回应
    logger.info("[PROBE] 发送: sget dp")
    result = screen.get_system_value("dp")
    logger.info(f"[PROBE] dp 返回: {result}")
    time.sleep(0.5)

    # 4. 发送页面切换到page 1
    logger.info("[PROBE] 发送: page 1")
    screen.send_command("page 1")
    time.sleep(1.0)

    # 5. 检查队列中是否有积压数据
    pending = []
    while True:
        try:
            item = screen._data_queue.get_nowait()
            pending.append(item)
        except Empty:
            break
    if pending:
        for item in pending:
            logger.info(f"[PROBE] 队列积压数据: type={item[0]}, value={item[1]!r}")
    else:
        logger.warning("[PROBE] 主动探测后队列仍为空 — 串口屏可能未回应任何数据")

    logger.info("=" * 50)
    logger.info("[PROBE] 探测完成，等待串口屏主动发送数据...")
    logger.info("=" * 50)


# ==================== 主程序 ====================

def main():
    # ====== 连接飞控 ======
    logger.info("[TEST] 正在连接飞控...")
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyACM0", print_state=False)
    fc.wait_for_connection()
    logger.info("[TEST] 飞控已连接")

    # ====== 初始化串口屏 ======
    screen = UARTScreen(fc)
    logger.info("[TEST] 串口屏已初始化")

    # ====== 注入诊断补丁 ======
    patch_fc_for_diag(fc)
    patch_screen_for_diag(screen)
    patch_queue_for_diag(screen)

    # ====== 注册报告回调 ======
    def on_report(data: str):
        logger.info(f"[REPORT] 串口屏主动上报: {data!r}")

    screen.register_report_callback(on_report)

    # ====== 主动探测 ======
    probe_screen(screen)

    # ====== 创建 ScreenManager ======
    mgr = ScreenManager(screen)

    mission_running = False

    def on_targets_collected(t1: str, t2: str):
        nonlocal mission_running
        logger.info(f"[TEST] >>> 目标收集完成: t1={t1}, t2={t2}")
        confirmed = mgr.show_target_info_and_countdown(t1, t2, countdown_sec=10.0)
        if confirmed:
            mission_running = True
            mgr.mission_running = True
            mgr.switch_to_main_page()
            logger.info(f"[TEST] >>> 模拟任务启动: target1={t1}, target2={t2}")
            def _auto_end():
                nonlocal mission_running
                time.sleep(15.0)
                if mission_running:
                    mission_running = False
                    mgr.mission_running = False
                    logger.info("[TEST] >>> 模拟任务自动结束")
            threading.Thread(target=_auto_end, daemon=True).start()
        else:
            logger.info("[TEST] >>> 任务启动被取消")

    def on_vision_acquire():
        logger.warning("[TEST] >>> 视觉模式被请求，摄像头未安装")
        time.sleep(3.0)
        mgr.acquiring_targets = False
        mgr.acquire_mode = None
        logger.info("[TEST] >>> 视觉模式已结束")

    def on_emergency_stop():
        nonlocal mission_running
        logger.warning("[TEST] >>> 紧急停止!")
        if mission_running:
            mission_running = False
            mgr.mission_running = False
            logger.info("[TEST] >>> 模拟任务已紧急停止")

    mgr.on_targets_collected = on_targets_collected
    mgr.on_vision_acquire_requested = on_vision_acquire
    mgr.on_emergency_stop = on_emergency_stop

    # ====== 进入主循环 ======
    logger.info("=" * 60)
    logger.info("串口屏通信诊断测试")
    logger.info("串口屏指令: tarset1 / tarset2 / targetN / mission stop")
    logger.info("现在请操作串口屏发送指令... (Ctrl+C 退出)")
    logger.info("=" * 60)

    try:
        mgr.run_command_loop()
    except KeyboardInterrupt:
        logger.info("[TEST] 用户中断")
    finally:
        logger.info("[TEST] 测试结束")


if __name__ == "__main__":
    main()

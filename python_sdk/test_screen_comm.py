"""
串口屏通信测试程序
连接真实飞控+串口屏，测试双向通信与指令控制功能。
不执行任何飞机运动指令，仅验证串口屏控制链路。
"""

import os, sys, time, threading, re

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from loguru import logger
from FlightController import FC_Controller
from FlightController.Components.UartScreen import UARTScreen
from screen_manager import ScreenManager


def main():
    # ====== 连接飞控 ======
    logger.info("[TEST] 正在连接飞控...")
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyAMA0", print_state=False)
    fc.wait_for_connection()
    logger.info("[TEST] 飞控已连接")

    # ====== 初始化串口屏 ======
    screen = UARTScreen(fc)
    logger.info("[TEST] 串口屏已初始化")

    # ====== 创建 ScreenManager ======
    mgr = ScreenManager(screen)

    # 状态
    mission_running = False

    # ------ 回调：目标收集完成 ------
    def on_targets_collected(t1: str, t2: str):
        nonlocal mission_running
        logger.info(f"[TEST] >>> 目标收集完成: t1={t1}, t2={t2}")

        # 倒计时（实际不启动任务）
        confirmed = mgr.show_target_info_and_countdown(t1, t2, countdown_sec=10.0)
        if confirmed:
            mission_running = True
            mgr.mission_running = True
            mgr.switch_to_main_page()
            logger.info(f"[TEST] >>> 模拟任务启动（不执行飞行动作）: target1={t1}, target2={t2}")
            logger.info("[TEST] >>> 发送 'mission stop' 可停止任务")

            # 模拟任务运行 15 秒后自动结束
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

    # ------ 回调：视觉模式请求 ------
    def on_vision_acquire():
        logger.warning("[TEST] >>> 视觉模式被请求，但摄像头未安装，无法执行")
        logger.info("[TEST] >>> 3秒后自动结束视觉模式...")
        time.sleep(3.0)
        mgr.acquiring_targets = False
        mgr.acquire_mode = None
        logger.info("[TEST] >>> 视觉模式已结束（未检测到目标）")

    # ------ 回调：紧急停止 ------
    def on_emergency_stop():
        nonlocal mission_running
        logger.warning("[TEST] >>> 紧急停止！")
        if mission_running:
            mission_running = False
            mgr.mission_running = False
            logger.info("[TEST] >>> 模拟任务已紧急停止（不执行降落）")
        else:
            logger.info("[TEST] >>> 无运行中的任务")

    mgr.on_targets_collected = on_targets_collected
    mgr.on_vision_acquire_requested = on_vision_acquire
    mgr.on_emergency_stop = on_emergency_stop

    # ====== 额外：原始数据监控线程 ======
    # 与 ScreenManager 并行运行，打印所有从串口屏收到的原始数据
    stop_monitor = threading.Event()

    def raw_data_monitor():
        """独立监控串口屏数据（仅显示，不消费）"""
        logger.info("[MONITOR] 原始数据监控已启动")
        while not stop_monitor.is_set():
            # 使用短超时轮询，避免与 ScreenManager 的 wait_for_data 冲突
            # 这里直接注册 report callback 来获取上报数据
            time.sleep(0.1)
        logger.info("[MONITOR] 原始数据监控已停止")

    # 注册串口屏主动上报回调，捕获所有屏幕主动推送的数据
    reported_data = []

    def on_report(data: str):
        reported_data.append(data)
        logger.info(f"[MONITOR] 串口屏上报: {data!r}")

    screen.register_report_callback(on_report)

    # ====== 启动 ======
    logger.info("=" * 60)
    logger.info("串口屏通信测试程序")
    logger.info("=" * 60)
    logger.info("支持的串口屏指令:")
    logger.info("  tarset1      - 串口屏选目标模式（发送 targetN 收集目标）")
    logger.info("  tarset2      - 视觉选目标模式（摄像头未安装，会提示）")
    logger.info("  mission stop - 紧急停止")
    logger.info("  targetN      - 目标编号 (N=1~12, N=0清空)")
    logger.info("=" * 60)
    logger.info("[TEST] 等待串口屏指令... (Ctrl+C 退出)")
    logger.info("")

    try:
        mgr.run_command_loop()
    except KeyboardInterrupt:
        logger.info("[TEST] 用户中断")
    finally:
        stop_monitor.set()
        logger.info(f"[TEST] 串口屏上报数据汇总: {reported_data}")
        logger.info("[TEST] 测试结束")


if __name__ == "__main__":
    main()

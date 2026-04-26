import os, sys, time, threading

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from loguru import logger
logger.remove()
logger.add(sys.stderr, level="INFO")
from FlightController import FC_Controller


def main():
    fc = FC_Controller()
    fc.start_listen_serial(serial_dev="/dev/ttyACM0", print_state=False)
    fc.wait_for_connection()
    logger.info("[TEST] 飞控已连接")

    DEFAULT_DURATION = 5.0  # 秒

    logger.info("============================================================")
    logger.info("蜂鸣器测试程序")
    logger.info("============================================================")
    logger.info("支持的指令:")
    logger.info("  buzz [时长s]  - 蜂鸣器发声 (默认5秒)")
    logger.info("  quit          - 退出程序")
    logger.info("============================================================")

    def pwm_output_task(channel: int, value: int, stop_event: threading.Event, interval: float = 0.05):
        """持续发送PWM输出，直到stop_event被设置"""
        while not stop_event.is_set():
            fc.set_PWM_output(channel, value)
            stop_event.wait(interval)

    try:
        while True:
            try:
                cmd = input(">>> ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            if cmd == "quit":
                logger.info("[TEST] 退出程序")
                break

            if cmd.startswith("buzz"):
                parts = cmd.split()
                duration = float(parts[1]) if len(parts) >= 2 else DEFAULT_DURATION
                logger.info(f"[TEST] 蜂鸣器发声 {duration}s")

                stop_event = threading.Event()
                t = threading.Thread(target=pwm_output_task, args=(3, 100, stop_event), daemon=True)
                t.start()
                time.sleep(duration)
                stop_event.set()
                t.join(timeout=2.0)
                logger.info("[TEST] 蜂鸣器关闭")
                continue

            logger.debug(f"[TEST] 忽略未知指令: {cmd}")
    finally:
        logger.info("[TEST] 程序结束")


if __name__ == "__main__":
    main()

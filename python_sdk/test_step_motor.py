import os, sys, time

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

    # 默认参数：0.5圈，与 Mission 中 revolutions_60cm 一致
    DEFAULT_REVOLUTIONS = 0.5
    DEFAULT_STEP_DELAY = 0.5  # 秒，对应 Mission 中的 0.5

    logger.info("============================================================")
    logger.info("步进电机测试程序")
    logger.info("============================================================")
    logger.info("支持的指令:")
    logger.info("  run [圈数] [步间延时ms] [fast] - 转动电机 (默认0.5圈, 0.5ms)")
    logger.info("    fast: 快速模式, 全步驱动+无ACK, 速度提升约6-8倍")
    logger.info("  quit                           - 退出程序")
    logger.info("============================================================")

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

            if cmd.startswith("run"):
                parts = cmd.split()
                revolutions = float(parts[1]) if len(parts) >= 2 else DEFAULT_REVOLUTIONS
                step_delay = float(parts[2]) if len(parts) >= 3 else DEFAULT_STEP_DELAY
                fast = "fast" in parts
                mode_str = "FAST" if fast else "NORMAL"
                logger.info(f"[TEST] 步进电机转动 [{mode_str}]: {revolutions} 圈, 步间延时 {step_delay}ms")
                t0 = time.perf_counter()
                fc.step_motor_rotate(revolutions, step_delay, fast=fast)
                elapsed = time.perf_counter() - t0
                logger.info(f"[TEST] 转动完成, 耗时 {elapsed:.2f}s")
                continue

            logger.debug(f"[TEST] 忽略未知指令: {cmd}")
    finally:
        logger.info("[TEST] 程序结束")


if __name__ == "__main__":
    main()

"""
实时读取飞控模式（程控/定点/定高）的程序。

仅连接飞控串口并读取遥测数据中的 mode 字段，不做任何控制动作：
  - 不解锁
  - 不发送实时控制帧
  - 不驱动电机 / PWM / 蜂鸣器
  - 不设置飞行模式

用法:
    python read_fc_mode.py [--port COM3] [--baud 500000]

默认自动搜索 VID:PID=66CC:2233 的飞控串口。
按 Ctrl+C 退出。
"""

import argparse
import sys
import time

from FlightController import FC_Controller

# 飞控模式号 → 名称映射（与 Protocal.py 常量一致）
MODE_MAP = {
    0: "姿态自稳 (STABILIZE) —— 危险，仅调试",
    1: "定高 (HOLD_ALT)",
    2: "定点 (HOLD_POS)",
    3: "程控 (PROGRAM)",
}


def mode_name(mode_value: int) -> str:
    """返回模式的可读名称，未知值给出警告。"""
    if mode_value in MODE_MAP:
        return MODE_MAP[mode_value]
    return f"未知模式 ({mode_value})"


def main():
    parser = argparse.ArgumentParser(
        description="实时读取飞控飞行模式（仅连接，不控制）"
    )
    parser.add_argument(
        "--port", "-p",
        default=None,
        help="飞控串口设备名（默认自动搜索 VID:PID=66CC:2233）",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=500000,
        help="串口波特率（默认 500000）",
    )
    args = parser.parse_args()

    # ---- 创建飞控对象并连接串口 ----
    fc = FC_Controller()
    print(f"[*] 正在连接飞控串口 ...")
    try:
        fc.start_listen_serial(
            serial_dev=args.port,
            baudrate=args.baud,
            print_state=False,   # 用自己的格式化输出
            block_until_connected=False,
        )
    except AssertionError:
        print("[!] 未找到飞控串口 (VID:PID=66CC:2233)，请检查连接或通过 --port 指定。")
        sys.exit(1)

    # ---- 等待首帧遥测 ----
    print("[*] 等待飞控遥测数据 ...")
    timeout_s = 5.0
    t0 = time.perf_counter()
    while not fc.connected:
        time.sleep(0.1)
        if time.perf_counter() - t0 > timeout_s:
            print(f"[!] {timeout_s}s 内未收到遥测数据，请检查飞控是否上电并正常发送。")
            fc.close()
            sys.exit(1)

    print("[✓] 飞控已连接，开始实时显示模式。按 Ctrl+C 退出。\n")

    last_mode = None
    last_print_time = 0.0
    print_interval = 1.0  # 每秒刷新一行

    try:
        while fc.running:
            current_mode = fc.state.mode.value
            unlock = fc.state.unlock.value
            alt = fc.state.alt_add.value   # cm
            bat = fc.state.bat.value       # V

            # 模式变化时打印醒目的变化行
            if current_mode != last_mode:
                print(
                    f"\n[模式变化] {mode_name(current_mode)}"
                    f" | 解锁={'是' if unlock else '否'}"
                    f" | 高度={alt} cm"
                    f" | 电池={bat:.2f} V"
                )
                last_mode = current_mode

            # 每秒刷新一行当前状态
            now = time.monotonic()
            if now - last_print_time >= print_interval:
                print(
                    f"\r[实时] {mode_name(current_mode):40s}"
                    f" | 解锁={'是' if unlock else '否'}"
                    f" | 高度={alt:5d} cm"
                    f" | 电池={bat:.2f} V",
                    end="",
                    flush=True,
                )
                last_print_time = now

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\n[!] 用户中断。")
    finally:
        fc.close()
        print("[✓] 飞控连接已关闭。")


if __name__ == "__main__":
    main()

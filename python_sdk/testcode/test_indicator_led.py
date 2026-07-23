"""
测试飞控 32 板载 LED（set_indicator_led）的程序。

仅连接飞控串口并发送 LED 颜色命令，不做任何控制动作：
  - 不解锁
  - 不发送实时控制帧
  - 不驱动电机 / PWM / 蜂鸣器
  - 不设置飞行模式

用法:
    # 设置指定颜色（R G B 各 0-255）
    python testcode/test_indicator_led.py --fc-port COM3 --color 255 0 0    # 红色
    python testcode/test_indicator_led.py --fc-port COM3 --color 0 255 0    # 绿色
    python testcode/test_indicator_led.py --fc-port COM3 --color 0 0 255    # 蓝色
    python testcode/test_indicator_led.py --fc-port COM3 --color 255 255 0  # 黄色

    # 循环展示预设颜色（默认行为，需手动 Ctrl+C 退出）
    python testcode/test_indicator_led.py --fc-port COM3 --cycle

    # 循环展示，自定义切换间隔（秒）
    python testcode/test_indicator_led.py --fc-port COM3 --cycle --interval 2.0

协议说明:
    set_indicator_led 将 0-255 的 RGB 值按比例映射到 0-20，通过命令 0x0A
    以 s8 格式发送到飞控。
"""

import argparse
import os
import sys
import time

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from FlightController import FC_Controller  # noqa: E402

# 预设颜色列表（R, G, B）
PRESET_COLORS = [
    (255, 0, 0, "红色 (Red)"),
    (0, 255, 0, "绿色 (Green)"),
    (0, 0, 255, "蓝色 (Blue)"),
    (255, 255, 0, "黄色 (Yellow)"),
    (0, 255, 255, "青色 (Cyan)"),
    (255, 0, 255, "品红 (Magenta)"),
    (255, 255, 255, "白色 (White)"),
    (255, 128, 0, "橙色 (Orange)"),
    (128, 0, 255, "紫色 (Purple)"),
    (0, 0, 0, "熄灭 (Off)"),
]


def rgb_to_fc_value(val: int) -> int:
    """将 0-255 的 RGB 分量映射到飞控的 0-20 范围（与 Protocal.py 一致）。"""
    return round(val / 255 * 20)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="测试飞控 32 板载 LED（仅连接，不控制飞行）"
    )
    parser.add_argument(
        "--fc-port",
        default=None,
        help="飞控串口设备名（默认自动搜索 VID:PID=66CC:2233）",
    )
    parser.add_argument(
        "--fc-baud",
        type=int,
        default=500000,
        help="串口波特率（默认 500000）",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="等待飞控遥测的超时秒数（默认 5）",
    )
    parser.add_argument(
        "--color",
        nargs=3,
        type=int,
        metavar=("R", "G", "B"),
        help="设置单个颜色（R G B 各 0-255），例如 --color 255 0 0",
    )
    parser.add_argument(
        "--cycle",
        action="store_true",
        help="循环展示预设颜色",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.5,
        help="循环模式下每种颜色的保持时间（秒，默认 1.5）",
    )
    return parser.parse_args()


def send_led_color(fc: FC_Controller, r: int, g: int, b: int) -> None:
    """发送 LED 颜色命令并打印映射详情。"""
    r_fc = rgb_to_fc_value(r)
    g_fc = rgb_to_fc_value(g)
    b_fc = rgb_to_fc_value(b)
    print(
        f"  -> set_indicator_led(R={r:3d}, G={g:3d}, B={b:3d})"
        f"  |  飞控值: ({r_fc:2d}, {g_fc:2d}, {b_fc:2d})  [0-20]"
    )
    fc.set_indicator_led(r, g, b)


def main() -> int:
    args = parse_args()

    # 参数校验
    if args.color:
        for val in args.color:
            if not (0 <= val <= 255):
                print(f"[!] RGB 值必须在 0-255 范围内，收到: {val}")
                return 2

    if args.interval <= 0:
        print("[!] --interval 必须大于 0")
        return 2

    # 默认行为：如果没有指定 --color 也没有 --cycle，则循环展示
    cycle_mode = args.cycle or (args.color is None)

    # ---- 创建飞控对象并连接串口 ----
    fc = FC_Controller()
    print(f"[*] 正在连接飞控串口 ...")
    try:
        fc.start_listen_serial(
            serial_dev=args.fc_port,
            baudrate=args.fc_baud,
            print_state=False,
        )
    except AssertionError:
        print("[!] 未找到飞控串口 (VID:PID=66CC:2233)，请检查连接或通过 --fc-port 指定。")
        return 1

    # ---- 等待首帧遥测 ----
    print(f"[*] 等待飞控遥测数据（超时 {args.connect_timeout}s）...")
    if not fc.wait_for_connection(timeout_s=args.connect_timeout):
        print(f"[!] {args.connect_timeout}s 内未收到遥测数据，请检查飞控是否上电并正常发送。")
        fc.close()
        return 1

    print("[✓] 飞控已连接。\n")

    # ---- 确认飞控未解锁 ----
    if fc.state.unlock.value:
        print("[!] 飞控已解锁/ARMED，为安全起见不发送 LED 命令。请先锁桨后再试。")
        fc.close()
        return 3

    try:
        if not cycle_mode:
            # 单次设置颜色
            r, g, b = args.color  # type: ignore[arg-type]
            print(f"[*] 设置 LED 颜色: RGB({r}, {g}, {b})")
            send_led_color(fc, r, g, b)
            print("[✓] 命令已发送。")
        else:
            # 循环展示预设颜色
            if args.color:
                print("[*] --color 与循环模式同时指定，将忽略 --color，使用预设颜色列表。")
            print(f"[*] 开始循环展示预设颜色（共 {len(PRESET_COLORS)} 种，间隔 {args.interval}s）")
            print("[*] 按 Ctrl+C 退出。\n")

            idx = 0
            while fc.running:
                r, g, b, name = PRESET_COLORS[idx % len(PRESET_COLORS)]
                print(f"[{idx + 1}] {name:20s}", end="")
                send_led_color(fc, r, g, b)
                idx += 1
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[!] 用户中断。")
    except Exception as exc:
        print(f"[!] LED 测试失败: {exc}")
        return 1
    finally:
        # 退出前关闭 LED（发送黑色）
        try:
            print("[*] 退出前关闭 LED ...")
            fc.set_indicator_led(0, 0, 0)
        except Exception:
            pass
        fc.close()
        print("[✓] 飞控连接已关闭。")

    return 0


if __name__ == "__main__":
    sys.exit(main())

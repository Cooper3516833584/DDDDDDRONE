"""PWM3 低亮度闪烁测试。

用于验证接在飞控 PWM3（通道 3）上的外接灯/负载。亮度为 8/255，
换算为飞控 PWM 接口使用的百分比约为 3.14%。

本脚本不发送解锁、飞行模式、起飞、降落或运动控制指令。
HW-159 为 WS2812 灯环；PWM3 只能使整个模块统一闪烁，不能逐颗流水。
"""

import argparse
import os
import sys
import time

SDK_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)

PWM_CHANNEL = 3
BRIGHTNESS = 8
PWM_PERCENT = BRIGHTNESS / 255.0 * 100.0
DEFAULT_DURATION_S = 10.0
BLINK_INTERVAL_S = 0.5


def parse_args():
    parser = argparse.ArgumentParser(description="PWM3 低亮度闪烁测试")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="总运行时间（秒），默认 10 秒",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=BLINK_INTERVAL_S,
        help="亮/灭切换间隔（秒），默认 0.5 秒",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0:
        parser.error("--duration 和 --interval 必须大于 0")
    return args


def main():
    args = parse_args()
    # 延迟导入：--help 和参数校验不应触发飞控日志初始化或访问串口。
    from FlightController import FC_Controller

    fc = FC_Controller()
    is_on = False

    try:
        fc.start_listen_serial(serial_dev="/dev/ttyACM0", print_state=False)
        if not fc.wait_for_connection(timeout_s=5.0):
            raise RuntimeError("未在 5 秒内连接到飞控")

        print(
            "PWM3 闪烁：亮度 {}/255（{:.2f}%），持续 {:.1f} 秒；按 Ctrl+C 可提前停止。".format(
                BRIGHTNESS, PWM_PERCENT, args.duration
            )
        )
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            is_on = not is_on
            fc.set_PWM_output(PWM_CHANNEL, PWM_PERCENT if is_on else 0.0)
            remaining = deadline - time.monotonic()
            time.sleep(min(args.interval, max(0.0, remaining)))
    except KeyboardInterrupt:
        print("收到停止请求。")
    finally:
        # 无论正常结束、连接失败或 Ctrl+C，均尝试关闭 PWM3。
        try:
            fc.set_PWM_output(PWM_CHANNEL, 0.0)
        except Exception as exc:
            print("关闭 PWM3 失败：{}".format(exc), file=sys.stderr)


if __name__ == "__main__":
    main()

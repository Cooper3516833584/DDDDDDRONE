"""LCUS 8 路 USB 继电器手动联调工具 (会真实操作继电器硬件)。

只在显式给出动作参数时操作硬件; 不导入飞控, 不发送任何飞控命令。

示例 (Ubuntu 上位机)::

    python3 testcode/relay_lcus_terminal.py --list
    python3 testcode/relay_lcus_terminal.py --print-frames
    python3 testcode/relay_lcus_terminal.py --port /dev/ttyUSB2 --status
    python3 testcode/relay_lcus_terminal.py --port /dev/ttyUSB2 --channel 1 --on
    python3 testcode/relay_lcus_terminal.py --port /dev/ttyUSB2 --channel 1 --off
    python3 testcode/relay_lcus_terminal.py --port /dev/ttyUSB2 --all-off
    python3 testcode/relay_lcus_terminal.py --port /dev/ttyUSB2 --blink 1 --interval 1.0 --repeat 5

示例 (PC 端 Windows, 串口名为 COMx)::

    python testcode/relay_lcus_terminal.py --list
    python testcode/relay_lcus_terminal.py --port COM5 --status
    python testcode/relay_lcus_terminal.py --port COM5 --channel 1 --on
    set D_TASK_RELAY_PORT=COM5 && python testcode/relay_lcus_terminal.py --status

纯逻辑单测在 PC 端和上位机都可以直接运行, 不需要任何硬件::

    python -m unittest testcode.test_relay_lcus -v     # 或在 python_sdk 下: python testcode/test_relay_lcus.py

``--port`` 省略时使用环境变量 ``D_TASK_RELAY_PORT``; 端口尚未确定时必须显式给出,
不要猜测易变的 ``/dev/ttyUSB*`` 编号。

风险与边界 (使用前必读):

- 工具会**真实吸合/断开**继电器触点。只有同时给出端口和动作参数才会动硬件;
  不给动作参数时只打印帮助和可用串口并以退出码 2 结束, 不会自动做任何事。
- 程序退出**不会**断开已吸合的通道(继电器自锁保持)。联调结束前请执行
  ``--all-off`` 并用 ``--status`` 复核; 断电重插会让板子复位为全 OFF, 但不要依赖它当安全手段。
- ``--all-on`` 会同时吸合全部通道, ``--blink`` 会反复吸合同一路: 使用前必须确认负载、
  线径和电源余量, 并注意继电器机械寿命。
- ``--channel N --on/--off`` 带 FF 回读确认。实机 (4 路板) 观察: 控制帧后约 50ms 内回读仍是旧状态,
  所以经常先出现一次"校验失败, 重发第 2 次"再显示 ``[OK]``; 这是预期行为, 不是接线故障。
- 端口必须显式指定: CH340 的 USB ID 与 HC-14 电台相同 (``1a86:7523``), 不能按 VID/PID 猜;
  上电后 ``COMx``/``ttyUSBx`` 编号可能变化, 每次都用 ``--list`` 先确认。
- 使用 4 路板时请加 ``--channels 4``, 否则 8 路默认配置会告警"缺少通道 [5, 6, 7, 8]"。
- 本工具只走独立 USB 串口, 不导入飞控、不打开飞控串口、不发送任何飞控命令,
  也没有任何硬件互锁; 不要把它当作飞行安全回路的一部分。
"""

import argparse
import os
import sys
import time

SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from loguru import logger  # noqa: E402

from FlightController.Components.relay_lcus import (  # noqa: E402
    DEFAULT_BAUDRATE,
    DEFAULT_CHANNEL_COUNT,
    RELAY_PORT_ENV,
    LCUSRelay,
    build_channel_command,
    format_port_list,
    format_states,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="LCUS 8 路 USB 继电器联调工具 (会真实操作继电器)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--list", action="store_true", help="列出当前串口后退出")
    parser.add_argument(
        "--print-frames",
        action="store_true",
        help="打印 1~N 路的开/关指令帧(不打开串口、不操作硬件)",
    )
    parser.add_argument(
        "--port",
        default=None,
        help=f"继电器串口, 例如 /dev/ttyUSB2; 省略时取环境变量 {RELAY_PORT_ENV}",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="波特率, 默认 9600")
    parser.add_argument("--channels", type=int, default=DEFAULT_CHANNEL_COUNT, help="继电器路数, 默认 8")
    parser.add_argument("--query-timeout", type=float, default=1.0, help="一次 FF 查询的总超时秒数, 默认 1.0")
    parser.add_argument("--status", action="store_true", help="发送 FF 查询全部通道状态")
    parser.add_argument(
        "--detect",
        action="store_true",
        help="发送 FF 识别板子实际路数(只读, 例如 4 路板/8 路板), 不改变任何一路状态",
    )
    parser.add_argument("--channel", type=int, help="目标通道号 (1 ~ 路数)")
    parser.add_argument("--on", action="store_true", help="与 --channel 一起使用: 打开该路")
    parser.add_argument("--off", action="store_true", help="与 --channel 一起使用: 关闭该路")
    parser.add_argument("--all-off", action="store_true", help="依次关闭全部通道")
    parser.add_argument("--all-on", action="store_true", help="依次打开全部通道")
    parser.add_argument("--blink", type=int, help="循环开/关指定通道, 用于联调 (需配合 --interval)")
    parser.add_argument("--interval", type=float, default=1.0, help="--blink 的间隔秒数, 默认 1.0")
    parser.add_argument("--repeat", type=int, default=3, help="--blink 的循环次数, 默认 3")
    parser.add_argument("--debug", action="store_true", help="输出 DEBUG 级日志")
    return parser.parse_args(argv)


def print_frames(channel_count):
    print("路号  打开指令      关闭指令")
    for channel in range(1, channel_count + 1):
        on_frame = " ".join(f"{byte:02X}" for byte in build_channel_command(channel, True, channel_count))
        off_frame = " ".join(f"{byte:02X}" for byte in build_channel_command(channel, False, channel_count))
        print(f"CH{channel:<3} {on_frame}      {off_frame}")
    print("查询指令 FF")


def run_actions(relay, args):
    """按参数顺序执行动作, 返回进程退出码。"""
    ok = True
    if args.detect:
        detected = relay.detect_channel_count()
        if detected is None:
            print("[FAIL] 路数识别失败: 未收到有效返回")
            ok = False
        else:
            print(f"[OK]   检测到 {detected} 路继电器 (配置路数 {relay.channel_count})")

    if args.status:
        states = relay.query_status()
        if states is None:
            print("[FAIL] FF 状态查询无有效返回")
            ok = False
        else:
            print(f"[OK]   状态: {format_states(states)}")

    if args.channel is not None:
        if args.on == args.off:
            print("[FAIL] --channel 需要且只能配合 --on 或 --off 之一")
            return 2
        requested = bool(args.on)
        verified = relay.set_channel(args.channel, requested, verify=True)
        state_text = "ON" if requested else "OFF"
        if verified:
            print(f"[OK]   第{args.channel}路 {state_text} (FF 回读一致)")
        else:
            print(f"[FAIL] 第{args.channel}路 {state_text} 未通过 FF 回读确认")
            ok = False

    if args.all_off:
        if relay.all_off(verify=True):
            print("[OK]   全部通道已关闭")
        else:
            print("[FAIL] 全部关闭未通过 FF 回读确认")
            ok = False

    if args.all_on:
        # 风险: 同时吸合全部通道, 只应在确认负载/电源余量后手动执行
        if relay.all_on(verify=True):
            print("[OK]   全部通道已打开")
        else:
            print("[FAIL] 全部打开未通过 FF 回读确认")
            ok = False

    if args.blink is not None:
        # 风险: 反复吸合同一路, 注意负载与继电器机械寿命
        if args.repeat <= 0 or args.interval <= 0:
            print("[FAIL] --blink 需要 --repeat 和 --interval 为正数")
            return 2
        for index in range(args.repeat):
            for requested in (True, False):
                if not relay.set_channel(args.blink, requested, verify=True):
                    print(f"[FAIL] 第{args.blink}路切换 {'ON' if requested else 'OFF'} 未通过确认")
                    return 1
                print(
                    f"[OK]   第 {index + 1}/{args.repeat} 次 第{args.blink}路 "
                    f"{'ON' if requested else 'OFF'}"
                )
                time.sleep(args.interval)

    return 0 if ok else 1


def main(argv=None):
    args = parse_args(argv)

    if args.print_frames:
        print_frames(args.channels)

    if args.list:
        print("当前串口:")
        print(f"  {format_port_list()}")

    has_action = any(
        (
            args.status,
            args.detect,
            args.channel is not None,
            args.all_off,
            args.all_on,
            args.blink is not None,
        )
    )
    if not has_action:
        if not args.list and not args.print_frames:
            print(
                "未指定任何动作; 请使用 --detect / --status / --channel/--on/--off / --all-off / --blink。"
            )
            print(f"可用串口: {format_port_list()}")
            return 2
        return 0

    logger.remove()
    logger.add(sys.stderr, level="DEBUG" if args.debug else "INFO")

    try:
        relay = LCUSRelay(
            port=args.port,
            baudrate=args.baud,
            channel_count=args.channels,
            query_timeout=args.query_timeout,
        )
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        print(f"可用串口: {format_port_list()}")
        return 2

    try:
        relay.open()
    except Exception as exc:
        print(f"[FAIL] 打开继电器串口失败: {exc}")
        return 1

    try:
        return run_actions(relay, args)
    except KeyboardInterrupt:
        print("\n[中断] 用户终止; 如需断开全部通道请执行 --all-off")
        return 130
    except Exception as exc:
        print(f"[FAIL] 操作失败: {exc}")
        return 1
    finally:
        # 注意: close() 只关串口, 不会断开已吸合的触点(继电器自锁);
        # 需要断开必须在上面显式执行 --all-off。
        relay.close()


if __name__ == "__main__":
    sys.exit(main())

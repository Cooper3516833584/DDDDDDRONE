#!/usr/bin/env python3
"""8 路 LCUS 继电器顺序测试：从第 1 路开始，每次只开一路，单路保持 30 秒。

流程（与需求一致）:

    1. 打开继电器串口, 先用 FF 只读识别板子实际路数;
    2. 预置安全状态: 先断开全部通道 (避免上一次运行留下吸合);
    3. 从第 1 路开始逐路测试, 每一路: 打开 -> FF 回读确认 ->
       保持 30 秒 -> 关闭 -> FF 回读确认 -> 下一路;
       任意时刻只有一路处于吸合状态;
    4. 第 8 路结束后程序自行结束, 结束前再次断开全部通道并回读确认。

会真实吸合继电器触点; 不导入飞控, 不打开飞控串口, 不发送任何飞控命令。

用法::

    python3 testcode/test_relay_lcus_sequence.py --port /dev/ttyUSB0
    python3 testcode/test_relay_lcus_sequence.py --port COM3 --channels 4   # 4 路板联调
    python3 testcode/test_relay_lcus_sequence.py --port COM3 --hold 5       # 缩短单路保持时间

``--port`` 省略时依次取环境变量 ``D_TASK_RELAY_PORT`` 和驱动内置默认端口
(机载上位机实测的 by-path); PC 上必须显式给 ``COMx``。

风险与边界:

- 会真实吸合继电器, 请先确认负载安全或空载; 端口必须显式给出, 不要猜 ``/dev/ttyUSB*`` 编号
  (CH340 的 USB ID 与 HC-14 电台相同, 不能按 VID/PID 自动识别)。
- 单路保持时间由 ``--hold`` 控制, 默认 30 秒; 8 路全程约 4 分钟。
- 某一路打开未通过 FF 回读确认时, 不会进入保持阶段, 而是立刻补发关闭并按失败记录, 继续下一路;
  结束时打印未通过确认的通道清单 (退出码 1)。
- Ctrl+C 会中断测试, 并在退出路径断开全部通道; 但被强杀 (SIGKILL/拔电) 时 ``finally`` 不会执行,
  已吸合的通道会保持自锁状态, 需要人工处理。
- 实测 4 路板存在"控制帧后约 50ms 内回读仍是旧状态"的现象, 因此打开/关闭时常会先出现一次
  "校验失败 -> 同状态重发 -> 确认成功", 属预期行为, 不是接线故障。
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
    format_port_list,
    format_states,
)

HOLD_SECONDS_DEFAULT = 30.0
COUNTDOWN_LOG_STEP_SECONDS = 5.0
LOOP_SLEEP_SECONDS = 0.2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="8 路继电器顺序测试: 从第 1 路开始, 每次只开一路, 单路保持 30 秒",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port",
        default=None,
        help=f"继电器串口, 例如 /dev/ttyUSB0 或 COM3; 省略时取环境变量 {RELAY_PORT_ENV} 或驱动默认端口",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="波特率, 默认 9600")
    parser.add_argument(
        "--channels",
        type=int,
        default=DEFAULT_CHANNEL_COUNT,
        help="测试路数, 默认 8 (从第 1 路开始)",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=HOLD_SECONDS_DEFAULT,
        help="单路保持秒数, 默认 30",
    )
    parser.add_argument("--query-timeout", type=float, default=1.0, help="一次 FF 查询的总超时秒数, 默认 1.0")
    parser.add_argument("--retries", type=int, default=1, help="回读校验失败后的重发次数, 默认 1")
    parser.add_argument("--debug", action="store_true", help="输出 DEBUG 级日志")
    return parser.parse_args(argv)


def hold_channel(channel: int, seconds: float) -> None:
    """保持 seconds 秒, 期间每 COUNTDOWN_LOG_STEP_SECONDS 秒打印一次剩余时间。

    用小步 sleep 轮询, 保证 Ctrl+C 能及时中断; 循环有明确退出条件。
    """
    deadline = time.monotonic() + seconds
    next_log_at = time.monotonic() + COUNTDOWN_LOG_STEP_SECONDS
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        now = time.monotonic()
        if now >= next_log_at:
            logger.info(f"[SEQ] 第{channel}路保持中, 剩余 {remaining:.0f}s")
            next_log_at = now + COUNTDOWN_LOG_STEP_SECONDS
        time.sleep(min(LOOP_SLEEP_SECONDS, remaining))


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.hold <= 0:
        print("[FAIL] --hold 必须为正数")
        return 2
    if args.channels < 1:
        print("[FAIL] --channels 必须为正数")
        return 2

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

    failed_channels = []
    exit_code = 0
    try:
        logger.info("=" * 64)
        logger.info(
            f"[SEQ] 端口 {relay.port} @ {relay.baudrate} 8N1; "
            f"计划测试 {args.channels} 路, 单路保持 {args.hold:g}s"
        )
        logger.info("[SEQ] 本程序会真实吸合继电器, 请确认负载安全或处于空载状态")
        logger.info("=" * 64)

        # 只读识别(仅发 FF), 用来核对板子实际路数与 --channels 是否一致
        detected = relay.detect_channel_count()
        if detected is None:
            logger.warning("[SEQ] 路数识别失败(FF 无有效返回), 仍按 --channels 继续")
        else:
            logger.info(f"[SEQ] 板子实际上报 {detected} 路")
            if detected < args.channels:
                logger.warning(
                    f"[SEQ] 板子路数({detected})少于测试路数({args.channels}); "
                    f"第 {detected + 1}~{args.channels} 路预计无法通过回读, 可用 --channels {detected} 重跑"
                )

        # 预置安全状态: 先断开全部通道
        logger.info("[SEQ] 预置: 断开全部通道")
        if not relay.all_off(verify=False):
            logger.warning("[SEQ] 预置全关未全部成功, 继续按逐路测试")

        for channel in range(1, args.channels + 1):
            logger.info(f"[SEQ] ===== 第 {channel}/{args.channels} 路: 打开 =====")
            if not relay.set_channel(channel, True, verify=True, retries=args.retries):
                logger.error(f"[SEQ] 第{channel}路打开未通过 FF 回读确认, 跳过保持阶段")
                failed_channels.append(channel)
                # 安全兜底: 无论回读结果如何都补发一次关闭
                relay.set_channel(channel, False, verify=False)
                continue

            hold_channel(channel, args.hold)

            logger.info(f"[SEQ] ===== 第 {channel}/{args.channels} 路: 关闭 =====")
            if not relay.set_channel(channel, False, verify=True, retries=args.retries):
                logger.error(f"[SEQ] 第{channel}路关闭未通过 FF 回读确认")
                failed_channels.append(channel)

            states = relay.query_status()
            if states is not None:
                logger.info(f"[SEQ] 当前状态: {format_states(states)}")

        logger.info("[SEQ] 第 1 路至第最后一路测试流程执行完毕")
    except KeyboardInterrupt:
        logger.warning("[SEQ] 用户中断, 退出前断开全部通道")
        exit_code = 130
    except Exception as exc:
        logger.exception(f"[SEQ] 测试异常中止: {exc}")
        exit_code = 1
    finally:
        try:
            if relay.all_off(verify=True):
                logger.info("[SEQ] 退出前已确认全部通道断开")
            else:
                logger.error("[SEQ] 退出前全关未通过 FF 回读确认, 请人工确认继电器状态")
                exit_code = 1
        except Exception as exc:
            logger.error(f"[SEQ] 退出前断开全部通道失败: {exc}")
            exit_code = 1
        relay.close()

    if failed_channels:
        logger.warning(f"[SEQ] 未通过 FF 回读确认的通道: {failed_channels}")
        if exit_code == 0:
            exit_code = 1
    if exit_code == 0:
        logger.info("[SEQ] 程序正常结束")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

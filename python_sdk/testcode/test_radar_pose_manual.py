"""
手动移动飞机时观察单雷达位姿，不连接飞控、不解锁、不起飞。

启动后先保持飞机静止；出现 READY 后再缓慢向前、向左和原地旋转。
运行前必须确认没有其他程序占用雷达串口。
"""

import argparse
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Optional, Tuple


SDK_DIR = Path(__file__).resolve().parent.parent
if str(SDK_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_DIR))

from FlightController.Components import LD_Radar  # noqa: E402


READY_TIMEOUT = 15.0
BASELINE_SAMPLES = 20
POSE_UPDATE_TIMEOUT = 0.6
JUMP_WARNING_CM = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="不连接飞控，手动移动飞机并实时打印雷达位置"
    )
    parser.add_argument(
        "--radar-port",
        default=None,
        help="雷达串口；默认按 VID:PID 自动搜索",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="打印间隔，单位秒，默认 0.1",
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    return args


def snapshot(radar: LD_Radar) -> Tuple[float, float, float]:
    x, y, yaw = radar.rt_pose
    return float(x), float(y), float(yaw)


def initialized(radar: LD_Radar) -> Tuple[bool, bool, bool]:
    state = getattr(radar, "_rt_pose_inited", [False, False, False])
    return bool(state[0]), bool(state[1]), bool(state[2])


def state_text(state: Tuple[bool, bool, bool]) -> str:
    return "x={} y={} yaw={}".format(
        "OK" if state[0] else "--",
        "OK" if state[1] else "--",
        "OK" if state[2] else "--",
    )


def wait_until_ready(radar: LD_Radar) -> None:
    """打印三轴初始化过程，防止把未初始化的零值当作原点。"""
    deadline = time.monotonic() + READY_TIMEOUT
    last_state: Optional[Tuple[bool, bool, bool]] = None
    next_report = 0.0

    while time.monotonic() < deadline:
        radar.rt_pose_update_event.wait(0.2)
        radar.rt_pose_update_event.clear()
        now = time.monotonic()
        state = initialized(radar)
        if state != last_state or now >= next_report:
            x, y, yaw = snapshot(radar)
            print(
                "[INIT] connected={} {} raw=({:.2f}, {:.2f}, {:.2f})".format(
                    radar.connected,
                    state_text(state),
                    x,
                    y,
                    yaw,
                ),
                flush=True,
            )
            last_state = state
            next_report = now + 0.5
        if radar.connected and all(state):
            return

    raise RuntimeError(
        "雷达三轴未在 {:.0f}s 内初始化：connected={} {}".format(
            READY_TIMEOUT,
            radar.connected,
            state_text(initialized(radar)),
        )
    )


def collect_baseline(radar: LD_Radar) -> Tuple[float, float, float]:
    """静止采集多帧，以中位数作为手动测试原点。"""
    samples = []
    deadline = time.monotonic() + READY_TIMEOUT
    radar.rt_pose_update_event.clear()

    while len(samples) < BASELINE_SAMPLES and time.monotonic() < deadline:
        if radar.rt_pose_update_event.wait(POSE_UPDATE_TIMEOUT):
            radar.rt_pose_update_event.clear()
            if radar.connected and all(initialized(radar)):
                samples.append(snapshot(radar))
    if len(samples) < BASELINE_SAMPLES:
        raise RuntimeError(
            "原点采样不足：需要{}帧，收到{}帧".format(
                BASELINE_SAMPLES,
                len(samples),
            )
        )

    baseline = tuple(
        statistics.median(sample[axis] for sample in samples)
        for axis in range(3)
    )
    spread = tuple(
        max(sample[axis] for sample in samples)
        - min(sample[axis] for sample in samples)
        for axis in range(3)
    )
    print(
        "[BASE] raw=({:.2f}, {:.2f}, {:.2f}) "
        "range=({:.2f}cm, {:.2f}cm, {:.2f}deg)".format(
            baseline[0],
            baseline[1],
            baseline[2],
            spread[0],
            spread[1],
            spread[2],
        ),
        flush=True,
    )
    return baseline


def angle_delta(angle: float, origin: float) -> float:
    return (angle - origin + 180.0) % 360.0 - 180.0


def monitor(
    radar: LD_Radar,
    baseline: Tuple[float, float, float],
    interval: float,
) -> None:
    base_x, base_y, base_yaw = baseline
    start_time = time.monotonic()
    next_print = start_time
    last_time: Optional[float] = None
    last_pose: Optional[Tuple[float, float, float]] = None
    radar.rt_pose_update_event.clear()
    print(
        "[READY] 可缓慢移动；x向前为正，y向左为正，Ctrl+C退出",
        flush=True,
    )

    while True:
        if not radar.rt_pose_update_event.wait(POSE_UPDATE_TIMEOUT):
            print("[STALE] 0.6s内没有新的雷达位姿", flush=True)
            continue
        radar.rt_pose_update_event.clear()
        now = time.monotonic()
        if now < next_print:
            continue
        next_print = now + interval

        state = initialized(radar)
        if not radar.connected or not all(state):
            print(
                "[INVALID] connected={} {}".format(
                    radar.connected,
                    state_text(state),
                ),
                flush=True,
            )
            continue

        x, y, yaw = snapshot(radar)
        step = 0.0
        speed = 0.0
        if last_pose is not None and last_time is not None:
            step = math.hypot(x - last_pose[0], y - last_pose[1])
            speed = step / max(now - last_time, 1e-6)
        warning = " **JUMP**" if step > JUMP_WARNING_CM else ""
        print(
            "[POSE +{:6.2f}s] raw=({:8.2f}, {:8.2f}, {:7.2f}) "
            "relative=({:8.2f}, {:8.2f}, {:7.2f}) "
            "step={:6.2f}cm speed={:7.2f}cm/s{}".format(
                now - start_time,
                x,
                y,
                yaw,
                x - base_x,
                y - base_y,
                angle_delta(yaw, base_yaw),
                step,
                speed,
                warning,
            ),
            flush=True,
        )
        last_pose = (x, y, yaw)
        last_time = now


def main() -> int:
    args = parse_args()
    radar: Optional[LD_Radar] = None
    try:
        print(
            "[SAFE] 不连接飞控、不解锁、不起飞；请保持飞控锁定。",
            flush=True,
        )
        radar = LD_Radar()
        radar.debug = False
        radar.start(com=args.radar_port)
        radar.start_resolve_pose(rotation_adapt=True)
        wait_until_ready(radar)
        print("[BASE] 保持静止，正在采集原点……", flush=True)
        monitor(radar, collect_baseline(radar), args.interval)
        return 0
    except KeyboardInterrupt:
        print("\n[STOP] 用户结束测试", flush=True)
        return 130
    except Exception as exc:
        print("[ERROR] {}: {}".format(type(exc).__name__, exc), flush=True)
        return 1
    finally:
        if radar is not None:
            try:
                radar.stop_resolve_pose()
            except Exception as exc:
                print("[WARN] 停止位姿解算失败：{}".format(exc), flush=True)
            try:
                if radar.running:
                    radar.stop()
            except Exception as exc:
                print("[WARN] 停止雷达失败：{}".format(exc), flush=True)


if __name__ == "__main__":
    sys.exit(main())

"""
在三易串口屏上显示机载上位机的 T265 USB 枚举状态。

本程序只执行两类操作：
1. 只读扫描 /sys/bus/usb/devices；
2. 通过已运行的 FC_Server 更新串口屏控件 ros.ellipse_t265.fColor。

不会启动 realsense-viewer、ROS、T265 驱动或飞行任务，也不会直连飞控串口。

已确认的 USB/内核产品名称：
    03e7:2150  Movidius MA2X5X
    8087:0b37  Intel(R) RealSense(TM) Tracking Camera T265

显示规则：
    仅检测到 T265：White，0xFFFFFFFF
    VPU、VPU/T265 混合、设备缺失或扫描异常：Black，0xFF000000

前置条件：
    server_ros.py 已运行并监听 TCP 5654，且没有其他 FC_Client 占用
    FC_Server 的单客户端连接。当前自启动配置会同时启动
    2026_disaster_survey.py；测试本程序前需先按现有安全流程停止该任务，
    但保留 server_ros.py 运行。

在机载上位机的 python_sdk 目录运行：
    python3 testcode/test_t265_screen_status.py

远程连接机载上位机上的 FC_Server：
    python3 testcode/test_t265_screen_status.py --host 192.168.31.176
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Set, Tuple


SDK_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)


VPU_USB_ID = ("03e7", "2150")
T265_USB_ID = ("8087", "0b37")

STATE_ABSENT = "absent"
STATE_VPU = "vpu"
STATE_T265 = "t265"
STATE_MIXED = "mixed"
STATE_ERROR = "error"

SCREEN_WIDGET_PROPERTY = "ros.ellipse_t265.fColor"
COLOR_BLACK = "0xFF000000"
COLOR_WHITE = "0xFFFFFFFF"

DEFAULT_SYSFS_USB_ROOT = Path("/sys/bus/usb/devices")


def _read_sysfs_value(path: Path) -> Optional[str]:
    try:
        return path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError):
        return None


def scan_t265_usb_devices(
    sysfs_root: Path = DEFAULT_SYSFS_USB_ROOT,
) -> Tuple[str, List[Tuple[str, str, str]]]:
    """
    返回 (状态, 匹配设备列表)。

    匹配设备列表元素为 (sysfs 目录名, USB ID, product 名称)。
    USB 拔插期间目录可能随时消失，因此单个目录读取失败时跳过。
    """
    found_ids: Set[Tuple[str, str]] = set()
    matched_devices: List[Tuple[str, str, str]] = []

    try:
        devices = list(sysfs_root.iterdir())
    except OSError:
        return STATE_ERROR, matched_devices

    for device_dir in devices:
        vendor = _read_sysfs_value(device_dir / "idVendor")
        product_id = _read_sysfs_value(device_dir / "idProduct")
        if vendor is None or product_id is None:
            continue

        usb_id = (vendor.lower(), product_id.lower())
        if usb_id not in (VPU_USB_ID, T265_USB_ID):
            continue

        found_ids.add(usb_id)
        product_name = _read_sysfs_value(device_dir / "product") or "unknown"
        matched_devices.append(
            (device_dir.name, f"{usb_id[0]}:{usb_id[1]}", product_name)
        )

    has_vpu = VPU_USB_ID in found_ids
    has_t265 = T265_USB_ID in found_ids
    if has_vpu and has_t265:
        state = STATE_MIXED
    elif has_t265:
        state = STATE_T265
    elif has_vpu:
        state = STATE_VPU
    else:
        state = STATE_ABSENT
    return state, matched_devices


def color_for_state(state: str) -> str:
    """只有明确的纯 T265 状态显示白色，其余状态全部故障安全地显示黑色。"""
    return COLOR_WHITE if state == STATE_T265 else COLOR_BLACK


def format_devices(devices: List[Tuple[str, str, str]]) -> str:
    if not devices:
        return "none"
    return ", ".join(
        f"{sysfs_name} {usb_id} {product_name}"
        for sysfs_name, usb_id, product_name in devices
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过 FC_Client 将 T265 USB 枚举状态显示到三易串口屏"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="FC_Server 地址，机载本机运行时保持默认 127.0.0.1",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5654,
        help="FC_Server 端口，默认 5654",
    )
    parser.add_argument(
        "--authkey",
        default="fc",
        help="FC_Server 认证密钥，默认 fc",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="USB 状态轮询间隔秒数，默认 1.0",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="连接 FC_Server 的超时秒数，默认 5.0",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0:
        print("[T265_SCREEN] --interval 必须大于 0", file=sys.stderr)
        return 2
    if args.connect_timeout <= 0:
        print("[T265_SCREEN] --connect-timeout 必须大于 0", file=sys.stderr)
        return 2

    # 延迟导入，便于在无飞控、无硬件环境中单独验证 USB 状态解析函数。
    from FlightController import FC_Client
    from FlightController.Components.UartScreen import UARTScreen

    fc = FC_Client()
    screen = None
    connected = False
    last_state: Optional[str] = None
    last_sent_color: Optional[str] = None

    print(
        f"[T265_SCREEN] 连接 FC_Server {args.host}:{args.port}；"
        "本程序不会直连飞控串口"
    )
    try:
        fc.connect(
            host=args.host,
            port=args.port,
            authkey=args.authkey.encode("utf-8"),
            print_state=False,
            # 屏蔽不需要的高频数据，只保留未列入过滤器的串口屏数据。
            filters=("state", "ack", "event", "radar"),
            block=True,
            timeout=args.connect_timeout,
        )
        connected = True
        screen = UARTScreen(fc)

        # 启动时先明确写入安全默认值，避免保留上一次程序留下的白色。
        screen.set_widget_value(SCREEN_WIDGET_PROPERTY, COLOR_BLACK)
        last_sent_color = COLOR_BLACK
        print(
            f"[T265_SCREEN] 已将 {SCREEN_WIDGET_PROPERTY} 初始化为 "
            f"{COLOR_BLACK}"
        )

        while fc.running:
            state, devices = scan_t265_usb_devices()
            if state != last_state:
                print(
                    f"[T265_SCREEN] USB 状态={state}; "
                    f"devices={format_devices(devices)}"
                )
                last_state = state

            target_color = color_for_state(state)
            if target_color != last_sent_color:
                try:
                    screen.set_widget_value(
                        SCREEN_WIDGET_PROPERTY,
                        target_color,
                    )
                    last_sent_color = target_color
                    print(
                        f"[T265_SCREEN] {SCREEN_WIDGET_PROPERTY}="
                        f"{target_color}"
                    )
                except Exception as exc:
                    # FC_Client 会在后台尝试重连；保留 None 以便下一轮重发。
                    last_sent_color = None
                    print(
                        f"[T265_SCREEN] 更新串口屏失败，将重试：{exc}",
                        file=sys.stderr,
                    )

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[T265_SCREEN] 用户中断")
    except Exception as exc:
        print(
            f"[T265_SCREEN] 运行失败：{exc}\n"
            "[T265_SCREEN] 请确认 server_ros.py 正在运行，且没有其他 "
            "FC_Client 占用其单客户端连接。",
            file=sys.stderr,
        )
        return 1
    finally:
        # 监控停止后恢复黑色，避免白色被误认为仍在实时监测。
        if connected and screen is not None:
            try:
                screen.set_widget_value(SCREEN_WIDGET_PROPERTY, COLOR_BLACK)
                print(
                    f"[T265_SCREEN] 退出前已恢复 {SCREEN_WIDGET_PROPERTY}="
                    f"{COLOR_BLACK}"
                )
            except Exception as exc:
                print(
                    f"[T265_SCREEN] 退出时恢复黑色失败：{exc}",
                    file=sys.stderr,
                )
        if fc.running:
            fc.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

import os
import ast
import math
import re
import time
from typing import List, Literal

os.chdir(os.path.dirname(os.path.abspath(__file__)))
import threading

from FlightController import FC_Server
from FlightController.Components.RosManager import RosManager
from FlightController.Components.LioPoseProvider import LioPoseProvider
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Components.Utils import Tmux
from loguru import logger

fc = FC_Server()
scr = UARTScreen(fc=fc)
rm = RosManager()
mis_tmux = Tmux("mission")
PATH = os.path.dirname(os.path.abspath(__file__))
PYTHON_EXCUTEABLE = "python3"
mis_num = 0
packages = [
    (0, ("livox_ros_driver2", "msg_MID360s_launch.py"), []),
    (0, ("fast_lio", "mapping.launch.py", "config_file:=mid360s_drone.yaml rviz:=false"), []),
]
REQUIRED_LIO_TOPICS = ("/livox/lidar", "/livox/imu", "/Odometry", "/Odometry_highrate")


def missing_lio_topics(topics):
    available = set(topics)
    return [topic for topic in REQUIRED_LIO_TOPICS if topic not in available]


def require_production_localization():
    config = os.path.join(PATH, "../ros2_ws/src/FAST_LIO_ROS2/config/mid360s_drone.yaml")
    if not os.path.isfile(config):
        raise RuntimeError("Measured MID360S LiDAR-to-IMU production config is missing")
    with open(config, encoding="utf-8") as stream:
        yaml_text = stream.read()
    if ("REQUIRED_MEASURED" in yaml_text or
            not re.search(r"^\s*extrinsic_est_en:\s*false\s*(?:#.*)?$", yaml_text, re.M)):
        raise RuntimeError("MID360S production extrinsic is not finalized")
    for key, length in (("extrinsic_T", 3), ("extrinsic_R", 9)):
        match = re.search(rf"^\s*{key}:\s*(\[[^\]]+\])", yaml_text, re.M)
        try:
            values = ast.literal_eval(match.group(1)) if match else None
            if len(values) != length or not all(math.isfinite(float(v)) for v in values):
                raise ValueError(key)
        except (TypeError, ValueError, SyntaxError):
            raise RuntimeError(f"Measured MID360S {key} is required")
        if key == "extrinsic_T" and values == [-0.011, -0.02329, 0.04412]:
            raise RuntimeError("Ordinary MID360 example extrinsic is forbidden")
    LioPoseProvider()  # Measured IMU-to-body mounting transform is mandatory.


def run_item(item, kill_exist=True):
    if item[1][0] == "fast_lio":
        require_production_localization()
    for dev in item[2]:
        rm.chmod(dev)
    if item[0] == 0:
        rm.launch_package(*item[1], kill_exist=kill_exist)
    else:
        rm.run_package(*item[1], kill_exist=kill_exist)


def set_ellipse(name: str, state: Literal[0, 1, 2]):
    CMP = [
        "0xffED4351",  # 0: error
        "0xffFCE123",  # 1: warning
        "0xff33DB33",  # 2: ok
    ]
    scr.set_widget_value(f"ellipse_{name}.fColor", CMP[state])


def check_pack(pack, wname):
    if not rm.is_running(pack):
        set_ellipse(wname, 0)
    elif not rm.is_live(pack):
        set_ellipse(wname, 1)
    else:
        set_ellipse(wname, 2)


sending_log = False


def callback(cmd: str):
    global mis_num
    global sending_log
    try:
        if not cmd.startswith(("ros", "mis")) or sending_log:
            return
        if cmd.startswith("ros_boot="):
            idx = int(cmd.split("=")[1])
            if idx == 0:
                for item in packages:
                    run_item(item)
            else:
                run_item(packages[idx - 1])
        elif cmd.startswith("ros_kill="):
            idx = int(cmd.split("=")[1])
            if idx == 0:
                for item in packages:
                    rm.kill_package(item[1][0])
            else:
                rm.kill_package(packages[idx - 1][1][0])
        elif cmd.startswith("ros_log="):
            idx = int(cmd.split("=")[1])
            lines: List[str] = []
            if idx == 0:
                for item in packages:
                    lines.append(f"\n> {item[1][0]}:\n\n")
                    log = rm.get_log(item[1][0], 2)
                    lines.extend([line.replace("\n", "").replace("\r", "") for line in log])
            else:
                lines.extend(rm.get_log(packages[idx - 1][1][0], 10))
            if len(lines) == 0:
                lines.append("> No log")
            sending_log = True
            scr.set_widget_value("var_log.val", 1)
            time.sleep(0.05)
            for line in lines:
                if len(line) > 220:
                    line = line[:100] + "..." + line[-115:]
                scr.send_string(line)
                time.sleep(0.05)
            scr.set_widget_value("var_log.val", 0)
            sending_log = False
        elif cmd.startswith("ros_state"):
            topics = rm.get_running_topics()
            set_ellipse("scan", 2 if "/livox/lidar" in topics else 0)
            set_ellipse("camera", 2 if "/livox/imu" in topics else 0)
            set_ellipse("map", 2 if "/Odometry" in topics else 0)
            set_ellipse("radar", 2 if "/Odometry_highrate" in topics else 0)
            set_ellipse("t265", 0)
            set_ellipse("cart", 0)
            set_ellipse("tf2", 0)
        elif cmd.startswith("mis_state"):
            scr.set_widget_value("main_volt.txt", f'"{fc.state.bat.value:.2f}V"')
            if mis_tmux.session_running:
                if mis_tmux.session_busy:
                    scr.set_widget_value("main_mis.txt", f'"繁忙/{mis_num}"')
                else:
                    scr.set_widget_value("main_mis.txt", f'"已结束"')
            else:
                scr.set_widget_value("main_mis.txt", f'"空闲"')
        elif cmd.startswith("mis_kill"):
            try:
                mis_tmux.send_key_interruption()
                time.sleep(1)
            except:
                pass
            finally:
                mis_tmux.kill_session()
            scr.set_widget_value("main_info.txt", f'"已结束任务"')
        elif cmd.startswith("mis_boot="):
            require_production_localization()
            mis_num = int(cmd.split("=")[1]) + 1
            if not os.path.exists(f"{PATH}/mission{mis_num}.py"):
                scr.set_widget_value("main_info.txt", f'"任务{mis_num}不存在"')
                return
            if mis_tmux.session_running:
                scr.set_widget_value("main_info.txt", f'"请先终止任务"')
                return
            missing = missing_lio_topics(rm.get_running_topics())
            if missing:
                logger.error("[US] Mission refused; missing LIO topics: {}", ", ".join(missing))
                scr.set_widget_value("main_info.txt", '"定位链未就绪"')
                return
            logger.info(f"[US] Start mission {mis_num}")
            scr.set_widget_value("main_info.txt", f'"正在启动任务{mis_num}"')
            mis_tmux.new_session()
            time.sleep(1)
            mis_tmux.send_command(f"cd {PATH}")
            time.sleep(1)
            mis_tmux.send_command(f"{PYTHON_EXCUTEABLE} ./mission{mis_num}.py")
            scr.set_widget_value("main_info.txt", f'"任务{mis_num}已开始运行"')
        elif cmd.startswith("mis_poweroff"):
            scr.set_widget_value("main_info.txt", f'"系统正在关机"')
            time.sleep(1)
            os.system("sudo poweroff")
    except Exception as e:
        logger.exception("UartScreen callback error")


scr.register_report_callback(lambda x: threading.Thread(target=callback, args=(x,)).start())

# for item in packages:
#     run_item(item, False)

fc.start_listen_serial(print_state=True, block_until_connected=True)
fc.serve_forever(indicator=True)

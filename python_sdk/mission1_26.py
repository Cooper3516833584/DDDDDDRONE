"""
单雷达定位的任务一程序。

坐标与单位：
- 水平坐标和高度均为 cm；
- x 向前为正，y 向左为正；
- PURSUIT_SPEED 和 ESCORT_SPEED 是 set_navigation_speed() 的参数，
  不是对无人机实际速度的保证。

当前占位行为：
- 起飞信号尚未接入，wait_for_takeoff_signal() 当前立即返回；
- 视觉尚未接入，wait_until_target_detected() 当前在前飞 3 秒后返回 True。
"""

import threading
import time

import numpy as np
from loguru import logger

from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Solutions.Navigation import Navigation


FC_SERIAL_DEV = "/dev/ttyACM0"

TAKEOFF_POINT = np.array([0.0, 0.0])
ENTRY_POINT = np.array([87.5, -37.5])
CRUISE_HEIGHT = 150
VERTICAL_SPEED = 20

PURSUIT_SPEED = 30
ESCORT_SPEED = 10
TARGET_DETECTION_PLACEHOLDER_SECONDS = 3.0
ESCORT_OUTPUT_ON_SECONDS = 5.0
ESCORT_OUTPUT_OFF_SECONDS = 2.0

# 仅用作持续沿 +x 飞行的 PID 引导目标，不代表任务要求到达该点。
# 后续接入真实视觉时，应结合实际场地边界调整，并保留未发现目标时的停止条件。
FORWARD_GUIDANCE_DISTANCE = 300.0


class Mission:
    def __init__(
        self,
        fc: FC_Controller,
        radar: LD_Radar,
        navi: Navigation,
        stop_event: threading.Event,
    ):
        self.fc = fc
        self.radar = radar
        self.navi = navi
        self.stop_event = stop_event
        self.takeoff_signal = threading.Event()

    def stop(self):
        self.stop_event.set()
        self.navi.stop()
        logger.info("[MISSION] Mission stopped")

    def notify_takeoff_signal(self):
        """供后续无线、按键或其他信号回调通知起飞。"""
        self.takeoff_signal.set()

    def wait_for_takeoff_signal(self):
        """
        起飞信号占位函数。

        TODO: 后续合作者接入真实信号源时，让信号回调调用
        notify_takeoff_signal()，再取消下面三行等待代码的注释。
        当前不等待任何外部信号，会立即继续任务。
        """
        logger.warning(
            "[MISSION] Takeoff signal is not implemented; placeholder continues immediately"
        )
        # self.takeoff_signal.clear()
        # self.takeoff_signal.wait()
        # self.takeoff_signal.clear()

    def _wait_with_stop(self, duration_s: float) -> bool:
        """等待指定时间；收到停止请求时立即返回 False。"""
        deadline = time.monotonic() + duration_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True
            if self.stop_event.wait(min(0.05, remaining)):
                return False

    def wait_until_target_detected(self) -> bool:
        """
        视觉检测占位函数。

        TODO: 后续将此函数体替换为摄像头检测循环；检测到目标时返回 True，
        收到 stop_event、定位失效或达到场地安全边界时返回 False。
        当前以“沿 +x 前飞 3 秒后发现目标”模拟视觉结果。
        """
        logger.warning(
            "[MISSION] Vision is not implemented; target will be reported after {}s",
            TARGET_DETECTION_PLACEHOLDER_SECONDS,
        )
        if not self._wait_with_stop(TARGET_DETECTION_PLACEHOLDER_SECONDS):
            return False
        logger.info("[MISSION] Target detected by placeholder")
        return True

    def run(self):
        fc = self.fc
        navi = self.navi

        navi.set_navigation_speed(PURSUIT_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)
        navi.start(mode="radar")
        logger.info("[MISSION] Single-radar navigation started")

        # 以起飞位置建立任务坐标原点；必须先获得雷达位姿更新。
        navi.calibrate_basepoint()
        logger.info("[MISSION] Radar basepoint calibrated: {}", navi.basepoint)

        self.wait_for_takeoff_signal()
        if self.stop_event.is_set():
            return

        logger.info(
            "[MISSION] Pointing takeoff to {}cm at {}",
            CRUISE_HEIGHT,
            TAKEOFF_POINT,
        )
        navi.pointing_takeoff(TAKEOFF_POINT, CRUISE_HEIGHT)

        logger.info("[MISSION] Navigate to entry point {}", ENTRY_POINT)
        if not navi.navigation_to_waypoint(ENTRY_POINT, wait=True):
            raise RuntimeError("Failed to reach entry point")

        # 使用远端 +x 目标维持前飞方向。set_navigation_speed() 只是 PID 参数，
        # 实际速度仍由定位误差、PID 输出和飞行状态共同决定。
        forward_target = np.array(
            [ENTRY_POINT[0] + FORWARD_GUIDANCE_DISTANCE, ENTRY_POINT[1]]
        )
        navi.set_navigation_speed(PURSUIT_SPEED)
        navi.switch_pid("navi")
        navi.direct_set_waypoint(forward_target)
        logger.info(
            "[MISSION] Pursuing along +x with navigation-speed parameter {}",
            PURSUIT_SPEED,
        )

        if not self.wait_until_target_detected():
            raise RuntimeError("Target detection stopped or failed")

        # 不更换目标点，只收紧 PID 输出限幅，因此继续保持原 +x 方向。
        navi.set_navigation_speed(ESCORT_SPEED)
        logger.info(
            "[MISSION] Escorting with navigation-speed parameter {}",
            ESCORT_SPEED,
        )
        if not self._wait_with_stop(ESCORT_OUTPUT_ON_SECONDS):
            return

        fc.set_digital_output(0, False)
        logger.info("[MISSION] Digital output 0 disabled")

        if not self._wait_with_stop(ESCORT_OUTPUT_OFF_SECONDS):
            return

        # 终止 +x 引导并锁定当前位置，再创建返航轨迹，避免目标点竞争。
        navi.navigation_stop_here()
        logger.info("[MISSION] Returning to takeoff point {}", TAKEOFF_POINT)
        if not navi.navigation_to_waypoint(TAKEOFF_POINT, wait=True):
            raise RuntimeError("Failed to return to takeoff point")
        logger.info("[MISSION] Returned to takeoff point")


def main():
    fc = FC_Controller()
    radar = LD_Radar()
    stop_event = threading.Event()
    navi = None
    mission = None
    digital_output_enabled = False

    try:
        # server_ros.py 已关闭：本程序按单雷达方案直连飞控串口。
        fc.start_listen_serial(serial_dev=FC_SERIAL_DEV, print_state=False)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("Flight controller connection timeout")
        logger.info("[MANAGER] Flight controller connected")

        # 按任务要求，在确认飞控连接后立即打开数字输出 0。
        fc.set_digital_output(0, True)
        digital_output_enabled = True
        logger.info("[MANAGER] Digital output 0 enabled")

        radar.debug = False
        radar.start()
        logger.info("[MANAGER] Single radar started")

        navi = Navigation(fc=fc, radar=radar, stop_event=stop_event)
        mission = Mission(
            fc=fc,
            radar=radar,
            navi=navi,
            stop_event=stop_event,
        )
        mission.run()
    except KeyboardInterrupt:
        logger.warning("[MANAGER] Mission interrupted by user")
    except Exception:
        logger.exception("[MANAGER] Mission failed")
    finally:
        if mission is not None:
            mission.stop()
        elif navi is not None:
            navi.stop()

        if digital_output_enabled:
            try:
                fc.set_digital_output(0, False)
            except Exception:
                logger.exception("[MANAGER] Failed to disable digital output 0")

        # 正常返航后或异常退出时均执行已有的安全降落兜底。
        try:
            if fc.state.unlock.value:
                logger.warning("[MANAGER] Auto landing")
                fc.set_flight_mode(fc.PROGRAM_MODE)
                fc.stablize()
                fc.land()
                if not fc.wait_for_lock(timeout_s=20):
                    logger.error(
                        "[MANAGER] Landing lock not confirmed; keep landing command active "
                        "and refuse airborne force-lock"
                    )
                    fc.land()
        except Exception:
            logger.exception("[MANAGER] Auto landing failed")

        try:
            if radar.running:
                radar.stop()
        except Exception:
            logger.exception("[MANAGER] Failed to stop radar")

        fc.close()
        logger.info("[MANAGER] Mission finished")


if __name__ == "__main__":
    main()

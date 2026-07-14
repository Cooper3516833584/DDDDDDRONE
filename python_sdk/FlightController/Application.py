import threading
import time
from typing import Optional

import numpy as np
from FlightController.Protocal import FC_Protocol
from loguru import logger


class FC_Application(FC_Protocol):
    """
    应用层, 基于协议层进行开发, 不触及底层通信
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._realtime_control_thread: threading.Thread = None  # type: ignore
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self._realtime_control_running = False
        self._ground_station = None

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度: (程控模式下有效)
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        self._action_log(
            "set height",
            f"{'fusion' if source == 0 else 'lidar'}, {height}cm, {speed}cm/s",
        )
        if source == 0:
            alt = self.state.alt_fused
        elif source == 1:
            alt = self.state.alt_add
        self.state.update_event.clear()  # 确保使用的是最新的高度
        self.state.update_event.wait()
        if height < alt.value:
            self.go_down(alt.value - height, speed)
        elif height > alt.value:
            self.go_up(height - alt.value, speed)

    def set_yaw(self, yaw: int, speed: int) -> None:
        """
        设置偏航角: (程控模式下有效)
        偏航角:-180-180 度
        偏航速度:5-90 deg/s
        """
        self._action_log("set yaw", f"{yaw}deg, {speed}deg/s")
        current_yaw = self.state.yaw.value
        if yaw < current_yaw:
            left_turn_deg = abs(current_yaw - yaw)
            right_turn_deg = abs(360 - left_turn_deg)
        else:
            right_turn_deg = abs(current_yaw - yaw)
            left_turn_deg = abs(360 - right_turn_deg)
        if left_turn_deg < right_turn_deg:
            self.turn_left(left_turn_deg, speed)
        else:
            self.turn_right(right_turn_deg, speed)

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)
        self._action_log("reset position prediction")

    def rectangular_move(self, x: int, y: int, speed: int) -> None:
        """
        匿名坐标系下的水平移动 (程控模式下有效)
        x,y: 匿名坐标系下的坐标
        speed: 移动速度:10-300 cm/s

        移动半径在0-10000 cm
        """
        div = y / x if x != 0 else np.inf
        target_deg = -np.arctan(div) / np.pi * 180
        distance = np.sqrt(x**2 + y**2)
        if target_deg < 0:
            target_deg += 360
        self._action_log("cordinate move", f"{x}, {y}, {speed}")
        self.horizontal_move(int(distance), speed, int(target_deg))

    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待飞控连接
        """
        t0 = time.perf_counter()
        while not self.connected:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for fc connection timeout")
                return False
        self._action_log("wait ok", "fc connection")
        return True

    def wait_for_last_command_done(self, timeout_s=10) -> bool:
        """
        等待最后一次指令完成
        """
        t0 = time.perf_counter()
        time.sleep(0.5)  # 等待数据回传
        while not self.last_command_done:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for last command done timeout")
                return False
        self._action_log("wait ok", "last cmd done")
        return True

    def wait_for_hovering(self, timeout_s=10) -> bool:
        """
        等待进入悬停状态
        """
        t0 = time.perf_counter()
        time.sleep(0.5)  # 等待数据回传
        while not self.hovering:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for stabilizing timeout")
                return False
        self._action_log("wait ok", "stabilizing")
        return True

    def wait_for_lock(self, timeout_s=10) -> bool:
        """
        等待锁定
        """
        t0 = time.perf_counter()
        while self.state.unlock.value:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for lock timeout")
                return False
        self._action_log("wait ok", "locked")
        return True

    def wait_for_takeoff_done(self, z_speed_threshold=4, timeout_s=5) -> bool:
        """
        等待起飞完成
        """
        t0 = time.perf_counter()
        time.sleep(1)  # 等待加速完成
        while self.state.vel_z.value < z_speed_threshold:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for takeoff done timeout")
                return False
        if self.state.alt_add.value < 10:
            logger.warning("[FC] takeoff failed, low altitude")
            return False
        time.sleep(1)  # 等待机身高度稳定
        self._action_log("wait ok", "takeoff done")
        return True

    def programmed_takeoff(self, height: int, speed: int) -> None:
        """
        程控起飞模式,使用前先解锁 (危险,当回传频率过低时容易冲顶,应使用较小的安全速度)
        """
        self._action_log("manual takeoff start", f"{height}cm")
        last_mode = self.state.mode.value
        hgt = self.state.alt_add
        timeout_value = height / speed * 2  # 安全时间
        self.set_flight_mode(self.HOLD_POS_MODE)
        time.sleep(0.1)  # 等待模式设置完成
        self.send_realtime_control_data(vel_z=speed)
        takeoff_time = time.perf_counter()
        while hgt.value < height - 20:
            time.sleep(0.1)
            if time.perf_counter() - takeoff_time > timeout_value:  # 超时, 终止
                logger.warning("[FC] takeoff timeout, force stop")
                self.stablize()
                return
            else:
                self.send_realtime_control_data(vel_z=speed)  # 持续发送
        self.send_realtime_control_data(vel_z=10)
        while hgt.value < height - 2:
            time.sleep(0.1)
        self.stablize()
        self.set_flight_mode(last_mode)  # 还原模式
        self._action_log("manual takeoff ok")

    def safe_takeoff(self, target_height: int, climb_speed: int = 20, first_lift: int = 60):
        # 1) 确保连接与解锁
        self.wait_for_connection()
        self.set_flight_mode(self.PROGRAM_MODE)
        if not self.state.unlock.value:
            self.unlock()
            time.sleep(2.0)  # 等电机/状态稳定（Navigation 也这么做）

        # 2) 先一键起飞抬离地面（绕开地面保护）
        lift = min(max(40, first_lift), max(40, target_height))
        self.take_off(lift)
        time.sleep(6.0)
        self.wait_for_hovering(3)

        # 3) 再用 HOLD_POS + realtime control 拉到目标高度（空中 vel_z 才可靠）
        self.set_flight_mode(self.HOLD_POS_MODE)
        t0 = time.perf_counter()
        while self.state.alt_add.value < target_height - 5:
            # 每 0.1s 发一次，保证 >1Hz
            self.send_realtime_control_data(vel_z=climb_speed)
            time.sleep(0.1)
            if time.perf_counter() - t0 > 20:
                break
        self.stablize()

    def _realtime_control_task(self, freq):
        logger.info("[FC] realtime control task started")
        last_send_time = time.perf_counter()
        pauesed = False
        interval = 1 / freq
        while self._realtime_control_running:
            while time.perf_counter() - last_send_time < interval:
                time.sleep(0.01)
            last_send_time += interval
            if self.state.mode.value != self.HOLD_POS_MODE:
                if not pauesed:
                    self._action_log("realtime control", "paused")
                pauesed = True
                continue
            if pauesed:  # 取消暂停时先清空数据避免失控
                pauesed = False
                self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
                self._action_log("realtime control", "resumed")
            try:
                self.send_realtime_control_data(*self._realtime_control_data_in_xyzYaw)
            except Exception as e:
                logger.exception(f"[FC] realtime control task error")
        logger.info("[FC] realtime control task stopped")

    def start_realtime_control(self, freq: float = 15) -> None:
        """
        开始自动发送实时控制, 仅在定点模式下有效
        freq: 后台线程自动发送控制帧的频率

        警告: 除非特别需要, 否则不建议使用该方法, 而是直接调用 send_realtime_control_data,
        虽然在主线程崩溃的情况下, 子线程会因daemon自动退出, 但这一操作的延时是不可预知的

        本操作不会强行锁定控制模式于所需的定点模式, 因此可以通过切换到程控来暂停实时控制
        """
        if self._realtime_control_running:
            self.stop_realtime_control()
        self._realtime_control_running = True
        self._realtime_control_thread = threading.Thread(target=self._realtime_control_task, args=(freq,), daemon=True)
        self._thread_list.append(self._realtime_control_thread)
        self._realtime_control_thread.start()

    def stop_realtime_control(self) -> None:
        """
        停止自动发送实时控制
        """
        self._realtime_control_running = False
        if self._realtime_control_thread:
            self._realtime_control_thread.join()
            self._thread_list.remove(self._realtime_control_thread)
            self._realtime_control_thread = None  # type: ignore
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]

    @property
    def realtime_control_status(self) -> bool:
        """
        自动发送实时控制是否正在运行
        """
        return self._realtime_control_running

    def update_realtime_control(
        self,
        vel_x: Optional[int] = None,
        vel_y: Optional[int] = None,
        vel_z: Optional[int] = None,
        yaw: Optional[int] = None,
    ) -> None:
        """
        更新自动发送实时控制的目标值
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正

        注意默认参数为None, 代表不更新对应的值, 若不需要建议置为0而不是留空
        """
        if vel_x is not None:
            self._realtime_control_data_in_xyzYaw[0] = vel_x
        if vel_y is not None:
            self._realtime_control_data_in_xyzYaw[1] = vel_y
        if vel_z is not None:
            self._realtime_control_data_in_xyzYaw[2] = vel_z
        if yaw is not None:
            self._realtime_control_data_in_xyzYaw[3] = yaw

    @property
    def ground_station(self):
        """已启动的机载地面站门面，未启动时为 None。"""
        return self._ground_station

    @property
    def ground_station_connected(self) -> bool:
        return bool(
            self._ground_station is not None
            and self._ground_station.connected
        )

    @property
    def ground_stop_requested(self) -> bool:
        return bool(
            self._ground_station is not None
            and self._ground_station.stop_requested
        )

    def start_ground_station(
        self,
        key: Optional[bytes] = None,
        stop_event: Optional[threading.Event] = None,
        **link_options
    ):
        """通过飞控无线串口桥启动机载 HC-14 地面站链路。

        key 为空时从 GROUND_STATION_HMAC_KEY_HEX 环境变量读取。
        返回的门面只收发状态、任务命令和 ACK，不直接执行飞行动作。
        """
        if self._ground_station is not None:
            return self._ground_station

        service_factory = link_options.pop("_service_factory", None)
        if service_factory is None:
            from FlightController.Components.GroundStationLink import (
                AircraftGroundStation,
            )

            service_factory = AircraftGroundStation

        if key is None:
            station = service_factory.from_environment(
                fc=self,
                stop_event=stop_event,
                **link_options
            )
        else:
            station = service_factory(
                fc=self,
                key=key,
                stop_event=stop_event,
                **link_options
            )
        station.start()
        self._ground_station = station
        return station

    def stop_ground_station(self) -> None:
        station = self._ground_station
        self._ground_station = None
        if station is not None:
            station.close()

    def _require_ground_station(self):
        if self._ground_station is None:
            raise RuntimeError("ground station is not started")
        return self._ground_station

    def receive_ground_command(self, timeout: Optional[float] = None):
        return self._require_ground_station().receive_command(timeout)

    def ground_command_done(self) -> None:
        self._require_ground_station().command_done()

    @property
    def ground_link_mode(self):
        return self._require_ground_station().mode

    def set_ground_link_mode(self, mode) -> None:
        self._require_ground_station().set_mode(mode)

    def enable_ground_command_reception(self) -> None:
        """Use before takeoff: accept ground commands and return their ACKs."""
        self._require_ground_station().enable_command_reception()

    def enable_ground_telemetry(self) -> None:
        """Use after takeoff: transmit state only; incoming commands are ignored."""
        self._require_ground_station().enable_telemetry_transmission()

    def accept_ground_command(self, command) -> None:
        self._require_ground_station().accept(command)

    def reject_ground_command(self, command, reason) -> None:
        self._require_ground_station().reject(command, reason)

    def complete_ground_command(self, command) -> None:
        self._require_ground_station().complete(command)

    def fail_ground_command(self, command, reason) -> None:
        self._require_ground_station().fail(command, reason)

    def prepare_ground_mission(self) -> None:
        self._require_ground_station().prepare_new_mission()

    def send_ground_status(
        self,
        state,
        target1: Optional[int] = None,
        target2: Optional[int] = None,
        progress: int = 0,
        error_code: int = 0,
        message: str = "",
    ) -> bool:
        return self._require_ground_station().send_status(
            state=state,
            target1=target1,
            target2=target2,
            progress=progress,
            error_code=error_code,
            message=message,
        )

    def send_ground_alarm(self, code: int, message: str) -> bool:
        return self._require_ground_station().send_alarm(code, message)

    def send_ground_led(self, control) -> bool:
        """Set the ground-station GPIO18 indicator while telemetry is enabled."""
        return self._require_ground_station().send_led_control(control)

    def set_ground_led_pixels(self, pixels, brightness: int = 3) -> bool:
        from FlightController.Components.GroundStationLink import LEDControl, LEDMode

        return self.send_ground_led(
            LEDControl(LEDMode.PIXELS, brightness=brightness, pixels=tuple(pixels))
        )

    def reset_ground_led_flow(self, brightness: int = 3) -> bool:
        from FlightController.Components.GroundStationLink import LEDControl, LEDMode

        return self.send_ground_led(LEDControl(LEDMode.FLOW, brightness=brightness))

    def send_ground_state(self) -> bool:
        return self._require_ground_station().send_state()

    def close(self, joined=True) -> None:
        self.stop_ground_station()
        return super().close(joined)

    def set_action_log(self, output: bool) -> None:
        """
        设置动作日志输出
        """
        self.settings.action_log_output = output

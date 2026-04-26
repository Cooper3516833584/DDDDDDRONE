import struct
import time
from typing import Optional

from FlightController.Base import Byte_Var, FC_Base_Uart_Comunication
from loguru import logger


class FC_Protocol(FC_Base_Uart_Comunication):
    """
    协议层, 定义了实际的控制命令
    """

    # constants
    HOLD_ALT_MODE = 1
    HOLD_POS_MODE = 2
    PROGRAM_MODE = 3

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs) # type: ignore
        self._byte_temp1 = Byte_Var()
        self._byte_temp2 = Byte_Var()
        self._byte_temp3 = Byte_Var()
        self._byte_temp4 = Byte_Var()
        self.last_sended_command = (0, 0, 0)  # (CID,CMD_0,CMD_1)

    def _action_log(self, action: str, data_info: Optional[str] = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    ######### 飞控命令 #########

    def set_rgb_led(self, r: int, g: int, b: int) -> None:
        """
        设置由32控制的RGB LED
        r,g,b: 0-255
        """
        self._byte_temp1.reset(r, "u8", int)
        self._byte_temp2.reset(g, "u8", int)
        self._byte_temp3.reset(b, "u8", int)
        self.send_data_to_fc(self._byte_temp1.bytes + self._byte_temp2.bytes + self._byte_temp3.bytes, 0x02, True)

    def set_indicator_led(self, r: int, g: int, b: int) -> None:
        """
        设置32板载LED
        r,g,b: 0-255
        """
        self._byte_temp1.reset(round(r / 255 * 20), "s8", int)
        self._byte_temp2.reset(round(g / 255 * 20), "s8", int)
        self._byte_temp3.reset(round(b / 255 * 20), "s8", int)
        self.send_data_to_fc(self._byte_temp1.bytes + self._byte_temp2.bytes + self._byte_temp3.bytes, 0x0A, True)

    def send_general_position(self, x: int = -2_147_483_648, y: int = -2_147_483_648, z: int = -2_147_483_648) -> None:
        """
        通用位置传感器回传
        x,y,z: cm
        默认值 -2_147_483_648 代表输入无效
        """
        x = x if x is not None else -2_147_483_648
        y = y if y is not None else -2_147_483_648
        z = z if z is not None else -2_147_483_648
        self.send_data_to_fc(struct.pack("<iii", x, y, z), 0x03, False)

    def send_general_speed(self, x: int = -32768, y: int = -32768, z: int = -32768) -> None:
        """
        通用速度传感器回传
        x,y,z: cm / s
        默认值 -32768 代表输入无效
        """
        x = x if x is not None else -32768
        y = y if y is not None else -32768
        z = z if z is not None else -32768
        self.send_data_to_fc(struct.pack("<hhh", x, y, z), 0x08, False)

    def send_general_height(self, height: int = -2_147_483_648) -> None:
        """
        通用高度传感器回传
        height: cm
        默认值 -2_147_483_648 代表输入无效
        """
        if height is None or height == -2_147_483_648:
            return  # 只有一个数据, 无效数据不发送
        self.send_data_to_fc(struct.pack("<i", height), 0x0B, False)

    def send_gps_data(
        self,
        lng: float, lat: float, alt: float,
        n_spd: int=0, e_spd: int=0, d_spd: int=0,
        fix_sta: int=3, sat_num: int=13,
        pdop: int=2, s_acc: int=2, v_acc: int=2,
    ):  # fmt: skip
        """
        GPS数据回传
        fix_sta: 定位状态 (0:无定位 1:2D定位 2:3D定位)
        sat_num: 卫星数量
        lng: 经度
        lat: 纬度
        alt: 高度 (cm)
        n_spd: 北向速度 (cm/s)
        e_spd: 东向速度 (cm/s)
        d_spd: 垂直速度 (cm/s)
        pdop: 定位精度 (0-200cm)
        s_acc: 速度精度 (0-200cm/s)
        v_acc: 垂直精度 (0-200cm)
        """
        self.send_data_to_fc(
            struct.pack(
                "<BBiiihhhBBB",
                fix_sta, sat_num,
                round(lng * 1000_0000),
                round(lat * 1000_0000),
                round(alt),
                n_spd, e_spd, d_spd,
                pdop, s_acc, v_acc,
            ), 0x09, False
        )  # fmt: skip

    def send_to_uart_screen(self, data: bytes) -> None:
        """
        转发数据到串口屏
        """
        self.send_data_to_fc(data, 0x0C, True)

    def send_to_wireless(self, data: bytes) -> None:
        """
        转发数据到无线串口
        """
        self.send_data_to_fc(data, 0x0D, True)

    def send_realtime_control_data(self, vel_x: int = 0, vel_y: int = 0, vel_z: int = 0, yaw: int = 0) -> None:
        """
        发送实时控制帧, 仅在定点模式下有效(MODE=2), 切换模式前需要确保遥控器摇杆全部归中
        在飞控内有实时控制帧的安全检查, 每个帧有效时间只有1s, 因此发送频率需要大于1Hz
        注意记得切换模式!!!
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正
        """
        # 性能优化:因为实时控制帧需要频繁低延迟发送, 所以不做过多次变量初始化
        self.send_data_to_fc(struct.pack("<hhhh", round(vel_x), round(vel_y), round(vel_z), round(-yaw)), 0x04, False)

    def set_PWM_output(self, channel: int, pwm: float) -> None:
        """
        设置PWM输出
        channel: 0-3
        pwm: 0.00-100.00
        """
        assert channel in [0, 1, 2, 3]
        pwm_int = int(pwm * 100)
        pwm_int = max(0, min(10000, pwm_int))
        self._byte_temp1.reset(channel, "u8", int)
        self._byte_temp2.reset(pwm_int, "s16", int)
        self.send_data_to_fc(self._byte_temp1.bytes + self._byte_temp2.bytes, 0x05, True)
        self._action_log("set pwm output", f"channel {channel} pwm {pwm:.2f}")

    def set_digital_output(self, channel: int, on: bool) -> None:
        """
        设置数字输出
        channel: 0-3
        on: 开关
        """
        assert channel in [0, 1, 2, 3]
        on_ = 1 if on else 0
        self._byte_temp1.reset(channel, "u8", int)
        self._byte_temp2.reset(on_, "u8", int)
        self.send_data_to_fc(self._byte_temp1.bytes + self._byte_temp2.bytes, 0x06, True)
        self._action_log("set digital output", f"channel {channel} {on}")
    
    def _set_digital_output_fast(self, channel: int, on: bool) -> None:
        """快速设置数字输出，不等待ACK、不加锁，用于步进电机高速模式"""
        on_ = 1 if on else 0
        self._byte_temp1.reset(channel, "u8", int)
        self._byte_temp2.reset(on_, "u8", int)
        self.send_data_to_fc(
            self._byte_temp1.bytes + self._byte_temp2.bytes, 0x06,
            need_ack=False, no_lock=True
        )

    def step_motor_rotate(self, revolutions: float, step_delay_ms: float = 2, steps_per_rev: int = 2048, fast: bool = False) -> None:
        """
        控制ULN2003驱动板驱动28BYJ-48步进电机转动指定圈数
        
        驱动板上的in1,in2,in3,in4分别与飞控代码中的channel0,1,2,3相连
        
        Args:
            revolutions: 转动圈数，正数为顺时针，负数为逆时针
            step_delay_ms: 每一步之间的延迟（毫秒），影响电机转速
            steps_per_rev: 每圈步数（28BYJ-48默认2048步/圈，半步模式；全步模式1024）
            fast: 快速模式，使用全步驱动+无ACK发送，速度提升约6-8倍
        
        工作原理：
        1. 默认使用8步半步序列，fast模式使用4步全步序列
        2. 每步依次设置channel0-3（对应IN1-IN4）的高低电平
        3. fast模式下跳过ACK等待和发送锁，大幅减少通信开销
        4. 转动结束后关闭所有输出以释放电机
        
        注意：
        - fast模式下不等待飞控应答，极小概率丢步
        - 实际转速受通信延迟影响
        - 电机扭矩较小，不建议在高负载下使用
        """
        if fast:
            # 4步全步序列（步数减半，速度翻倍，扭矩略降）
            step_sequence_cw = [
                [1, 0, 0, 1],  # IN1+IN4
                [1, 1, 0, 0],  # IN1+IN2
                [0, 1, 1, 0],  # IN2+IN3
                [0, 0, 1, 1],  # IN3+IN4
            ]
            if steps_per_rev == 2048:
                steps_per_rev = 1024  # 全步模式每圈1024步
            set_output = self._set_digital_output_fast
        else:
            # 8步半步序列（默认，运行更平稳）
            step_sequence_cw = [
                [1, 0, 0, 0],  # 步骤1: IN1高
                [1, 1, 0, 0],  # 步骤2: IN1+IN2高
                [0, 1, 0, 0],  # 步骤3: IN2高
                [0, 1, 1, 0],  # 步骤4: IN2+IN3高
                [0, 0, 1, 0],  # 步骤5: IN3高
                [0, 0, 1, 1],  # 步骤6: IN3+IN4高
                [0, 0, 0, 1],  # 步骤7: IN4高
                [1, 0, 0, 1],  # 步骤8: IN4+IN1高
            ]
            set_output = self.set_digital_output
        
        # 逆时针序列就是顺时针序列的反向
        step_sequence_ccw = list(reversed(step_sequence_cw))
        
        # 选择方向和序列
        if revolutions >= 0:
            sequence = step_sequence_cw
            direction = "clockwise"
        else:
            sequence = step_sequence_ccw
            direction = "counterclockwise"
            revolutions = -revolutions  # 取绝对值计算步数
        
        # 计算总步数
        total_steps = int(revolutions * steps_per_rev)
        
        if total_steps == 0:
            logger.debug("[FC] No steps to rotate, returning")
            return
        
        mode_str = "FAST" if fast else "NORMAL"
        logger.info(f"[FC] Starting stepper motor [{mode_str}]: {revolutions} revs {direction}, "
                   f"{total_steps} steps, delay {step_delay_ms}ms")
        
        try:
            # 循环执行步进
            for step_count in range(total_steps):
                # 获取当前步的引脚状态
                step_index = step_count % len(sequence)
                pin_states = sequence[step_index]
                
                # 设置四个引脚的状态（channel0-3对应IN1-IN4）
                for channel, state in enumerate(pin_states):
                    set_output(channel, bool(state))
                
                # 步间延迟（转换为秒）
                if step_delay_ms > 0:
                    time.sleep(step_delay_ms / 1000.0)
                
                # 每1000步输出一次进度
                if step_count % 1000 == 0 and step_count > 0:
                    progress = (step_count / total_steps) * 100
                    logger.debug(f"[FC] Stepper progress: {progress:.1f}%")
            
            # 转动完成后关闭所有输出，释放电机
            for channel in range(4):
                set_output(channel, False)
            
            self._action_log("step motor rotate", 
                           f"{revolutions} revs {direction}, {total_steps} steps [{mode_str}]")
            logger.info(f"[FC] Stepper motor rotation completed")
            
        except Exception as e:
            # 发生异常时立即关闭所有输出
            for channel in range(4):
                try:
                    self.set_digital_output(channel, False)
                except:
                    pass
            logger.error(f"[FC] Stepper motor error: {e}")
            raise

    def set_pod(self, state: int, time: int) -> None:
        """
        设置吊舱
        state: 1 放线 2 收线
        time: 动作时间 毫秒 收线触发限位开关时或到达时间会停止, 放线仅超时停止
        """
        self._byte_temp1.reset(state, "u8", int)
        self._byte_temp2.reset(time, "u32", int)
        self.send_data_to_fc(self._byte_temp1.bytes + self._byte_temp2.bytes, 0x07, True)
        self._action_log("set pod", f"state {state} time {time}")

    ######### IMU 命令 #########

    def _send_imu_command_frame(self, CID: int, CMD0: int, CMD1: int, CMD_data=b""):
        self._byte_temp1.reset(CID, "u8", int)
        self._byte_temp2.reset(CMD0, "u8", int)
        self._byte_temp3.reset(CMD1, "u8", int)
        bytes_data = bytes(CMD_data)
        if len(bytes_data) < 8:
            bytes_data += b"\x00" * (8 - len(bytes_data))
        assert len(bytes_data) == 8, "CMD_data length is too long"
        data_to_send = self._byte_temp1.bytes + self._byte_temp2.bytes + self._byte_temp3.bytes + bytes_data
        self.send_data_to_fc(data_to_send, 0x01, True)
        self.last_sended_command = (CID, CMD0, CMD1)

    def _check_mode(self, target_mode) -> bool:
        """
        检查当前模式是否与需要的模式一致
        """
        mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        if self.state.mode.value != target_mode:
            if self.settings.auto_change_mode:
                self._action_log("auto mode set", mode_dict[target_mode])
                self.set_flight_mode(target_mode)
                time.sleep(0.1)  # 等待模式改变完成
                return True
            else:
                logger.error(
                    f"[FC] Mode error: action required mode is {mode_dict[target_mode]}"
                    f", but current mode is {mode_dict[self.state.mode.value]}"
                )
                return False
        return True

    def set_flight_mode(self, mode: int) -> None:
        """
        设置飞行模式: (随时有效)
        0: 姿态自稳 (危险,禁用)
        1: 定高
        2: 定点
        3: 程控
        """
        if mode not in [1, 2, 3]:
            raise ValueError("mode must be 1,2,3")
        self._byte_temp1.reset(mode, "u8", int)
        self._send_imu_command_frame(0x01, 0x01, 0x01, self._byte_temp1.bytes)
        self._action_log("set flight mode", f"{mode}")

    def unlock(self) -> None:
        """
        解锁电机 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x01)
        self._action_log("unlock")

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x02)
        self._action_log("lock")

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x04)
        self._action_log("stablize")

    def take_off(self, target_height: int = 0) -> None:
        """
        一键起飞 (除姿态模式外, 随时有效)
        目标高度: 0-500 cm, 0为默认高度
        """
        self._byte_temp1.reset(target_height, "u16", int)
        self._send_imu_command_frame(0x10, 0x00, 0x05, self._byte_temp1.bytes)
        self._action_log("take off")

    def land(self) -> None:
        """
        一键降落 (除姿态模式外, 随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x06)
        self._action_log("land")

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动: (程控模式下有效)
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._byte_temp3.reset(direction, "u16", int)
        self._send_imu_command_frame(
            0x10,
            0x02,
            0x03,
            self._byte_temp1.bytes + self._byte_temp2.bytes + self._byte_temp3.bytes,
        )
        self._action_log("horizontal move", f"{distance}cm, {speed}cm/s, {direction}deg")

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升: (程控模式下有效)
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(0x10, 0x02, 0x01, self._byte_temp1.bytes + self._byte_temp2.bytes)
        self._action_log("go up", f"{distance}cm, {speed}cm/s")

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降: (程控模式下有效)
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(0x10, 0x02, 0x02, self._byte_temp1.bytes + self._byte_temp2.bytes)
        self._action_log("go down", f"{distance}cm, {speed}cm/s")

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转: (程控模式下有效)
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(deg, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(0x10, 0x02, 0x07, self._byte_temp1.bytes + self._byte_temp2.bytes)
        self._action_log("turn left", f"{deg}deg, {speed}deg/s")

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转: (程控模式下有效)
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(deg, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(0x10, 0x02, 0x08, self._byte_temp1.bytes + self._byte_temp2.bytes)
        self._action_log("turn right", f"{deg}deg, {speed}deg/s")

    def set_target_position(self, x: int, y: int) -> None:
        """
        设置目标位置: (程控模式下有效)
        x:+-100000 cm
        y:+-100000 cm
        """
        self._check_mode(3)
        self._byte_temp1.reset(x, "s32", int)
        self._byte_temp2.reset(y, "s32", int)
        self._send_imu_command_frame(0x10, 0x01, 0x01, self._byte_temp1.bytes + self._byte_temp2.bytes)
        self._action_log("set target position", f"{x}, {y}")

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度: (程控模式下有效)
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self._check_mode(3)
        self._byte_temp1.reset(height, "s32", int)
        self._send_imu_command_frame(0x10, 0x01, 0x02, self._byte_temp1.bytes)
        self._action_log("set target height", f"{height}cm")

    @property
    def last_command_done(self) -> bool:
        """
        最后一次指令是否完成
        """
        return self.last_sended_command != self.state.command_now

    @property
    def hovering(self) -> bool:
        """
        是否正在悬停
        """
        stable_command = (0x10, 0x00, 0x04)
        return self.state.command_now == stable_command

# pip-generator: skip file
# mypy: ignore-errors
import sys
import time

import cv2
import numpy as np
from fc_gui.ui_gui import Ui_MainWindow
from FlightController import FC_Client, FC_Controller
from FlightController.Base import FC_State_Struct
from FlightController.Components.LDRadar_Driver import LD_Radar
from loguru import logger

""" qtdesigner file """

import qdarktheme
from PySide6 import QtSerialPort
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtUiTools import *
from PySide6.QtWidgets import *

logger.remove()


def set_button_font_color(btn: QPushButton, color: str):
    btn.setStyleSheet(f"color: {color};")


class MainWindow(Ui_MainWindow, QMainWindow):
    def __init__(self) -> None:
        super(MainWindow, self).__init__()
        self.log_list = []
        self.connected = False
        self.setupUi(self)
        self.init_misc()
        self.fc = None
        self.server_ip = "127.0.0.1:5654"
        self.updating_serial = False
        self.speed_xyzYaw = [0, 0, 0, 0]
        self.btn_serial_update.click()
        self.radar = LD_Radar()
        self.radar_map_gen = self.radar._radar_map_generator()
        self.label_radar_image.setText("")
        self.label_radar_image.setMouseTracking(True)
        self.jotstick1.moveSignal.connect(self.on_xy_move)
        self.jotstick2.moveSignal.connect(self.on_zYaw_move)
        self.box_hori_param.setEnabled(False)
        self.box_vert_param.setEnabled(False)
        self.box_spin_param.setEnabled(False)
        set_button_font_color(self.btn_unlock, "#bb3f38")
        set_button_font_color(self.btn_lock, "#7eb778")
        logger.add(self.get_fake_writer(), format="{message}", level="INFO")
        self.line_info.setText("未连接")
        self.tabWidget.setTabText(2, "激光雷达(离线)")
        self.label_radar_image.setPixmap(QPixmap())
        self.label_radar_image.setText("[已离线]")

    def init_misc(self) -> None:
        self.text_log.clear()
        self.line_info.clear()
        self.fc_timer = QTimer()
        self.fc_timer.timeout.connect(self.fc_timer_update)
        self.radar_timer = QTimer()
        self.radar_timer.timeout.connect(self.radar_timer_update)
        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.log_timer_update)
        self.log_timer.start(10)

    @Slot()
    def on_check_enable_ctl_stateChanged(self):
        state = self.check_enable_ctl.isChecked()
        if state:
            self.speed_xyzYaw = [0, 0, 0, 0]
            self.fc_timer.start(20)
            self.box_hori_param.setEnabled(True)
            self.box_vert_param.setEnabled(True)
            self.box_spin_param.setEnabled(True)
            self.print_log("上位机控制帧激活")
            if self.fc is not None:
                self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
                self.print_log("自动切换飞控到定点模式")
        else:
            self.box_hori_param.setEnabled(False)
            self.box_vert_param.setEnabled(False)
            self.box_spin_param.setEnabled(False)
            self.print_log("控制帧停止发送")
            self.speed_xyzYaw = [0, 0, 0, 0]
            QTimer.singleShot(100, lambda: self.fc_timer.stop())

    def on_xy_move(self, deg, distance):
        x_ratio = np.sin(deg / 180 * np.pi)
        y_ratio = np.cos(deg / 180 * np.pi)
        x_speed = round(self.box_hori_param.value() * x_ratio * distance)
        y_speed = round(-self.box_hori_param.value() * y_ratio * distance)
        self.speed_xyzYaw[0] = x_speed
        self.speed_xyzYaw[1] = y_speed
        if x_speed == 0 and y_speed == 0:
            self.label_10.setText("俯仰 / 滚转 (方向键)")
        else:
            self.label_10.setText(f"X={x_speed} Y={y_speed}")

    def on_zYaw_move(self, deg, distance):
        z_ratio = np.sin(deg / 180 * np.pi)
        yaw_ratio = np.cos(deg / 180 * np.pi)
        z_speed = round(self.box_vert_param.value() * z_ratio * distance)
        yaw_speed = round(self.box_spin_param.value() * yaw_ratio * distance)
        self.speed_xyzYaw[2] = z_speed
        self.speed_xyzYaw[3] = yaw_speed
        if z_speed == 0 and yaw_speed == 0:
            self.label_11.setText("高度 / 偏航 (WASD)")
        else:
            self.label_11.setText(f"Z={z_speed} Yaw={yaw_speed}")

    @Slot(int)
    def on_tabWidget_currentChanged(self, index):
        if index == 2 and self.fc is not None and not self.radar.running:
            self.radar.start(self.fc)
        if self.radar.running:
            if index == 2:
                self.radar_timer.start(round(1000 / 30))
                self.tabWidget.setTabText(2, "激光雷达(运行)")
                self.label_radar_image.setText("")
            elif self.radar_timer.isActive():
                self.radar_timer.stop()
                self.tabWidget.setTabText(2, "激光雷达(空闲)")
                self.label_radar_image.setPixmap(QPixmap())
                self.label_radar_image.setText("[已离线]")

    def get_fake_writer(self):
        class FakeWriter:
            def __init__(self, obj: MainWindow):
                self.obj = obj

            def write(self, msg):
                msg = msg.replace("\n", " ")
                time_str = QDateTime.currentDateTime().toString("[hh:mm:ss]")
                text = f"{time_str} {msg}"
                self.obj.log_list.append(text)

            def flush(self):
                pass

        return FakeWriter(self)

    def log_timer_update(self) -> None:
        while len(self.log_list) > 0:
            self.text_log.appendPlainText(self.log_list.pop(0))

    def radar_timer_update(self) -> None:
        if self.fc is None:
            return
        try:
            cv_img = next(self.radar_map_gen).copy()
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self._qimage = QImage(cv_img.data, cv_img.shape[1], cv_img.shape[0], QImage.Format_RGB888)
            self._qpixmap = QPixmap.fromImage(self._qimage)
            self.label_radar_image.setPixmap(self._qpixmap)
        except Exception as e:
            logger.exception(e)

    # 滚轮事件
    def wheelEvent(self, event: QWheelEvent) -> None:
        if self.radar.running:
            if event.angleDelta().y() > 0:
                self.radar._radar_map_img_scale *= 1.1
            else:
                self.radar._radar_map_img_scale *= 0.9
        return super().wheelEvent(event)

    # 鼠标移动事件
    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self.radar.running:
            pos = self.label_radar_image.mapFromGlobal(event.globalPos())
            dy = self.label_radar_image.height() / 2 - pos.y()
            dx = pos.x() - self.label_radar_image.width() / 2
            self.radar._radar_map_info_angle = (90 - np.arctan2(dy, dx) * 180 / np.pi) % 360
        return super().mouseMoveEvent(event)

    def print_log(self, msg: str) -> None:
        msg = msg.replace("\n", " ")
        time_str = QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.log_list.append(f"{time_str}[GUI] {msg}")

    @Slot()
    def on_btn_serial_update_clicked(self) -> None:
        if not self.connected:
            self.updating_serial = True
            self.combo_serial.clear()
            self.combo_serial.addItem("选择连接方式")
            self.combo_serial.addItem("远端TCP服务")
            self.combo_serial.setCurrentIndex(0)
            ports = QtSerialPort.QSerialPortInfo.availablePorts()
            add_ports = []
            for port in ports:
                add_ports.append(port.portName())
            try:
                add_ports.sort(key=lambda s: int("".join([i for i in s if i.isdigit()])))
            except:
                pass
            self.combo_serial.addItems(add_ports)
            self.print_log(f"找到{len(ports)}个串口")
            self.updating_serial = False
        else:
            self.disconnect()

    def after_connect(self) -> None:
        self.connected = True
        self.btn_serial_update.setText("断开")
        self.combo_serial.setEnabled(False)
        self.line_info.setText("已连接, 等待飞控数据...")
        self.tabWidget.setTabText(2, "激光雷达(空闲)")

    def after_disconnect(self) -> None:
        self.connected = False
        self.btn_serial_update.setText("刷新")
        self.combo_serial.setEnabled(True)
        self.updating_serial = True
        self.combo_serial.setCurrentIndex(0)
        self.updating_serial = False
        self.line_info.setText("未连接")
        self.tabWidget.setTabText(2, "激光雷达(离线)")

    def disconnect(self):
        self.print_log("正在断开连接...")
        if self.radar is not None:
            self.radar_timer.stop()
            self.radar.stop()
        self.fc.close()
        self.print_log("断开连接成功")
        self.fc = None
        self.line_info.setText("")
        self.after_disconnect()

    @Slot()
    def on_combo_serial_currentIndexChanged(self) -> None:
        if self.updating_serial:
            return
        text = self.combo_serial.currentText()
        try:
            if text == "远端TCP服务":
                try:
                    old_port = self.server_ip.split(":")[1]
                    self.server_ip, ok = QInputDialog.getText(
                        self, "远端TCP服务", "请输入服务器IP地址/端口", QLineEdit.Normal, self.server_ip
                    )
                    if not ok:
                        return
                    if len(self.server_ip.split(":")) != 2:
                        self.server_ip += f":{old_port}"
                    self.print_log(f"正在连接到 {self.server_ip} ...")
                    self.fc = FC_Client()
                    self.fc.connect(
                        self.server_ip.split(":")[0],
                        int(self.server_ip.split(":")[1]),
                        callback=self.update_fc_state,
                        print_state=False,
                        timeout=4,
                    )
                    self.print_log("连接成功")
                    self.after_connect()
                except Exception as e:
                    self.print_log(f"连接失败, {e}")
            elif text != "选择连接方式":
                self.print_log("正在连接...")
                self.fc = FC_Controller()
                self.fc.start_listen_serial(text, 500000, callback=self.update_fc_state, print_state=False)
                self.print_log("串口连接成功")
                self.after_connect()
        except Exception as e:
            self.print_log(f"操作失败, {e}")

    def fc_timer_update(self) -> None:
        if self.fc is None:
            return
        if not self.fc.state.unlock.value:
            return
        if self.check_enable_ctl.isChecked():
            if self.fc.state.mode.value != self.fc.PROGRAM_MODE:
                self.fc.send_realtime_control_data(*self.speed_xyzYaw)

    def update_fc_state(self, state: FC_State_Struct) -> None:
        self.lcd_h.display(f"{state.alt_add.value/100:9.02f}")
        self.lcd_d.display(f"{state.yaw.value:9.02f}")
        self.lcd_v.display(f"{state.bat.value:9.02f}")
        self.lcd_x.display(f"{state.vel_x.value/100:9.02f}")
        self.lcd_y.display(f"{state.vel_y.value/100:9.02f}")
        self.lcd_z.display(f"{state.vel_z.value/100:9.02f}")
        state_text = ""
        if state.unlock.value:
            state_text += "电机已解锁  "
        else:
            state_text += "电机已锁定  "
        mode_dict = {1: "定高", 2: "定点", 3: "程控"}
        state_text += f"模式: {mode_dict[state.mode.value]}  "
        state_text += f"指令状态: x{state.cid.value:02X} / x{state.cmd_0.value:02X} / x{state.cmd_1.value:02X} /"
        if self.fc.last_command_done:
            state_text += " 已完成  "
        else:
            state_text += " 进行中  "
        if self.fc.hovering:
            state_text += "机体已稳定"
        elif not state.unlock.value:
            state_text += "等待起飞"
        else:
            state_text += "机体运动中"
        self.line_info.setText(state_text)

    @Slot()
    def on_btn_takeoff_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.take_off()

    @Slot()
    def on_btn_land_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.land()

    @Slot()
    def on_btn_unlock_clicked(self) -> None:
        if self.fc is None:
            return
        if self.fc.state.mode.value == self.fc.HOLD_ALT_MODE:
            QMessageBox.critical(self, "安全限制", "当前处于姿态定高模式,禁止使用上位机解锁,\n请切换模式或使用遥控器操控", QMessageBox.Ok)
            return
        ret = QMessageBox.warning(self, "危险警告", "确认执行解锁操作?\n(确保桨叶净空)", QMessageBox.Yes | QMessageBox.No)
        if ret == QMessageBox.No:
            return
        self.fc.unlock()

    @Slot()
    def on_btn_lock_clicked(self) -> None:
        if self.fc is None:
            return
        self.fc.lock()

    @Slot()
    def on_buttonGroup_buttonClicked(self) -> None:
        if self.fc is None:
            return
        if self.radio_control_realtime.isChecked():
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            self.print_log("飞控切换到定点模式")
        else:
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            self.print_log("飞控切换到程控模式")

    @Slot()
    def on_btn_rgb_clicked(self) -> None:
        if self.fc is None:
            return
        s, ok = QInputDialog.getText(self, "设置WS2812 RGB", "RGB(hex):", QLineEdit.Normal, "#000000")
        if not ok:
            return
        try:
            r, g, b = int(s[1:3], 16), int(s[3:5], 16), int(s[5:7], 16)
            self.fc.set_rgb_led(r, g, b)
        except:
            QMessageBox.warning(self, "命令执行失败", "检查输入格式和飞控连接", QMessageBox.Ok)

    @Slot()
    def on_btn_indicator_clicked(self) -> None:
        if self.fc is None:
            return
        s, ok = QInputDialog.getText(self, "设置板载RGB", "RGB(hex):", QLineEdit.Normal, "#000000")
        if not ok:
            return
        try:
            r, g, b = int(s[1:3], 16), int(s[3:5], 16), int(s[5:7], 16)
            self.fc.set_indicator_led(r, g, b)
        except:
            QMessageBox.warning(self, "命令执行失败", "检查输入格式和飞控连接", QMessageBox.Ok)

    def _set_io(self, io):
        if self.fc is None:
            return
        s, ok = QInputDialog.getItem(self, f"设置IO:{io}", "操作:", ["开", "关"], 0, False)
        if not ok:
            return
        try:
            if s == "开":
                self.fc.set_digital_output(io, True)
            else:
                self.fc.set_digital_output(io, False)
        except:
            QMessageBox.warning(self, "命令执行失败", "检查飞控连接", QMessageBox.Ok)

    def _set_pwm(self, channel):
        if self.fc is None:
            return
        get, ok = QInputDialog.getDouble(self, f"设置PWM:{channel}", "占空比:", 0, 0, 100, 1)
        if not ok:
            return
        try:
            self.fc.set_PWM_output(channel, get)
        except:
            QMessageBox.warning(self, "命令执行失败", "检查飞控连接", QMessageBox.Ok)

    @Slot()
    def on_btn_pod_clicked(self) -> None:
        if self.fc is None:
            return
        s, ok = QInputDialog.getItem(self, "设置吊舱", "操作:", ["上升", "下降"], 0, False)
        if not ok:
            return
        try:
            if s == "上升":
                self.fc.set_pod(2, 20000)
            else:
                get, ok = QInputDialog.getInt(self, "设置吊舱", "放线时间(ms):", 0, 0, 20000, 1)
                if not ok:
                    return
                self.fc.set_pod(1, get)
        except:
            QMessageBox.warning(self, "命令执行失败", "检查飞控连接", QMessageBox.Ok)

    @Slot()
    def on_btn_io_0_clicked(self) -> None:
        self._set_io(0)

    @Slot()
    def on_btn_io_1_clicked(self) -> None:
        self._set_io(1)

    @Slot()
    def on_btn_io_2_clicked(self) -> None:
        self._set_io(2)

    @Slot()
    def on_btn_io_3_clicked(self) -> None:
        self._set_io(3)

    @Slot()
    def on_btn_pwm_0_clicked(self) -> None:
        self._set_pwm(0)

    @Slot()
    def on_btn_pwm_1_clicked(self) -> None:
        self._set_pwm(1)

    @Slot()
    def on_btn_pwm_2_clicked(self) -> None:
        self._set_pwm(2)

    @Slot()
    def on_btn_pwm_3_clicked(self) -> None:
        self._set_pwm(3)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat():
            return super().keyPressEvent(event)
        key = event.key()
        if key == Qt.Key_W:
            self.jotstick2.setFakeMove(90)
        elif key == Qt.Key_S:
            self.jotstick2.setFakeMove(270)
        elif key == Qt.Key_A:
            self.jotstick2.setFakeMove(180)
        elif key == Qt.Key_D:
            self.jotstick2.setFakeMove(0)
        elif key == Qt.Key_Left:
            self.jotstick1.setFakeMove(180)
        elif key == Qt.Key_Right:
            self.jotstick1.setFakeMove(0)
        elif key == Qt.Key_Up:
            self.jotstick1.setFakeMove(90)
        elif key == Qt.Key_Down:
            self.jotstick1.setFakeMove(270)
        elif key == Qt.Key_Z:
            self.on_btn_takeoff_clicked()
        elif key == Qt.Key_X:
            self.on_btn_land_clicked()
        elif key == Qt.Key_C:
            self.on_btn_unlock_clicked()
        elif key == Qt.Key_V:
            self.on_btn_lock_clicked()
        return super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat():
            return super().keyReleaseEvent(event)
        key = event.key()
        if key == Qt.Key_W:
            self.jotstick2.resetFakeMove(90)
        elif key == Qt.Key_S:
            self.jotstick2.resetFakeMove(270)
        elif key == Qt.Key_A:
            self.jotstick2.resetFakeMove(180)
        elif key == Qt.Key_D:
            self.jotstick2.resetFakeMove(0)
        elif key == Qt.Key_Left:
            self.jotstick1.resetFakeMove(180)
        elif key == Qt.Key_Right:
            self.jotstick1.resetFakeMove(0)
        elif key == Qt.Key_Up:
            self.jotstick1.resetFakeMove(90)
        elif key == Qt.Key_Down:
            self.jotstick1.resetFakeMove(270)
        return super().keyReleaseEvent(event)


def main():
    app = QApplication(sys.argv)

    mwin = MainWindow()
    app.setStyleSheet(qdarktheme.load_stylesheet())
    mwin.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()

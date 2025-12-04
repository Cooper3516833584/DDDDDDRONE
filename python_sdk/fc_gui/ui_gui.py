# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'guizKPVwJ.ui'
##
## Created by: Qt User Interface Compiler version 6.4.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox,
    QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout,
    QLCDNumber, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QPushButton, QRadioButton, QSizePolicy,
    QTabWidget, QVBoxLayout, QWidget)

from fc_gui.joystick import Joystick

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(647, 670)
        MainWindow.setMinimumSize(QSize(647, 670))
        MainWindow.setMaximumSize(QSize(647, 670))
        MainWindow.setAnimated(True)
        MainWindow.setDockNestingEnabled(False)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_8 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setMinimumSize(QSize(0, 0))
        self.tabWidget.setMaximumSize(QSize(900, 900))
        self.tabWidget.setFocusPolicy(Qt.NoFocus)
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.verticalLayout_3 = QVBoxLayout(self.tab)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(self.tab)
        self.label.setObjectName(u"label")
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMinimumSize(QSize(80, 0))
        font = QFont()
        font.setPointSize(13)
        font.setBold(True)
        self.label.setFont(font)
        self.label.setFrameShape(QFrame.NoFrame)
        self.label.setMidLineWidth(0)
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.label)

        self.lcd_h = QLCDNumber(self.tab)
        self.lcd_h.setObjectName(u"lcd_h")
        self.lcd_h.setMinimumSize(QSize(0, 40))
        font1 = QFont()
        font1.setPointSize(7)
        self.lcd_h.setFont(font1)
        self.lcd_h.setFrameShape(QFrame.StyledPanel)
        self.lcd_h.setSmallDecimalPoint(True)
        self.lcd_h.setDigitCount(9)
        self.lcd_h.setMode(QLCDNumber.Dec)
        self.lcd_h.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_h.setProperty("value", 0.000000000000000)
        self.lcd_h.setProperty("intValue", 0)

        self.horizontalLayout.addWidget(self.lcd_h)

        self.horizontalLayout.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(self.tab)
        self.label_2.setObjectName(u"label_2")
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setMinimumSize(QSize(80, 0))
        self.label_2.setFont(font)
        self.label_2.setFrameShape(QFrame.NoFrame)
        self.label_2.setMidLineWidth(0)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_2.addWidget(self.label_2)

        self.lcd_d = QLCDNumber(self.tab)
        self.lcd_d.setObjectName(u"lcd_d")
        self.lcd_d.setMinimumSize(QSize(0, 40))
        self.lcd_d.setFont(font1)
        self.lcd_d.setFrameShape(QFrame.StyledPanel)
        self.lcd_d.setSmallDecimalPoint(True)
        self.lcd_d.setDigitCount(9)
        self.lcd_d.setMode(QLCDNumber.Dec)
        self.lcd_d.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_d.setProperty("value", 0.000000000000000)
        self.lcd_d.setProperty("intValue", 0)

        self.horizontalLayout_2.addWidget(self.lcd_d)

        self.horizontalLayout_2.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_3 = QLabel(self.tab)
        self.label_3.setObjectName(u"label_3")
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setMinimumSize(QSize(80, 0))
        self.label_3.setFont(font)
        self.label_3.setFrameShape(QFrame.NoFrame)
        self.label_3.setMidLineWidth(0)
        self.label_3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_3.addWidget(self.label_3)

        self.lcd_v = QLCDNumber(self.tab)
        self.lcd_v.setObjectName(u"lcd_v")
        self.lcd_v.setMinimumSize(QSize(0, 40))
        self.lcd_v.setFont(font1)
        self.lcd_v.setFrameShape(QFrame.StyledPanel)
        self.lcd_v.setSmallDecimalPoint(True)
        self.lcd_v.setDigitCount(9)
        self.lcd_v.setMode(QLCDNumber.Dec)
        self.lcd_v.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_v.setProperty("value", 0.000000000000000)
        self.lcd_v.setProperty("intValue", 0)

        self.horizontalLayout_3.addWidget(self.lcd_v)

        self.horizontalLayout_3.setStretch(1, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_3)


        self.horizontalLayout_7.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_6 = QLabel(self.tab)
        self.label_6.setObjectName(u"label_6")
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setMinimumSize(QSize(80, 0))
        self.label_6.setFont(font)
        self.label_6.setFrameShape(QFrame.NoFrame)
        self.label_6.setMidLineWidth(0)
        self.label_6.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_6.addWidget(self.label_6)

        self.lcd_x = QLCDNumber(self.tab)
        self.lcd_x.setObjectName(u"lcd_x")
        self.lcd_x.setMinimumSize(QSize(0, 40))
        self.lcd_x.setFont(font1)
        self.lcd_x.setFrameShape(QFrame.StyledPanel)
        self.lcd_x.setSmallDecimalPoint(True)
        self.lcd_x.setDigitCount(9)
        self.lcd_x.setMode(QLCDNumber.Dec)
        self.lcd_x.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_x.setProperty("value", 0.000000000000000)
        self.lcd_x.setProperty("intValue", 0)

        self.horizontalLayout_6.addWidget(self.lcd_x)

        self.horizontalLayout_6.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_4 = QLabel(self.tab)
        self.label_4.setObjectName(u"label_4")
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setMinimumSize(QSize(80, 0))
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QFrame.NoFrame)
        self.label_4.setMidLineWidth(0)
        self.label_4.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_4.addWidget(self.label_4)

        self.lcd_y = QLCDNumber(self.tab)
        self.lcd_y.setObjectName(u"lcd_y")
        self.lcd_y.setMinimumSize(QSize(0, 40))
        self.lcd_y.setFont(font1)
        self.lcd_y.setFrameShape(QFrame.StyledPanel)
        self.lcd_y.setSmallDecimalPoint(True)
        self.lcd_y.setDigitCount(9)
        self.lcd_y.setMode(QLCDNumber.Dec)
        self.lcd_y.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_y.setProperty("value", 0.000000000000000)
        self.lcd_y.setProperty("intValue", 0)

        self.horizontalLayout_4.addWidget(self.lcd_y)

        self.horizontalLayout_4.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_5 = QLabel(self.tab)
        self.label_5.setObjectName(u"label_5")
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setMinimumSize(QSize(80, 0))
        self.label_5.setFont(font)
        self.label_5.setFrameShape(QFrame.NoFrame)
        self.label_5.setMidLineWidth(0)
        self.label_5.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_5.addWidget(self.label_5)

        self.lcd_z = QLCDNumber(self.tab)
        self.lcd_z.setObjectName(u"lcd_z")
        self.lcd_z.setMinimumSize(QSize(0, 40))
        self.lcd_z.setFont(font1)
        self.lcd_z.setFrameShape(QFrame.StyledPanel)
        self.lcd_z.setSmallDecimalPoint(True)
        self.lcd_z.setDigitCount(9)
        self.lcd_z.setMode(QLCDNumber.Dec)
        self.lcd_z.setSegmentStyle(QLCDNumber.Flat)
        self.lcd_z.setProperty("value", 0.000000000000000)
        self.lcd_z.setProperty("intValue", 0)

        self.horizontalLayout_5.addWidget(self.lcd_z)

        self.horizontalLayout_5.setStretch(1, 1)

        self.verticalLayout_2.addLayout(self.horizontalLayout_5)


        self.horizontalLayout_7.addLayout(self.verticalLayout_2)


        self.verticalLayout_9.addLayout(self.horizontalLayout_7)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.line_info = QLineEdit(self.tab)
        self.line_info.setObjectName(u"line_info")
        self.line_info.setMinimumSize(QSize(0, 43))
        font2 = QFont()
        font2.setPointSize(11)
        self.line_info.setFont(font2)
        self.line_info.setFocusPolicy(Qt.NoFocus)
        self.line_info.setAlignment(Qt.AlignCenter)
        self.line_info.setDragEnabled(True)
        self.line_info.setReadOnly(True)

        self.horizontalLayout_15.addWidget(self.line_info)

        self.horizontalLayout_15.setStretch(0, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_15)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.text_log = QPlainTextEdit(self.tab)
        self.text_log.setObjectName(u"text_log")
        font3 = QFont()
        font3.setPointSize(9)
        self.text_log.setFont(font3)
        self.text_log.setFocusPolicy(Qt.NoFocus)
        self.text_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.text_log.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.text_log.setUndoRedoEnabled(False)
        self.text_log.setReadOnly(True)

        self.horizontalLayout_16.addWidget(self.text_log)


        self.verticalLayout_9.addLayout(self.horizontalLayout_16)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.label_13 = QLabel(self.tab)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setAlignment(Qt.AlignCenter)

        self.verticalLayout_4.addWidget(self.label_13)

        self.btn_takeoff = QPushButton(self.tab)
        self.btn_takeoff.setObjectName(u"btn_takeoff")
        self.btn_takeoff.setMinimumSize(QSize(0, 45))
        font4 = QFont()
        font4.setPointSize(12)
        font4.setBold(False)
        self.btn_takeoff.setFont(font4)
        self.btn_takeoff.setFocusPolicy(Qt.NoFocus)

        self.verticalLayout_4.addWidget(self.btn_takeoff)

        self.btn_land = QPushButton(self.tab)
        self.btn_land.setObjectName(u"btn_land")
        self.btn_land.setMinimumSize(QSize(0, 45))
        self.btn_land.setFont(font4)
        self.btn_land.setFocusPolicy(Qt.NoFocus)

        self.verticalLayout_4.addWidget(self.btn_land)

        self.btn_unlock = QPushButton(self.tab)
        self.btn_unlock.setObjectName(u"btn_unlock")
        self.btn_unlock.setMinimumSize(QSize(0, 45))
        font5 = QFont()
        font5.setPointSize(13)
        font5.setBold(True)
        font5.setItalic(False)
        font5.setStyleStrategy(QFont.PreferAntialias)
        self.btn_unlock.setFont(font5)
        self.btn_unlock.setFocusPolicy(Qt.NoFocus)

        self.verticalLayout_4.addWidget(self.btn_unlock)

        self.btn_lock = QPushButton(self.tab)
        self.btn_lock.setObjectName(u"btn_lock")
        self.btn_lock.setMinimumSize(QSize(0, 45))
        self.btn_lock.setFont(font5)
        self.btn_lock.setFocusPolicy(Qt.NoFocus)

        self.verticalLayout_4.addWidget(self.btn_lock)

        self.verticalLayout_4.setStretch(1, 1)
        self.verticalLayout_4.setStretch(2, 1)
        self.verticalLayout_4.setStretch(3, 1)
        self.verticalLayout_4.setStretch(4, 1)

        self.horizontalLayout_10.addLayout(self.verticalLayout_4)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(-1, 0, 4, -1)
        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.label_11 = QLabel(self.tab)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setAlignment(Qt.AlignCenter)

        self.verticalLayout_6.addWidget(self.label_11)

        self.jotstick2 = Joystick(self.tab)
        self.jotstick2.setObjectName(u"jotstick2")

        self.verticalLayout_6.addWidget(self.jotstick2)

        self.verticalLayout_6.setStretch(1, 1)

        self.horizontalLayout_8.addLayout(self.verticalLayout_6)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label_10 = QLabel(self.tab)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setAlignment(Qt.AlignCenter)

        self.verticalLayout_5.addWidget(self.label_10)

        self.jotstick1 = Joystick(self.tab)
        self.jotstick1.setObjectName(u"jotstick1")

        self.verticalLayout_5.addWidget(self.jotstick1)

        self.verticalLayout_5.setStretch(1, 1)

        self.horizontalLayout_8.addLayout(self.verticalLayout_5)


        self.horizontalLayout_10.addLayout(self.horizontalLayout_8)

        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")

        self.horizontalLayout_17.addLayout(self.horizontalLayout_14)

        self.combo_serial = QComboBox(self.tab)
        self.combo_serial.addItem("")
        self.combo_serial.setObjectName(u"combo_serial")
        self.combo_serial.setFont(font3)
        self.combo_serial.setFocusPolicy(Qt.NoFocus)

        self.horizontalLayout_17.addWidget(self.combo_serial)

        self.btn_serial_update = QPushButton(self.tab)
        self.btn_serial_update.setObjectName(u"btn_serial_update")
        font6 = QFont()
        font6.setPointSize(10)
        font6.setKerning(True)
        self.btn_serial_update.setFont(font6)
        self.btn_serial_update.setFocusPolicy(Qt.NoFocus)
        self.btn_serial_update.setIconSize(QSize(0, 0))

        self.horizontalLayout_17.addWidget(self.btn_serial_update)

        self.horizontalLayout_17.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_17)

        self.horizontalLayout_20 = QHBoxLayout()
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.radio_control_realtime = QRadioButton(self.tab)
        self.buttonGroup = QButtonGroup(MainWindow)
        self.buttonGroup.setObjectName(u"buttonGroup")
        self.buttonGroup.addButton(self.radio_control_realtime)
        self.radio_control_realtime.setObjectName(u"radio_control_realtime")
        self.radio_control_realtime.setFocusPolicy(Qt.NoFocus)
        self.radio_control_realtime.setChecked(True)

        self.horizontalLayout_20.addWidget(self.radio_control_realtime)

        self.radio_control_flow = QRadioButton(self.tab)
        self.buttonGroup.addButton(self.radio_control_flow)
        self.radio_control_flow.setObjectName(u"radio_control_flow")
        self.radio_control_flow.setFocusPolicy(Qt.NoFocus)

        self.horizontalLayout_20.addWidget(self.radio_control_flow)


        self.verticalLayout_7.addLayout(self.horizontalLayout_20)

        self.check_enable_ctl = QCheckBox(self.tab)
        self.check_enable_ctl.setObjectName(u"check_enable_ctl")
        font7 = QFont()
        font7.setBold(True)
        font7.setItalic(False)
        self.check_enable_ctl.setFont(font7)
        self.check_enable_ctl.setFocusPolicy(Qt.NoFocus)
        self.check_enable_ctl.setLayoutDirection(Qt.LeftToRight)

        self.verticalLayout_7.addWidget(self.check_enable_ctl)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.label_7 = QLabel(self.tab)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setFont(font3)

        self.horizontalLayout_11.addWidget(self.label_7)

        self.box_hori_param = QDoubleSpinBox(self.tab)
        self.box_hori_param.setObjectName(u"box_hori_param")
        self.box_hori_param.setMinimumSize(QSize(20, 0))
        self.box_hori_param.setFont(font3)
        self.box_hori_param.setFocusPolicy(Qt.NoFocus)
        self.box_hori_param.setDecimals(1)
        self.box_hori_param.setMaximum(100.000000000000000)
        self.box_hori_param.setSingleStep(5.000000000000000)
        self.box_hori_param.setValue(50.000000000000000)

        self.horizontalLayout_11.addWidget(self.box_hori_param)

        self.horizontalLayout_11.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_8 = QLabel(self.tab)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setFont(font3)

        self.horizontalLayout_12.addWidget(self.label_8)

        self.box_vert_param = QDoubleSpinBox(self.tab)
        self.box_vert_param.setObjectName(u"box_vert_param")
        self.box_vert_param.setMinimumSize(QSize(20, 0))
        self.box_vert_param.setFont(font3)
        self.box_vert_param.setFocusPolicy(Qt.NoFocus)
        self.box_vert_param.setDecimals(1)
        self.box_vert_param.setMaximum(100.000000000000000)
        self.box_vert_param.setSingleStep(5.000000000000000)
        self.box_vert_param.setValue(50.000000000000000)

        self.horizontalLayout_12.addWidget(self.box_vert_param)

        self.horizontalLayout_12.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_12)

        self.horizontalLayout_21 = QHBoxLayout()
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.label_12 = QLabel(self.tab)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setFont(font3)

        self.horizontalLayout_21.addWidget(self.label_12)

        self.box_spin_param = QDoubleSpinBox(self.tab)
        self.box_spin_param.setObjectName(u"box_spin_param")
        self.box_spin_param.setMinimumSize(QSize(20, 0))
        self.box_spin_param.setFont(font3)
        self.box_spin_param.setFocusPolicy(Qt.NoFocus)
        self.box_spin_param.setDecimals(1)
        self.box_spin_param.setMaximum(100.000000000000000)
        self.box_spin_param.setSingleStep(5.000000000000000)
        self.box_spin_param.setValue(45.000000000000000)

        self.horizontalLayout_21.addWidget(self.box_spin_param)

        self.horizontalLayout_21.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_21)


        self.horizontalLayout_10.addLayout(self.verticalLayout_7)

        self.horizontalLayout_10.setStretch(1, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_10)


        self.verticalLayout_3.addLayout(self.verticalLayout_9)

        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.verticalLayout_12 = QVBoxLayout(self.tab_2)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.gridLayout = QGridLayout()
        self.gridLayout.setSpacing(36)
        self.gridLayout.setObjectName(u"gridLayout")
        self.btn_pwm_3 = QPushButton(self.tab_2)
        self.btn_pwm_3.setObjectName(u"btn_pwm_3")
        self.btn_pwm_3.setMinimumSize(QSize(0, 92))
        font8 = QFont()
        font8.setPointSize(10)
        font8.setBold(False)
        self.btn_pwm_3.setFont(font8)

        self.gridLayout.addWidget(self.btn_pwm_3, 3, 2, 1, 1)

        self.btn_pwm_0 = QPushButton(self.tab_2)
        self.btn_pwm_0.setObjectName(u"btn_pwm_0")
        self.btn_pwm_0.setMinimumSize(QSize(0, 92))
        self.btn_pwm_0.setFont(font8)

        self.gridLayout.addWidget(self.btn_pwm_0, 0, 2, 1, 1)

        self.btn_pwm_1 = QPushButton(self.tab_2)
        self.btn_pwm_1.setObjectName(u"btn_pwm_1")
        self.btn_pwm_1.setMinimumSize(QSize(0, 92))
        self.btn_pwm_1.setFont(font8)

        self.gridLayout.addWidget(self.btn_pwm_1, 1, 2, 1, 1)

        self.btn_io_3 = QPushButton(self.tab_2)
        self.btn_io_3.setObjectName(u"btn_io_3")
        self.btn_io_3.setMinimumSize(QSize(0, 92))
        self.btn_io_3.setFont(font8)

        self.gridLayout.addWidget(self.btn_io_3, 3, 3, 1, 1)

        self.btn_pwm_2 = QPushButton(self.tab_2)
        self.btn_pwm_2.setObjectName(u"btn_pwm_2")
        self.btn_pwm_2.setMinimumSize(QSize(0, 92))
        self.btn_pwm_2.setFont(font8)

        self.gridLayout.addWidget(self.btn_pwm_2, 2, 2, 1, 1)

        self.btn_io_2 = QPushButton(self.tab_2)
        self.btn_io_2.setObjectName(u"btn_io_2")
        self.btn_io_2.setMinimumSize(QSize(0, 92))
        self.btn_io_2.setFont(font8)

        self.gridLayout.addWidget(self.btn_io_2, 2, 3, 1, 1)

        self.btn_pod = QPushButton(self.tab_2)
        self.btn_pod.setObjectName(u"btn_pod")
        self.btn_pod.setMinimumSize(QSize(0, 92))
        self.btn_pod.setFont(font8)

        self.gridLayout.addWidget(self.btn_pod, 1, 1, 1, 1)

        self.btn_indicator = QPushButton(self.tab_2)
        self.btn_indicator.setObjectName(u"btn_indicator")
        self.btn_indicator.setMinimumSize(QSize(0, 92))
        self.btn_indicator.setFont(font8)

        self.gridLayout.addWidget(self.btn_indicator, 2, 1, 1, 1)

        self.btn_io_1 = QPushButton(self.tab_2)
        self.btn_io_1.setObjectName(u"btn_io_1")
        self.btn_io_1.setMinimumSize(QSize(0, 92))
        self.btn_io_1.setFont(font8)

        self.gridLayout.addWidget(self.btn_io_1, 1, 3, 1, 1)

        self.btn_rgb_12 = QPushButton(self.tab_2)
        self.btn_rgb_12.setObjectName(u"btn_rgb_12")
        self.btn_rgb_12.setMinimumSize(QSize(0, 92))
        self.btn_rgb_12.setFont(font8)

        self.gridLayout.addWidget(self.btn_rgb_12, 3, 1, 1, 1)

        self.btn_io_0 = QPushButton(self.tab_2)
        self.btn_io_0.setObjectName(u"btn_io_0")
        self.btn_io_0.setMinimumSize(QSize(0, 92))
        self.btn_io_0.setFont(font8)

        self.gridLayout.addWidget(self.btn_io_0, 0, 3, 1, 1)

        self.btn_rgb = QPushButton(self.tab_2)
        self.btn_rgb.setObjectName(u"btn_rgb")
        self.btn_rgb.setMinimumSize(QSize(0, 92))
        self.btn_rgb.setFont(font8)

        self.gridLayout.addWidget(self.btn_rgb, 0, 1, 1, 1)


        self.verticalLayout_12.addLayout(self.gridLayout)

        self.tabWidget.addTab(self.tab_2, "")
        self.tab_4 = QWidget()
        self.tab_4.setObjectName(u"tab_4")
        self.verticalLayout_11 = QVBoxLayout(self.tab_4)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.verticalLayout_10 = QVBoxLayout()
        self.verticalLayout_10.setSpacing(0)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.label_radar_image = QLabel(self.tab_4)
        self.label_radar_image.setObjectName(u"label_radar_image")
        self.label_radar_image.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.label_radar_image)


        self.verticalLayout_11.addLayout(self.verticalLayout_10)

        self.tabWidget.addTab(self.tab_4, "")

        self.verticalLayout_8.addWidget(self.tabWidget)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"\u4e0a\u4f4d\u673a", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u6fc0\u5149\u9ad8\u5ea6", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u822a\u5411\u89d2", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u7535\u6c60\u7535\u538b", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"X-\u901f\u5ea6", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Y-\u901f\u5ea6", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Z-\u901f\u5ea6", None))
        self.line_info.setText(QCoreApplication.translate("MainWindow", u"\u6d4b\u8bd5\u6587\u672c", None))
        self.line_info.setPlaceholderText("")
        self.text_log.setPlainText(QCoreApplication.translate("MainWindow", u"\u6d4b\u8bd5\u6587\u672c", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u64cd\u4f5c (ZXCV)", None))
        self.btn_takeoff.setText(QCoreApplication.translate("MainWindow", u"\u8d77\u98de", None))
        self.btn_land.setText(QCoreApplication.translate("MainWindow", u"\u7740\u9646", None))
        self.btn_unlock.setText(QCoreApplication.translate("MainWindow", u"\u89e3\u9501", None))
        self.btn_lock.setText(QCoreApplication.translate("MainWindow", u"\u9501\u5b9a", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"\u9ad8\u5ea6 / \u504f\u822a (WASD)", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"\u4fef\u4ef0 / \u6eda\u8f6c (\u65b9\u5411\u952e)", None))
        self.combo_serial.setItemText(0, QCoreApplication.translate("MainWindow", u"\u65ad\u5f00\u8fde\u63a5", None))

        self.btn_serial_update.setText(QCoreApplication.translate("MainWindow", u"\u5237\u65b0", None))
        self.radio_control_realtime.setText(QCoreApplication.translate("MainWindow", u"\u5b9a\u70b9\u6a21\u5f0f", None))
        self.radio_control_flow.setText(QCoreApplication.translate("MainWindow", u"\u7a0b\u63a7\u6a21\u5f0f", None))
        self.check_enable_ctl.setText(QCoreApplication.translate("MainWindow", u"   \u4f7f\u80fd\u4e0a\u4f4d\u673a\u5b9e\u65f6\u63a7\u5236", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u6c34\u5e73\u901f\u5ea6", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u5782\u76f4\u901f\u5ea6", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u65cb\u8f6c\u901f\u5ea6", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u" \u98de\u63a7\u72b6\u6001 ", None))
        self.btn_pwm_3.setText(QCoreApplication.translate("MainWindow", u"PWM 3", None))
#if QT_CONFIG(shortcut)
        self.btn_pwm_3.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_pwm_0.setText(QCoreApplication.translate("MainWindow", u"PWM 0", None))
#if QT_CONFIG(shortcut)
        self.btn_pwm_0.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_pwm_1.setText(QCoreApplication.translate("MainWindow", u"PWM 1", None))
#if QT_CONFIG(shortcut)
        self.btn_pwm_1.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_io_3.setText(QCoreApplication.translate("MainWindow", u"\u6570\u5b57\u8f93\u51fa 3", None))
#if QT_CONFIG(shortcut)
        self.btn_io_3.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_pwm_2.setText(QCoreApplication.translate("MainWindow", u"PWM 2", None))
#if QT_CONFIG(shortcut)
        self.btn_pwm_2.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_io_2.setText(QCoreApplication.translate("MainWindow", u"\u6570\u5b57\u8f93\u51fa 2", None))
#if QT_CONFIG(shortcut)
        self.btn_io_2.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_pod.setText(QCoreApplication.translate("MainWindow", u"\u540a\u8231", None))
#if QT_CONFIG(shortcut)
        self.btn_pod.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_indicator.setText(QCoreApplication.translate("MainWindow", u"\u677f\u8f7dRGB", None))
#if QT_CONFIG(shortcut)
        self.btn_indicator.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_io_1.setText(QCoreApplication.translate("MainWindow", u"\u6570\u5b57\u8f93\u51fa 1", None))
#if QT_CONFIG(shortcut)
        self.btn_io_1.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_rgb_12.setText(QCoreApplication.translate("MainWindow", u"N/A", None))
#if QT_CONFIG(shortcut)
        self.btn_rgb_12.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_io_0.setText(QCoreApplication.translate("MainWindow", u"\u6570\u5b57\u8f93\u51fa 0", None))
#if QT_CONFIG(shortcut)
        self.btn_io_0.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.btn_rgb.setText(QCoreApplication.translate("MainWindow", u"WS2812", None))
#if QT_CONFIG(shortcut)
        self.btn_rgb.setShortcut(QCoreApplication.translate("MainWindow", u"W", None))
#endif // QT_CONFIG(shortcut)
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u" \u98de\u63a7\u5916\u8bbe ", None))
        self.label_radar_image.setText("")
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), QCoreApplication.translate("MainWindow", u" \u6fc0\u5149\u96f7\u8fbe ", None))
    # retranslateUi


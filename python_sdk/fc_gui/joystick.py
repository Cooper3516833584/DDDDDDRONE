# pip-generator: skip file
# mypy: ignore-errors
from enum import Enum

import PySide6.QtGui
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtUiTools import *
from PySide6.QtWidgets import *


class Joystick(QWidget):
    moveSignal = Signal(float, float)  # (deg,distance)

    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(100, 100)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 50
        self.__fake_angles = []
        self.__bound_color = QColor(255, 255, 255, 255)
        self.__fill_color = QColor(23, 23, 23, 255)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(self.__bound_color)
        bounds = QRectF(
            -self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2
        ).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(self.__fill_color)
        painter.drawEllipse(self._centerEllipse())

    def setColor(self, bound_color: tuple, fill_color: tuple):
        self.__bound_color = QColor(*bound_color)
        self.__fill_color = QColor(*fill_color)
        self.update()

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if limitLine.length() > self.__maxDistance:
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def sendMoveSignal(self):
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        angle = normVector.angle()
        distance = min(currentDistance / self.__maxDistance, 1.0)
        self.moveSignal.emit(angle, distance)

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = self._center()
        self.update()
        self.sendMoveSignal()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
            self.sendMoveSignal()

    def _updateFakeMove(self):
        if len(self.__fake_angles) == 0:
            self.grabCenter = False
            self.movingOffset = self._center()
        else:
            self.grabCenter = True
            normVector = QLineF(self._center(), QPointF(self.__maxDistance, 0))
            normVector.setAngle(self.__fake_angles[0])
            normVector.setLength(self.__maxDistance)
            for angle in self.__fake_angles[1:2]:
                newVector = QLineF(self._center(), QPointF(self.__maxDistance, 0))
                newVector.setAngle(angle if angle > 0 else 360 + angle)
                newVector.setLength(self.__maxDistance)
                toangle = normVector.angleTo(newVector)
                if toangle > 180.01:
                    toangle = toangle - 360
                normVector.setAngle(normVector.angle() + toangle / 2)
            self.movingOffset = normVector.p2()
        self.update()
        self.sendMoveSignal()

    def setFakeMove(self, angle):
        self.__fake_angles.append(angle)
        self._updateFakeMove()

    def resetFakeMove(self, angle=None):
        if angle is None:
            self.__fake_angles = []
        else:
            if angle in self.__fake_angles:
                self.__fake_angles.remove(angle)
        self._updateFakeMove()

    def resizeEvent(self, event: QResizeEvent) -> None:
        self.__maxDistance = min(self.width(), self.height()) / 2 - 20
        return super().resizeEvent(event)

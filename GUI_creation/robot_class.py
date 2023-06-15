from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsScene
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np

JOINT_WIDTH = 20


class Robot_joint_visuals(QGraphicsEllipseItem):

    def __init__(self, joint_num, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.joint_num = joint_num
        robot_pen = QtGui.QPen(QtGui.Qt.black)
        robot_pen.setWidth(5)
        robot_brush = QtGui.QBrush(QtGui.Qt.red)
        self.setPen(robot_pen)
        self.setBrush(robot_brush)

        self.setFlag(QGraphicsItem.ItemIsMovable)

    def mousePressEvent(self, event):
        
        event.accept()  # захват событий мыши

    def mouseMoveEvent(self, event):
        item_coords = self.mapToScene(self.transformOriginPoint())
        transform = self.sceneTransform()
        # вращение звена, вызванное вращением предыдущих звеньев
        scene_angle = np.arctan2(
            transform.m21(), transform.m11()) * 180 / np.pi
        # получаем с помощью матрицы преобразований
        item_angle = self.rotation()  # собственное вращение звена
        mouse_coords = event.scenePos()
        delta = mouse_coords - item_coords
        angle = (((np.arctan2(delta.y(), delta.x())+np.pi/2) * 180 / np.pi +
                 180 + scene_angle+item_angle) % 360)-180  # вращение, задаваемое мышью
        self.scene().angle_changed.emit(self.joint_num, -angle)

    def mouseReleaseEvent(self, event):
        event.ignore()  # отпустить захват событий мыши


class robot_joint:
    def __init__(self, joint_num, left_angle=-180, start_angle=0, right_angle=180, length=100):
        self.joint_num = joint_num
        self.left_angle = left_angle
        self.start_angle = start_angle
        self.right_angle = right_angle
        self.width = JOINT_WIDTH
        self.length = length

        self.visuals: Robot_joint_visuals = Robot_joint_visuals(
            self.joint_num, QRectF(0, 0, self.width, self.length))
        self.set_pos(0, 0)
        self.visuals.setTransformOriginPoint(self.width//2, self.length)

    def update_start_angle(self, angle):
        self.start_angle = angle
        # минус потому что viewScene отражено по оси абсцисс
        self.visuals.setRotation(-angle)

    def update_left_limit(self, angle):
        self.left_angle = angle

    def update_right_limit(self, angle):
        self.right_angle = angle

    def set_pos(self, x, y):
        self.visuals.setPos(x, y-self.length)

    def update_length(self, length):
        self.visuals.setPos(self.visuals.x(), self.visuals.y()+self.length)
        self.length = length
        self.visuals.setRect(0, 0, self.width, self.length)
        self.visuals.setPos(self.visuals.x(), self.visuals.y()-self.length)
        self.visuals.setTransformOriginPoint(self.width//2, self.length)





class Robot_class():
    def __init__(self):

        self.joints = [robot_joint(1)]
        self.joint_count = 1
        self._current_index = 0

    def set_pos(self, x, y):
        """set robot_pos

        Args:
            x (float): x
            y (float): y
        """
        self.joints[0].set_pos(x-10, y)

    def add_joint(self, left_angle=-180, start_angle=0, right_angle=180, length=100):
        """Add joint to robot

        Args:
            left_angle (float, optional): левая граница угла. 
            start_angle (int, optional): начальное значение угла. Defaults to 0.
            right_angle (float, optional): правая граница угла. 
            length (int, optional): длина звена. Defaults to 100.
        """
        self.joint_count += 1
        self.joints.append(robot_joint(
            self.joint_count, left_angle, start_angle, right_angle, length))
        if self.joint_count != 1:
            self.joints[-1].visuals.setParentItem(self.joints[-2].visuals)

    def pop_joint(self):
        """delete last joint
        """
        self.joints[-1].visuals.setParentItem(None)
        self.joints.pop()
        self.joint_count -= 1

    def change_joint_number(self, number):
        """Увеличить количество звеньев. Если хотим больше, чем есть - то создаем больше.

        Args:
            number (_type_): _description_
        """
        assert(isinstance(number, int))
        assert(number >= 0)
        assert(number <= 100)
        while number > self.joint_count:
            self.add_joint()
        while number < self.joint_count:
            self.pop_joint()
        self.joint_count = number

    def __getitem__(self, key):
        """Получение joint через квадратные скобки для удобства

        Args:
            key (int): индекс

        Raises:
            IndexError: Если выходим за границы

        Returns:
            joint: robot_joint
        """
        if key > self.joint_count or key < 0:
            raise IndexError(
                f"Была попытка получить {key} звено в {self.joint_count} звенном роботе")
        return self.joints[key]

    def __iter__(self):
        self._current_index = 0
        return self

    def __next__(self):
        if self._current_index < self.joint_count:
            return self.joints[self._current_index]
        raise StopIteration

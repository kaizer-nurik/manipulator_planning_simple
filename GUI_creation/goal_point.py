from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np

GOAL_POINT_SQUARE_RADIUS = 20
ARROW_LENGTH = 50

QGraphicsEllipseItem()
class GoalPointVisuals(QGraphicsEllipseItem):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        robot_pen = QtGui.QPen(QtGui.Qt.black)
        robot_pen.setWidth(2)
        robot_brush = QtGui.QBrush(QtGui.Qt.blue)
        self.setPen(robot_pen)
        self.setBrush(robot_brush)
        
        self.setFlag(QGraphicsItem.ItemIsMovable)

        self.arrow = QGraphicsLineItem(-GOAL_POINT_SQUARE_RADIUS,
        GOAL_POINT_SQUARE_RADIUS, GOAL_POINT_SQUARE_RADIUS+ARROW_LENGTH,GOAL_POINT_SQUARE_RADIUS,self)
 
        self.arrow.setPos(0,0)
        self.arrow.setPen(robot_pen)

    def mousePressEvent(self, event):

        event.accept()  # захват событий мыши

    def mouseMoveEvent(self, event):
        mouse_coords = event.scenePos()
        self.change_angle_by_dot(mouse_coords)

    def mouseReleaseEvent(self, event):
        event.ignore()  # отпустить захват событий мыши

    def change_angle(self, angle):
        print(self.scenePos().x(),self.scenePos().y())
        self.arrow.setLine(0, 0, ARROW_LENGTH *
                           np.cos(angle), ARROW_LENGTH*np.sin(angle))
    def change_angle_by_dot(self, coords):
        
        item_coords = self.scenePos()
        delta = coords - item_coords 
        print(delta)
        angle = np.arctan2(delta.y(), delta.x())    # вращение, задаваемое мышью
        self.change_angle(angle)


class GoalPoint():
    def __init__(self, scene):
        self.scene = scene
        self.dot = GoalPointVisuals()
        self.dot.setVisible(False)
        self.dot.setRect(-GOAL_POINT_SQUARE_RADIUS, 
         -GOAL_POINT_SQUARE_RADIUS, GOAL_POINT_SQUARE_RADIUS*2, GOAL_POINT_SQUARE_RADIUS*2)
        self.scene.addItem(self.dot)

    def create_goal_point(self):
        self.scene.go_goal_point_mode(self)

    def create_dot(self, coords):
        print(4)
        self.dot.setPos(coords)
        self.dot.setVisible(True)
        
    def make_arrow(self, coords):
        self.dot.change_angle_by_dot(coords)

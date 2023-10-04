from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np
import random
ARROW_LENGTH = 50

class GoalPointVisuals(QGraphicsEllipseItem):

    def __init__(self,radius, number=None,*args, **kwargs):
        super().__init__(*args, **kwargs)
        self.number = number
        robot_pen = QtGui.QPen(QtGui.Qt.black)
        robot_pen.setWidth(2)
        robot_brush = QtGui.QBrush(QtGui.QColor.fromRgb(random.randint(0,255),random.randint(0,255),random.randint(0,255)))
        self.setPen(robot_pen)
        self.setBrush(robot_brush)
        if self.number is not None:
            self.draw_number()

        self.arrow = QGraphicsLineItem(-radius,
                                       radius, radius+ARROW_LENGTH, radius, self)

        self.arrow.setPos(0, 0)
        self.arrow.setPen(robot_pen)
        self.angle = 0

    def set_number(self,number):
        self.number = number
        if self.number is not None:
            self.draw_number()
            
    def draw_number(self):
        if self.scene() is None:
            return
        font = QtGui.QFont()
        font.setPointSize(10)
        
        
        text = self.scene().addText(str(self.number),font)
        text.setTransform(QtGui.QTransform(1,0,0,-1,int(self.x()),int(self.y())))
    def change_angle(self, angle):
        self.angle = angle
        self.arrow.setLine(0, 0, ARROW_LENGTH *
                           np.cos(angle), ARROW_LENGTH*np.sin(angle))


    def change_angle_by_dot(self, coords):

        item_coords = self.scenePos()
        delta = coords - item_coords
        # вращение, задаваемое мышью
        angle = np.arctan2(delta.y(), delta.x())
        self.change_angle(angle)
        
    def update_radius(self,radius):
        self.setRect(-radius,
                         -radius, radius*2, radius*2)
        


class GoalPoint():
    def __init__(self,radius, scene = None,number = None):

        self.scene = scene
        self.number = number
        self.radius = radius
        self.dot = GoalPointVisuals(self.radius,self.number)
        self.dot.setVisible(False)
        self.dot.setRect(-self.radius,
                         -self.radius, self.radius*2, self.radius*2)
        if scene is not None:
            self.scene.addItem(self.dot)
        
    def set_number(self, number):
        self.number = number
        self.dot.set_number(number)
    def create_goal_point(self):
        self.scene.go_goal_point_mode(self)
        
    def create_by_coords(self,x,y,angle):
        self.create_dot(QPointF(x,y))
        self.set_angle((angle-180)*np.pi/180)

    def create_dot(self, coords):
        self.dot.setPos(coords)
        self.dot.setVisible(True)
        
    def update_radius(self,radius):
        self.radius = radius
        self.dot.update_radius(radius)

    def make_arrow(self, coords):
        self.dot.change_angle_by_dot(coords)

    def set_scene(self,scene):
        self.scene=scene
        self.scene.addItem(self.dot)
        self.dot.setVisible(True)
        self.dot.draw_number()
    def x(self):
        return self.dot.x()

    def y(self):
        return self.dot.y()

    def angle(self):
        return self.dot.angle*180/np.pi
    
    def set_angle(self,angle):
        self.dot.change_angle(angle)
    
    def reset(self):
        self.dot.setVisible(False)

    def copy(self,scene=None):
        if scene == None:
            scene = self.scene
        new = GoalPoint(self.radius,scene)
        new.set_number(self.number)
        new.set_scene(scene)
        new.create_by_coords(self.x(),self.y(),self.angle())
        new.set_angle(self.dot.angle)
        return new
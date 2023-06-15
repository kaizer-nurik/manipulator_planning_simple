from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsScene
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np
from  PySide6 import QtCore
class Robo_scene(QGraphicsScene):
    angle_changed = Signal(int, float)

    def __init__(self):
        super().__init__()
        self.obstacle = None
        
    def go_poli_mode(self,obstacle):
        self.obstacle = obstacle
        
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        if self.obstacle is None:
            event.ignore()
            return
        event.accept()
        dot_pos = event.scenePos()
        print(dot_pos)
        self.obstacle.add_dot(dot_pos)
        
        if self.obstacle.is_done():
            self.obstacle.set_dots_visible(False)
            self.obstacle = None
            

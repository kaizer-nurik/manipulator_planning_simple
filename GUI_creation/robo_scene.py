from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsScene
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np
from  PySide6 import QtCore
import obstacles
class Robo_scene(QGraphicsScene):
    angle_changed = Signal(int, float)

    def __init__(self):
        super().__init__()
        self.obstacle = None
        self.goal_point_manager = None
    
    def reset(self):
        self.obstacle = None
        self.goal_point_manager = None
        
    def go_poli_mode(self,obstacle):
        self.obstacle = obstacle
        
    def go_goal_point_mode(self,gpm):
        self.obstacle = None
        self.goal_point_manager = gpm
        
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        
        if (self.obstacle is None) and (self.goal_point_manager is None):
            event.ignore()
            return
        dot_pos = event.scenePos()
        
        if self.obstacle is not None:
            event.accept()
            
            self.obstacle.add_dot(dot_pos)
            
            if self.obstacle.is_done():
                self.obstacle.set_dots_visible(False)
                self.obstacle = None
        if self.goal_point_manager is not None:
            self.goal_point_manager.create_dot(dot_pos)
            
    def mouseMoveEvent(self, event) -> None:
        super().mouseMoveEvent(event)
        
        if (self.obstacle is None) and (self.goal_point_manager is None):
            event.ignore()
            return
        dot_pos = event.scenePos()    
        
        if self.goal_point_manager is not None:
            self.goal_point_manager.make_arrow(dot_pos)
            
    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        if self.goal_point_manager is not None:
            self.goal_point_manager = None

    def keyPressEvent(self, event):
        super().keyPressEvent(event)
        if isinstance(self.focusItem(),obstacles.Obstacle):
            if event.matches(QtGui.QKeySequence.Delete):
                self.focusItem().suicide()
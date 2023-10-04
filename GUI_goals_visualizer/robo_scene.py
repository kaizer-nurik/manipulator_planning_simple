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
    

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

            
    def mouseMoveEvent(self, event) -> None:
        super().mouseMoveEvent(event)
      
            
    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
      
    def keyPressEvent(self, event):
        super().keyPressEvent(event)
  
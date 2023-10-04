from PySide6.QtWidgets import QApplication,QGraphicsEllipseItem,QGraphicsPolygonItem,QGraphicsItem,QGraphicsScene
from PySide6.QtCore import QRectF,Qt, QPointF, Signal,QObject
from PySide6.QtGui import QPolygonF
from PySide6 import QtGui
import numpy as np

class Dot(QGraphicsEllipseItem):
    def __init__(self,dot,R=20,*args,**kwargs):
        super().__init__(*args,**kwargs)
        dot_pen = QtGui.QPen(QtGui.Qt.black)
        dot_pen.setWidth(5)
        dot_brush = QtGui.QBrush(QtGui.Qt.blue)
        self.setPen(dot_pen)
        self.setBrush(dot_brush)
        self.R = R
        self.dot = dot
        self.setRect(self.dot.x()-self.R//2,self.dot.y()-self.R//2,self.R,self.R)
    def get_dot(self):
        return self.dot
    
    def mousePressEvent(self, event):
        if self.scene().obstacle is self.parentItem() and self.parentItem().dots[0] is self:
            # self.parentItem().add_dot(self.dot)
            self.scene().obstacle.set_dots_visible(False)
            self.scene().obstacle = None
            return
        if self.scene().obstacle is None:
            event.accept()  # захват событий мыши
        

    def mouseMoveEvent(self, event):
        
        mouse_coords = QPointF(event.scenePos().x()-self.R//2,event.scenePos().y()-self.R//2)
        self.dot = mouse_coords
        self.setRect(self.dot.x()-self.R//2,self.dot.y()-self.R//2,self.R,self.R)
        self.parentItem().update()


    def mouseReleaseEvent(self, event):
        event.ignore()  # отпустить захват событий мыши


    
    
class Obstacle(QGraphicsPolygonItem):
    def __init__(self,manager,*args,**kwargs):
        
        super().__init__(*args,**kwargs)
        self.manager = manager
        self.dots_visible=True
        self.fig = QPolygonF()
        obs_pen = QtGui.QPen(QtGui.Qt.black)
        obs_pen.setWidth(5)

        obs_brush = QtGui.QBrush(QtGui.Qt.green)
        self.setPen(obs_pen)
        self.setBrush(obs_brush)
        self.setZValue(1)
        self.dots = []
        
    def set_dots_visible(self,param):
        dots_visible = param
        for dot in self.dots:
            dot.setVisible(param)
        
    def add_dot(self,dot):
        self.dots.append(Dot(dot,20,self))        
        self.setPolygon(self.poly())
        
    def is_done(self):
        return self.fig.isClosed() and self.fig.count()>1
    
    def update(self):
        self.setPolygon(self.poly())
        
    def poly(self):
        p = QPolygonF()
        for dot in self.dots:
            p.append(dot.get_dot())
        return p
    
    def suicide(self):
        pass
        

                

class ObstacleManager(QObject):
    def __init__(self,X_line_edit, Y_line_edit,poli_list,scene):
        self.X_line_edit = X_line_edit
        self.Y_line_edit = Y_line_edit
        self.poli_list = poli_list
        self.scene = scene
        self.obstacles = list()
        self._counter = 0
        
    def off_dots(self):
        for obstacle in self.obstacles:
            obstacle.set_dots_visible(False)
            

    def add_obstacle(self):
        self.off_dots()
        self.obstacles.append(Obstacle(self))
        if self.scene is not None:
            self.scene.addItem(self.obstacles[-1])
    def reset(self):
        for obs_ind in range(len(self.obstacles)):
            self.obstacles[0].setParentItem(None)
            if self.scene is not None:
                self.scene.removeItem(self.obstacles[0])
            self.obstacles.pop(0)

    def delete(self,obstacle_to_delete):
        for index, obstacle in enumerate(self.obstacles):
            if obstacle is obstacle_to_delete:
                self.obstacles[index].setParentItem(None)
                if self.scene is not None:
                    self.scene.removeItem(self.obstacles[index])
                self.obstacles.pop(index)
    def __iter__(self):
        self._current_index = 0
        return self

    def __next__(self):
        if self._current_index < len(self.obstacles):
            self._current_index+=1
            return Polygon(self.obstacles[self._current_index-1].polygon())
        raise StopIteration
    
    def copy(self,scene):
        new = ObstacleManager(None,None,None,scene)
        for obstacle in self.obstacles:
            new.add_obstacle()
            poli = new.obstacles[-1]
            poli.set_dots_visible(False)
            for vertex in obstacle.dots:
                poli.add_dot(vertex.dot)
            poli.set_dots_visible(False)
        return new
            


class Polygon():
    """Вспомогательный итератор для полигонов, для упрощения кода
    """
    def __init__(self, poly_qt:QGraphicsPolygonItem):
        self.poly_qt:QGraphicsPolygonItem = poly_qt
        self.vertexes = poly_qt.toList()
        self.vertex_count:int = len(self.vertexes)
        self._counter = 0

    def __iter__(self):
        self._current_index = 0
        return self

    def __next__(self):
        if self._current_index < self.vertex_count:
            self._current_index+=1
            return self.vertexes[self._current_index-1]
        raise StopIteration
    

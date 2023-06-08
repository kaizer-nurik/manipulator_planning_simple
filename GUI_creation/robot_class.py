from PySide6.QtWidgets import QGraphicsEllipseItem
from PySide6.QtCore import QRectF
from PySide6 import QtGui

class robot_joint:
    def __init__(self,left_angle = -3.1415, start_angle = 0, right_angle = 3.1415,length = 100):
        self.left_angle = left_angle
        self.start_angle = start_angle
        self.right_angle = right_angle
        self.length = length
        self.width  = 20
        robot_pen = QtGui.QPen(QtGui.Qt.black)
        robot_pen.setWidth(5)
        
        robot_brush = QtGui.QBrush(QtGui.Qt.red)
        self.visuals:QGraphicsEllipseItem = QGraphicsEllipseItem(QRectF(0,0,self.width,self.length))
        self.set_pos(0,0)
        self.visuals.setPen(robot_pen)
        self.visuals.setBrush(robot_brush)
        self.visuals.setTransformOriginPoint(self.width//2,self.length)
        
    def set_pos(self,x,y):
        self.visuals.setPos(x,y-self.length)
        
        
    def update_length(self,length):
        self.visuals.setPos(self.visuals.x(),self.visuals.y()+self.length)
        self.length = length
        self.visuals.boundingRect(0,0,self.width,self.length)
        self.visuals.setPos(self.visuals.x(),self.visuals.y()-self.length)
        self.visuals.setTransformOriginPoint(self.width//2,self.length)

        

class Robot_class:
    def __init__(self):
        self.joints = [robot_joint()]
        self.joint_count = 1 
        self._current_index = 0
    def set_pos(self,x,y):
        """set robot_pos

        Args:
            x (float): x
            y (float): y
        """
        self.joints[0].set_pos(x-10,y)
        
    def add_joint(self,left_angle = -3.1415, start_angle = 0, right_angle = 3.1415,length = 100):
        """Add joint to robot

        Args:
            left_angle (float, optional): левая граница угла. Defaults to -3.1415.
            start_angle (int, optional): начальное значение угла. Defaults to 0.
            right_angle (float, optional): правая граница угла. Defaults to 3.1415.
            length (int, optional): длина звена. Defaults to 100.
        """
        self.joints.append(robot_joint(left_angle,start_angle,right_angle,length))
        self.joint_count +=1
        if self.joint_count!= 1:
            self.joints[-1].visuals.setParentItem(self.joints[-2].visuals)
    def pop_joint(self):
        """delete last joint
        """
        self.joints[-1].visuals.setParentItem(None)
        self.joints.pop()
        self.joint_count-=1
    
    def change_joint_number(self,number):
        """Увеличить количество звеньев. Если хотим больше, чем есть - то создаем больше.

        Args:
            number (_type_): _description_
        """
        assert(isinstance(number,int))
        assert(number >=0)
        assert(number <=100)
        while number>self.joint_count:
            self.add_joint()
        while number<self.joint_count:
            self.pop_joint()
        self.joint_count = number
        
    def __getitem__(self,key):
        """Получение joint через квадратные скобки для удобства

        Args:
            key (int): индекс

        Raises:
            IndexError: Если выходим за границы

        Returns:
            joint: robot_joint
        """
        if key > self.joint_count or key < 0:
            raise IndexError(f"Была попытка получить {key} звено в {self.joint_count} звенном роботе")
        return self.joints[key]
            
    def __iter__(self):
        self._current_index = 0
        return self
    def __next__(self):
        if self._current_index < self.joint_count:
            return self.joints[self._current_index]
        raise StopIteration
    

        
    
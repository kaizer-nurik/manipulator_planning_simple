from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsScene
from PySide6.QtCore import QRectF, Qt, QPointF, Signal, QObject
from PySide6 import QtGui
import numpy as np
from PySide6 import QtCore
import numpy as np
DIVIDE_FACTOR = 10
class HeatbarScene(QGraphicsScene):
    """Класс хендлер сцены с heatbar

    Parent:
        QGraphicsScene (QGraphicsScene): тип сцены из Qt
    """
    def __init__(self, value: int = 5000,height: int = 5000):
        """конструктор

        Args:
            value (int, optional): Макс значение. Defaults to 1000.
            height (int, optional): высота в пикс на экране. Defaults to 5000.
        """
        super().__init__()
        self.max_value = value
        self.max_height = height
        self.log_scale = False

    def set_log_scale(self,log_scale):
        self.log_scale = log_scale
        self.draw()

    def set_max_value(self, value: int):
        self.max_value = value

    def value_to_hsv(self, value: int) -> QtGui.QColor:
        """Перевод value в HSV цвет из QT

        Args:
            value (int): значение
        """

        if self.log_scale:
            mapped_value = np.round((255//DIVIDE_FACTOR+255//DIVIDE_FACTOR+280)*np.log10(value)/np.log10(self.max_value))
        else:
            mapped_value = np.round((255//DIVIDE_FACTOR+255//DIVIDE_FACTOR+280)*value/self.max_value)

        
        
        if mapped_value <=255//DIVIDE_FACTOR:
            return QtGui.QColor.fromHsv( 280, 255,mapped_value*DIVIDE_FACTOR)
        elif mapped_value <=(255//DIVIDE_FACTOR+280):
            return QtGui.QColor.fromHsv( 280-(mapped_value-255//DIVIDE_FACTOR), 255,255)
        elif mapped_value<=(255//DIVIDE_FACTOR+280+255//DIVIDE_FACTOR):
            return QtGui.QColor.fromHsv( 0, 255-(mapped_value-255//DIVIDE_FACTOR-280)*DIVIDE_FACTOR,255)
        return QtGui.QColor.fromHsv( 0, 0,255)
                                    
    
    def pixel2value(self, pixel:int):
        """Перевод из пикселей в значение

        Args:
            pixel (_type_): пиксели.
        """
        if self.log_scale:
            return 10**(np.log10(self.max_value)*pixel/self.max_height)
        return self.max_value * pixel/self.max_height
    
    def value2pixel(self, value:int):
        """Перевод из значений в пискели

        Args:
            value (_type_): пиксели.
        """
        if self.log_scale:
            return self.max_height * np.log10(value)/np.log10(self.max_value)
        return self.max_height * value/self.max_value
    
    
    def draw(self):
        self.clear()
        line_pen = QtGui.QPen()
        line_pen.setColor(QtGui.QColor.fromHsv(0,0,0))
        line_pen.setWidth(3)
        
        for pixel in range(0,self.max_height):
            self.addLine(0,pixel,50,pixel,self.value_to_hsv(self.pixel2value(pixel)))
        self.addLine(0,0,0,self.max_height,line_pen)
        self.addLine(50,0,50,self.max_height,line_pen)    
        
        line_pen.setWidth(2)
        font = QtGui.QFont()
        font.setPointSize(10)
        
        if  self.log_scale:
            values = [10**i for i in range(int(np.log10(self.max_value)+1))]
            values.append(self.max_value)
        else:
            values = range(0,self.max_value//10 * 11 ,self.max_value//10 )
        for value in values:
            self.addLine(45,self.value2pixel(value),55,self.value2pixel(value),line_pen)
            text = self.addText(str(value),font)
            text.setTransform(QtGui.QTransform(1,0,0,-1,60,self.value2pixel(value)+12))

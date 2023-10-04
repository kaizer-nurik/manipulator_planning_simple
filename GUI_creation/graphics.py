from PySide6.QtWidgets import QApplication,QFileDialog,QGraphicsScene,QGraphicsScene
from PySide6 import QtGui

from robot_class import Robot_class

# def draw_grid(scene:QGraphicsScene,step:int = 50):
#     """Draw grid in scene with step 50

#     Args:
#         scene (QGraphicsScene): Сцена
#         step (int, optional): step of grid. Defaults to 50.
#     """
#     pen = QtGui.QPen(QtGui.Qt.black)
#     pen.setCosmetic(True)  # ***
    
#     for coord in range(-500, 500, step):
#         line = scene.addLine( coord, -500, coord,500, pen)
#         line.setZValue(-10)# Порядок объекта, задний план.
#         scene.addLine( -500,coord, 500, coord, pen)
#         line.setZValue(-10) # Порядок объекта, задний план.

def draw_robot(scene:QGraphicsScene,robot:Robot_class):
    """Draw robot in scene

    Args:
        scene (QGraphicsScene): _description_
        robot (Robot_class): _description_
    """
    center = 0
    robot_pen = QtGui.QPen(QtGui.Qt.black)
    robot_pen.setWidth(5)
    
    robot_brush = QtGui.QBrush(QtGui.Qt.red)
    x_coord = center
    y_coord = center
    scene.addEllipse(x_coord-25,y_coord-25,50,50,robot_pen,robot_brush)
    
    scene.addItem(robot[0].visuals)
    robot.set_pos(x_coord,y_coord)
    
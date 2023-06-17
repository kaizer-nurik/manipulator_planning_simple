from PySide6.QtWidgets import QApplication,QFileDialog,QGraphicsScene
from PySide6 import QtGui
import pyqtgraph as pg
from datetime import datetime
from PyQt6.QtCore import pyqtSlot
import os
from PySide6.QtCore import Qt
from pathlib import Path
from PySide6.QtCore import QRectF
import graphics
from robot_class import Robot_class
from robo_scene import Robo_scene
from obstacles import ObstacleManager, Obstacle
from GUI_ui import Ui_MainWindow
from goal_point import GoalPoint
from xml_maker import to_xml
uiclass, baseclass = pg.Qt.loadUiType(
    "./GUI.ui")  # подгузка файла с дизайном

class MainWindow(uiclass, baseclass):  # класс окна
    def __init__(self):
        super().__init__()  # инициализация родительских классов
        self.setupUi(self)  # инициализация
        #подключение к кнопке "Обзор" выбора файла
        self.scene = Robo_scene()             

        self.current_joint=0
        self.robot = Robot_class()
        self.robot.change_joint_number(1)
        self.obstacles = ObstacleManager(self.poli_x_text,self.poli_y_text,self.poly_list,self.scene)
        self.goal_point = GoalPoint(self.scene)
        
        self.create_poli_btn.clicked.connect(self.create_poli)
        self.file_choose_btn.clicked.connect(self.open_file_dialog) 
        self.plot = self.graphicsView # для удобства
        self.robot_joint_count_spin.valueChanged.connect(self.change_robot_joints_count)
        self.robot_change_j_spin.valueChanged.connect(self.change_robot_joints_change)
        self.joint_length_line_edit.textChanged.connect(self.change_joint_length)
        self.joint_length_line_edit.setText('100')
        self.left_limit_text.textChanged.connect(self.change_joint_left_limit)
        self.left_limit_text.setText('-180')
        self.right_limit_text.textChanged.connect(self.change_joint_right_limit)
        self.right_limit_text.setText('180')
        self.start_angle_text.textChanged.connect(self.change_joint_start_angle)
        self.start_angle_text.setText('0')
        self.zoom_slider.valueChanged.connect(self.ZoomSliderChange)
        self.create_goal_point_btn.clicked.connect(self.goal_point_connect)
        self.export_btn.clicked.connect(self.export_xml)
        SCENE_MAX_LENGTH = 1000
        # self.scene_rect  = QRectF(-100000,-100000,100000,100000)
        self.scene.angle_changed.connect(self.on_joint_angle_change_by_mouse)    
        # self.scene.setSceneRect(self.scene_rect)
        # graphics.draw_grid(self.scene,10)
        graphics.draw_robot(self.scene,self.robot)
        self.graphicsView.setSceneRect(QRectF(-5000, -5000, 10000, 10000))
        self.graphicsView.setScene(self.scene)
        self.graphicsView.centerOn(0,0)
        # self.graphicsView.scale(self.plot.geometry().width()/SCENE_MAX_LENGTH,self.plot.geometry().width()/SCENE_MAX_LENGTH)

    @pyqtSlot()
    def export_xml(self):
        to_xml(self.file_choose_edit.text(),self.robot,self.obstacles,self.goal_point)
        
    @pyqtSlot()
    def goal_point_connect(self):
        print(1)
        self.goal_point.create_goal_point()
    @pyqtSlot(int)
    def ZoomSliderChange(self,value):
        tr = self.graphicsView.transform()
        self.graphicsView.setTransform(QtGui.QTransform(value/100, 0,0, value/100, tr.dx(), tr.dy()))
        # self.graphicsView.scale(value/100,value/100)
    @pyqtSlot()
    def create_poli(self):
        """Слот для кнопки "Создать полигон"
        создает новый полигон. Нажимая на сцену, можно создать его  точки
        """
        self.obstacles.create_obstacle()
        
    @pyqtSlot(int, float)
    def on_joint_angle_change_by_mouse(self,joint_num,angle):
        self.robot[joint_num-1].update_start_angle(angle)
        if (joint_num-1) == self.current_joint:
            self.start_angle_text.blockSignals(True)
            self.start_angle_text.setText(f"{angle:.2f}")
            self.start_angle_text.blockSignals(False)
        
    @pyqtSlot(str)
    def change_joint_start_angle(self,text):
        try:
            self.robot[self.current_joint].update_start_angle(float(text))
            self.start_angle_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.start_angle_text.setStyleSheet("color: red;")
            print(e)
    @pyqtSlot(str)
    def change_joint_right_limit(self,text):
        try:
            self.robot[self.current_joint].update_right_limit(float(text))
            self.right_limit_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.right_limit_text.setStyleSheet("color: red;")
            print(e)
                
    @pyqtSlot(str)
    def change_joint_left_limit(self,text):
        try:
            self.robot[self.current_joint].update_left_limit(float(text))
            self.left_limit_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.left_limit_text.setStyleSheet("color: red;")
            print(e)
    @pyqtSlot(str)
    def change_joint_length(self,text):
        try:
            self.robot[self.current_joint].update_length(float(text))
            
            self.joint_length_line_edit.setStyleSheet("color: green;")
        except BaseException as e:
            self.joint_length_line_edit.setStyleSheet("color: red;")
            print(e)
    @pyqtSlot(int)
    def change_robot_joints_change(self,value):
        self.current_joint = int(value)-1
        self.joint_length_line_edit.blockSignals(True)
        self.joint_length_line_edit.setText(str(self.robot[value-1].length))
        self.joint_length_line_edit.blockSignals(False)
        self.left_limit_text.blockSignals(True)
        self.left_limit_text.setText(f"{self.robot[value-1].left_angle:.2f}")
        self.left_limit_text.blockSignals(False)
        self.right_limit_text.blockSignals(True)
        self.right_limit_text.setText(f"{self.robot[value-1].right_angle:.2f}")
        self.right_limit_text.blockSignals(False)
        self.start_angle_text.blockSignals(True)
        self.start_angle_text.setText(f"{self.robot[value-1].start_angle:.2f}")
        self.start_angle_text.blockSignals(False)
    @pyqtSlot(int)
    def change_robot_joints_count(self,value):
        """Change joints count according to spinbox

        Args:
            value (int): spinbox value
        """
        self.robot.change_joint_number(value)
        self.robot_change_j_spin.setMaximum(value)
    @pyqtSlot()
    def open_file_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог сохранения файла и записывает имя в соотвествующий виджет
        """
        fname = QFileDialog.getSaveFileName(
            self,
            "Сохранить в",
            "test",
            "XML (*.xml);; All Files (*)",
        )
        #self.plot.fitInView(QRectF(0,0,1000,1000))
        self.file_choose_edit.setText(fname[0])


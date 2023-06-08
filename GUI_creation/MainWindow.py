from PySide6.QtWidgets import QApplication,QFileDialog,QGraphicsScene
from PySide6 import QtGui
import pyqtgraph as pg
from datetime import datetime
from PyQt6.QtCore import pyqtSlot
import os
from pathlib import Path
from PySide6.QtCore import QRectF
import graphics
from robot_class import Robot_class
uiclass, baseclass = pg.Qt.loadUiType(
    "./GUI.ui")  # подгузка файла с дизайном

class MainWindow(uiclass, baseclass):  # класс окна
    def __init__(self):
        super().__init__()  # инициализация родительских классов
        self.setupUi(self)  # инициализация
        #подключение к кнопке "Обзор" выбора файла
        self.file_choose_btn.clicked.connect(self.open_file_dialog) 
        self.plot = self.graphicsView # для удобства
        self.robot_joint_count_spin.valueChanged.connect(self.change_robot_joints_count)
        self.robot_change_j_spin.valueChanged.connect(self.change_robot_joints_change)
        self.joint_length_line_edit.textChanged.connect(self.change_joint_length)
        self.joint_length_line_edit.setText('100')
        self.robot = Robot_class()
        self.robot.change_joint_number(1)
        SCENE_MAX_LENGTH = 1000
        self.scene_rect  = QRectF(0,0,SCENE_MAX_LENGTH,SCENE_MAX_LENGTH)
        self.scene = QGraphicsScene()                    
        self.scene.setSceneRect(self.scene_rect)
        graphics.draw_grid(self.scene,10)
        graphics.draw_robot(self.scene,self.robot)
        self.plot.setScene(self.scene)
        print(self.plot.geometry().width())
        self.plot.scale(self.plot.geometry().width()/SCENE_MAX_LENGTH,self.plot.geometry().width()/SCENE_MAX_LENGTH)
        

    @pyqtSlot()
    def change_joint_length(self,text):
        try:
            self.robot[self.robot_change_j_spin.value()].update_length(int(text))
        except BaseException as e:
            print(e)
    @pyqtSlot()
    def change_robot_joints_change(self,value):
        self.joint_length_line_edit.setText(str(self.robot[value].length))
    @pyqtSlot()
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


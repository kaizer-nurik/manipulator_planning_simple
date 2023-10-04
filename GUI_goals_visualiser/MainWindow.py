from PySide6.QtWidgets import QApplication, QFileDialog, QGraphicsScene, QMainWindow
from PySide6 import QtGui
from PySide6.QtCore import Qt, QTimer, QPoint,QPointF, QEasingCurve, QRectF, QBuffer, QIODevice
import graphics
from PIL import Image, ImageQt
from robot_class import Robot_class
from robo_scene import Robo_scene
from obstacles import ObstacleManager, Obstacle
from goal_point import GoalPoint
from xml_maker import to_xml
from xml_parser import read_xml
import io
import os
import numpy as np
import moviepy.editor as mp
import copy
# from scipy import interpolate
from GUI import Ui_MainWindow
import math
from typing import List
from heatbar_scene import HeatbarScene
import json
import re
import time
class MainWindow(QMainWindow, Ui_MainWindow):  # класс окна
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.heatmap_scene = HeatbarScene(33810,500)
        self.heatbarView.setScene(self.heatmap_scene)
        
        self.heatbarView.scale(1, -1)  
        self.heatmap_scene.draw()
        self.heatbarView.setSceneRect(0,-50,500,5000)
        # подключение к кнопке "Обзор" выбора файла
        self.scene = Robo_scene()
        self.log_scale_checkBox.stateChanged.connect(self.set_log_scale_callback)

        self.current_joint = 0
        self.robot = Robot_class()
        self.robot.change_joint_number(1)
        self.obstacles = ObstacleManager(
            None, None, None, self.scene)
        self.goal_point = GoalPoint(10,self.scene)
        self.dataset_dot_spin.valueChanged.connect(self.dataset_value_changed)
        self.folder_choose_btn.clicked.connect(self.open_folder_dialog)
        self.goal_point_delta_edit.textChanged.connect(
            self.change_goal_point_radius)
        self.process_dataset_btn.clicked.connect(self.process_dataset)
        self.plot = self.graphicsView  # для удобства
        
        self.zoom_slider.valueChanged.connect(self.ZoomSliderChange)
        # self.scene_rect  = QRectF(-100000,-100000,100000,100000)
        self.animate_btn.clicked.connect(self.animate_robot)
        self.csv_choose_btn.clicked.connect(self.open_csv_dialog)
        # self.scene.setSceneRect(self.scene_rect)
        # graphics.draw_grid(self.scene,10)
        graphics.draw_robot(self.scene, self.robot)
        self.graphicsView.setSceneRect(-5000, -5000, 10000, 10000)
        self.graphicsView.setScene(self.scene)
        self.graphicsView.centerOn(0, 0)
        self.graphicsView.scale(1, -1)
        # self.graphicsView.scale(self.plot.geometry().width()/SCENE_MAX_LENGTH,self.plot.geometry().width()/SCENE_MAX_LENGTH)

    def set_log_scale_callback(self,value):
        self.heatmap_scene.set_log_scale(value)
        try:
            for number,goal in self.goals:
                goal.dot.setBrush(self.heatmap_scene.value_to_hsv(self.number2open_nodes[number]))
        except BaseException as e:
            print(e)

    def dataset_value_changed(self,value):
        self.heatmap_scene.set_max_value(value)
        self.heatmap_scene.draw()
        try:
            for number,goal in self.goals:
                goal.dot.setBrush(self.heatmap_scene.value_to_hsv(self.number2open_nodes[number]))
        except BaseException as e:
            print(e)
            
        
    def process_dataset(self):
        files = []
        error = False
        try:
            files = os.listdir(self.folder_choose_edit.text())
        except os.error as e:
            print(e)
            error = True
        
        
        try:
            with open(self.csv_choose_edit.text(),'r') as f:
                scenes_data = json.load(f)
        except BaseException as e:
            print(e)
            error = True        
         
        if error:
            return
        
        self.number2open_nodes = dict()
        
        for data in scenes_data:
            self.number2open_nodes[data["_number"]] = data["opened_nodes"]
        scene = Robo_scene()
        self.goals :List[(int,GoalPoint)] = []
        self.trajectories :dict[str] = dict()
        self.robot.joints[0].visuals.setParentItem(None)
        csv,self.robot, self.obstacles, self.goal_point =  read_xml(self.folder_choose_edit.text()+"\\\\"+files[0],scene)
        number = int(re.findall("\d+",files[0])[-1])
        # print(files[0],number)
        self.goal_point.set_number(number)
        self.goals.append((number,self.goal_point))
        self.trajectories[number] = csv
        for file in files[1:]:
            csv,Robot, obstacles, goal =  read_xml(self.folder_choose_edit.text()+"\\\\"+file, None)
            number = int(re.findall("\d+",file)[-1])
            # print(file,number)
            goal.set_number(number)
            self.goals.append((number,goal))
            self.trajectories[number] = csv

        for number,goal in self.goals:
            print(number)
            print(self.number2open_nodes)
            goal.dot.setBrush(self.heatmap_scene.value_to_hsv(self.number2open_nodes[number]))
        for number,goal in self.goals:
            
            goal.set_scene(scene)    
            
        self.scene = scene
        graphics.draw_robot(self.scene, self.robot)
        self.graphicsView.setScene(self.scene)
        

        
    def change_goal_point_radius(self, text):
        try:
            for number,goal in self.goals:
                goal.update_radius(float(text))

            self.goal_point_delta_edit.setStyleSheet("color: green;")
        except BaseException as e:
            self.goal_point_delta_edit.setStyleSheet("color: red;")
            print(e)    
    def unlock_view(self):
        self.graphicsView.setInteractive(True)
        unlock_brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        self.graphicsView.setBackgroundBrush(unlock_brush)
        self.graphicsView.setScene(self.scene)

    def QImageToPil(self,img):
        buffer = QBuffer()
        buffer.open(QBuffer.ReadWrite)
        img.save(buffer, "PNG")
        pil_im = Image.open(io.BytesIO(buffer.data()))
        buffer.close()
        return pil_im
    def lock_view(self):
        self.graphicsView.setInteractive(False)
        lock_brush = QtGui.QBrush(QtGui.QColor(230, 230, 203))
        self.graphicsView.setBackgroundBrush(lock_brush)

    def animate_robot(self):
        for number,goal in self.goals:
            self.anim_scene = Robo_scene()
            self.plot.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.plot.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            

            heatbar_img = self.QImageToPil(QtGui.QImage(self.heatbarView.grab().toImage()))
            self.current_joint = 0
            self.anim_robot = self.robot.copy()
            self.anim_obstacles = self.obstacles.copy(self.anim_scene)
            self.anim_goal_point = goal.copy()
            self.anim_goal_point.set_scene(self.anim_scene)
            graphics.draw_robot(self.anim_scene, self.anim_robot)
            self.graphicsView.setScene(self.anim_scene)
            self.graphicsView.centerOn(0, 0)
            trajectory = self.trajectories[number]
            self.anim = np.loadtxt(trajectory.split('\n'), delimiter=',')
            if len(self.anim.shape) == 1:
                self.anim = self.anim.reshape(self.anim.shape[0], 1)

            images = []
            for angles in self.anim:
                

                QApplication.processEvents()
                self.anim_robot.set_angles(tuple(angles))
                self.plot.update()
                self.update()
                QApplication.processEvents()
  
                images.append(self.QImageToPil( QtGui.QImage(self.plot.grab().toImage())))

            
            for i in range(len(images)):
                to_cat = [images[i],heatbar_img]
                widths, heights = zip(*(i.size for i in to_cat))

                total_width = sum(widths)
                max_height = max(heights)

                new_im = Image.new('RGB', (total_width, max_height))

                x_offset = 0
                for im in to_cat:
                    new_im.paste(im, (x_offset,0))
                    x_offset += im.size[0]
                images[i]=new_im
                
            images.pop(0)
            
            os.makedirs("./GIFs",exist_ok=True)
            
            images[0].save(f"./GIFs/case_{number}.gif", append_images=images[1:],
             save_all=True, duration=20,loop=0)
            
            
            os.makedirs("./MP4s",exist_ok=True)
            clip = mp.VideoFileClip(f"./GIFs/case_{number}.gif")
            clip.write_videofile(f"./MP4s/case_{number}.mp4")
           
            self.plot.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
            self.plot.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

            # ANIM_COUNT_INTP = 10000
            # axes = []
            # for i in range(self.anim.shape[1]):
            #     axes.append(np.interp(np.linspace(0, 100, ANIM_COUNT_INTP), np.linspace(
            #         0, 100, self.anim.shape[0]), self.anim[:, i]))
            # self.anim_temp = np.vstack(axes).T
        # mymin,mymax = 0,10000
        # X = np.linspace(mymin,mymax,self.anim.shape[0])
        # Y = np.linspace(np.min(self.anim),np.max(self.anim),self.robot.joint_count)

        # x,y = np.meshgrid(Y,X)

        # f = interpolate.interp2d(x,y,self.anim,kind='cubic')

        # # use linspace so your new range also goes from 0 to 3, with 8 intervals
        # Xnew = np.linspace(mymin,mymax,10000)
        # Ynew = np.linspace(np.min(self.anim),np.max(self.anim),self.robot.joint_count)

        # self.anim_temp = f(Ynew,Xnew)
        #




    def ZoomSliderChange(self, value):
        tr = self.graphicsView.transform()
        self.graphicsView.setTransform(QtGui.QTransform(
            value/100, 0, 0, -value/100, tr.dx(), tr.dy()))
        # self.graphicsView.scale(value/100,value/100)


    def open_csv_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог открытия файла и записывает имя в соотвествующий виджет
        """
        fname = QFileDialog.getOpenFileName(
            self,
            "Открыть",
            "",
            "json (*.json);; All Files (*)",
        )
        # self.plot.fitInView(QRectF(0,0,1000,1000))
        self.csv_choose_edit.setText(fname[0])

    def open_folder_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог открытия папки датасета
        """
        fname = QFileDialog.getExistingDirectory(
            None, 'Select a folder:', 'C:\\', QFileDialog.ShowDirsOnly)
        # self.plot.fitInView(QRectF(0,0,1000,1000))
        self.folder_choose_edit.setText(fname)

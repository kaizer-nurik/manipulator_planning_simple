from PySide6.QtWidgets import QApplication,QFileDialog,QGraphicsScene
from PySide6 import QtGui
import pyqtgraph as pg
from PyQt6.QtCore import Qt,pyqtSlot,QTimer, QPoint, QEasingCurve, QRectF
import graphics
from robot_class import Robot_class
from robo_scene import Robo_scene
from obstacles import ObstacleManager, Obstacle
from goal_point import GoalPoint
from xml_maker import to_xml
from xml_parser import read_xml
import numpy as np
from scipy import interpolate
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
        self.reset_scene_btn.clicked.connect(self.reset_scene)
        # self.scene_rect  = QRectF(-100000,-100000,100000,100000)
        self.scene.angle_changed.connect(self.on_joint_angle_change_by_mouse)
        self.reset_btn.clicked.connect(self.reset_animation)
        self.animate_btn.clicked.connect(self.animate_robot)

        self.csv_choose_btn.clicked.connect(self.open_csv_dialog)
        # self.scene.setSceneRect(self.scene_rect)
        # graphics.draw_grid(self.scene,10)
        graphics.draw_robot(self.scene,self.robot)
        self.graphicsView.setSceneRect(-5000, -5000, 10000, 10000)
        self.graphicsView.setScene(self.scene)
        self.graphicsView.centerOn(0,0)
        self.graphicsView.scale(1, -1)
        self.timer = QTimer()
        self.timer.setInterval(20)  # msecs 100 = 1/10th sec
        self.timer.timeout.connect(self.update_anim)
        self.anim_speed_sldr.value()
        self.anim_manual_sldr.valueChanged.connect(self.manual_anim)
        # self.graphicsView.scale(self.plot.geometry().width()/SCENE_MAX_LENGTH,self.plot.geometry().width()/SCENE_MAX_LENGTH)

    def unlock_view(self):
        self.graphicsView.setInteractive(True)
        unlock_brush = QtGui.QBrush(QtGui.QColor(255,255,255))
        self.graphicsView.setBackgroundBrush(unlock_brush)
        
    def lock_view(self):
        self.graphicsView.setInteractive(False)
        lock_brush = QtGui.QBrush(QtGui.QColor(230,230,203))
        self.graphicsView.setBackgroundBrush(lock_brush)
    def animate_robot(self):
        self.anim = np.loadtxt(self.csv_choose_edit.text(),delimiter = ',')
        self.lock_view()
        mymin,mymax = 0,10000
        X = np.linspace(mymin,mymax,self.anim.shape[0])
        Y = np.linspace(np.min(self.anim),np.max(self.anim),self.robot.joint_count)

        x,y = np.meshgrid(Y,X)


        f = interpolate.interp2d(x,y,self.anim,kind='cubic')

        # use linspace so your new range also goes from 0 to 3, with 8 intervals
        Xnew = np.linspace(mymin,mymax,10000)
        Ynew = np.linspace(np.min(self.anim),np.max(self.anim),self.robot.joint_count)

        self.anim_temp = f(Ynew,Xnew)
        print(self.anim_temp.shape)
        self.anim_count = 0
        
        self.anim_manual_sldr.blockSignals(True)
        self.timer.start()


    def update_anim(self):  
        self.robot.set_angles(tuple(self.anim_temp[self.anim_count]))
        
        self.anim_count+=int(1000*self.anim_speed_sldr.value()/100)
        self.anim_manual_sldr.setValue(int(self.anim_count/self.anim_temp.shape[0]*100))
        if self.anim_count >= self.anim_temp.shape[0]:
            self.anim_manual_sldr.blockSignals(False)
            self.timer.stop()
            
    def update_anim_old(self):  
        self.robot.set_angles(tuple(self.anim[self.anim_count]))
        
        self.anim_count+=int(1000*self.anim_speed_sldr.value()/100)
        self.anim_manual_sldr.setValue(int(self.anim_count/self.anim.shape[0]*100))
        if self.anim_count >= self.anim.shape[0]:
            self.anim_manual_sldr.blockSignals(False)
            self.timer.stop()
            
    @pyqtSlot(int)
    def manual_anim(self,value):
        self.robot.set_angles(tuple(self.anim[int((self.anim.shape[0]-1)*value/100)]))
        
    @pyqtSlot()
    def reset_animation(self):
        self.unlock_view()
        self.robot.reset_animation()
        

    @pyqtSlot()
    def reset_scene(self):
        self.robot.reset()
        self.obstacles.reset()
        self.goal_point.reset()
        self.scene.reset()
        self.joint_length_line_edit.blockSignals(True)
        self.joint_length_line_edit.setText('100')
        self.joint_length_line_edit.blockSignals(False)
        self.left_limit_text.blockSignals(True)
        self.left_limit_text.setText('-180')
        self.left_limit_text.blockSignals(False)
        self.right_limit_text.blockSignals(True)
        self.right_limit_text.setText('180')
        self.right_limit_text.blockSignals(False)
        self.start_angle_text.blockSignals(True)
        self.start_angle_text.setText('0')
        self.start_angle_text.blockSignals(False)
        self.robot_change_j_spin.blockSignals(True)
        self.robot_change_j_spin.setValue(1)
        self.robot_change_j_spin.blockSignals(False)
        self.robot_joint_count_spin.blockSignals(True)
        self.robot_joint_count_spin.setValue(1)
        self.robot_joint_count_spin.blockSignals(False)
        
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
        self.graphicsView.setTransform(QtGui.QTransform(value/100, 0,0, -value/100, tr.dx(), tr.dy()))
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
        try:
            read_xml(filename=fname[0], 
                    robot=self.robot, 
                    obstacles=self.obstacles, 
                    goal_point=self.goal_point)
            value = 1
            self.robot_joint_count_spin.blockSignals(True)
            self.robot_joint_count_spin.setValue(self.robot.joint_count)
            self.robot_joint_count_spin.blockSignals(False)
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
        except BaseException as e:
            print(e)
    @pyqtSlot()
    def open_csv_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог открытия файла и записывает имя в соотвествующий виджет
        """
        fname = QFileDialog.getOpenFileName(
            self,
            "Открыть",
            "",
            "CSV (*.csv);; All Files (*)",
        )
        #self.plot.fitInView(QRectF(0,0,1000,1000))
        self.csv_choose_edit.setText(fname[0])



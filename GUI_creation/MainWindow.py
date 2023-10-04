from PySide6.QtWidgets import QApplication, QFileDialog, QGraphicsScene, QMainWindow
from PySide6 import QtGui
from PySide6.QtCore import Qt, QTimer, QPoint, QEasingCurve, QRectF
import graphics
from robot_class import Robot_class
from robo_scene import Robo_scene
from obstacles import ObstacleManager, Obstacle
from goal_point import GoalPoint
from xml_maker import to_xml
from xml_parser import read_xml
import numpy as np
# from scipy import interpolate
from GUI import Ui_MainWindow
import math


class MainWindow(QMainWindow, Ui_MainWindow):  # класс окна
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # подключение к кнопке "Обзор" выбора файла
        self.scene = Robo_scene()

        self.current_joint = 0
        self.robot = Robot_class()
        self.robot.change_joint_number(1)
        self.obstacles = ObstacleManager(
            None, None, None, self.scene)
        self.goal_point = GoalPoint(self.scene)

        self.folder_choose_btn.clicked.connect(self.open_folder_dialog)
        self.goal_point_delta_edit.textChanged.connect(
            self.change_goal_point_radius)
        self.create_poli_btn.clicked.connect(self.create_poli)
        self.file_choose_btn.clicked.connect(self.open_file_dialog)
        self.plot = self.graphicsView  # для удобства
        self.robot_joint_count_spin.valueChanged.connect(
            self.change_robot_joints_count)
        self.robot_change_j_spin.valueChanged.connect(
            self.change_robot_joints_change)
        self.joint_length_line_edit.textChanged.connect(
            self.change_joint_length)
        self.joint_length_line_edit.setText('100')
        self.left_limit_text.textChanged.connect(self.change_joint_left_limit)
        self.left_limit_text.setText('-180')
        self.right_limit_text.textChanged.connect(
            self.change_joint_right_limit)
        self.right_limit_text.setText('180')
        self.start_angle_text.textChanged.connect(
            self.change_joint_start_angle)
        self.start_angle_text.setText('0')
        self.zoom_slider.valueChanged.connect(self.ZoomSliderChange)
        self.create_goal_point_btn.clicked.connect(self.goal_point_connect)
        self.export_btn.clicked.connect(self.export_xml)
        self.reset_scene_btn.clicked.connect(self.reset_scene)
        # self.scene_rect  = QRectF(-100000,-100000,100000,100000)
        self.scene.angle_changed.connect(self.on_joint_angle_change_by_mouse)
        self.reset_btn.clicked.connect(self.reset_animation)
        self.animate_btn.clicked.connect(self.animate_robot)
        self.create_goal_point_btn_pos.clicked.connect(
            self.create_goal_point_by_pos)

        self.csv_choose_btn.clicked.connect(self.open_csv_dialog)
        # self.scene.setSceneRect(self.scene_rect)
        # graphics.draw_grid(self.scene,10)
        graphics.draw_robot(self.scene, self.robot)
        self.graphicsView.setSceneRect(-5000, -5000, 10000, 10000)
        self.graphicsView.setScene(self.scene)
        self.graphicsView.centerOn(0, 0)
        self.graphicsView.scale(1, -1)
        self.timer = QTimer()
        self.timer.setInterval(20)  # msecs 100 = 1/10th sec
        self.timer.timeout.connect(self.update_anim)
        self.anim_speed_sldr.value()
        self.anim_manual_sldr.valueChanged.connect(self.manual_anim)
        # self.graphicsView.scale(self.plot.geometry().width()/SCENE_MAX_LENGTH,self.plot.geometry().width()/SCENE_MAX_LENGTH)

        self.create_dataset_btn.clicked.connect(self.build_IK)

    def build_IK(self):
        IK_res = np.loadtxt(self.folder_choose_edit.text(),delimiter=',')
        print(IK_res)
        self.robot.set_angles(list(IK_res[0])[:self.robot.joint_count])
        self.IK_robots = []
        
        for ik in IK_res[::10]:
            # np.loadtxt(csv.split('\n'), delimiter=',')
            self.IK_robots.append(self.robot.copy())
            self.IK_robots[-1].set_angles(ik[:self.robot.joint_count])
            graphics.draw_robot(self.scene, self.IK_robots[-1])



    def unlock_view(self):
        self.graphicsView.setInteractive(True)
        unlock_brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        self.graphicsView.setBackgroundBrush(unlock_brush)
        self.graphicsView.setScene(self.scene)

    def lock_view(self):
        self.graphicsView.setInteractive(False)
        lock_brush = QtGui.QBrush(QtGui.QColor(230, 230, 203))
        self.graphicsView.setBackgroundBrush(lock_brush)

    def animate_robot(self):
        self.lock_view()
        self.anim_scene = Robo_scene()

        self.current_joint = 0
        self.anim_robot = Robot_class()
        self.anim_robot.change_joint_number(1)
        self.anim_obstacles = ObstacleManager(
            None, None, None, self.anim_scene)
        
        self.anim_goal_point = GoalPoint(self.anim_scene)
        graphics.draw_robot(self.anim_scene, self.anim_robot)
        self.graphicsView.setScene(self.anim_scene)
        csv = read_xml(filename=self.csv_choose_edit.text(),
                       robot=self.anim_robot,
                       obstacles=self.anim_obstacles,
                       goal_point=self.anim_goal_point,
                       csv=True)
        value = 1
        self.robot_joint_count_spin.blockSignals(True)
        self.robot_joint_count_spin.setValue(self.anim_robot.joint_count)
        self.robot_joint_count_spin.blockSignals(False)
        self.joint_length_line_edit.blockSignals(True)
        self.joint_length_line_edit.setText(
            str(self.anim_robot[value-1].length))
        self.joint_length_line_edit.blockSignals(False)
        self.left_limit_text.blockSignals(True)
        self.left_limit_text.setText(
            f"{self.anim_robot[value-1].left_angle:.2f}")
        self.left_limit_text.blockSignals(False)
        self.right_limit_text.blockSignals(True)
        self.right_limit_text.setText(
            f"{self.anim_robot[value-1].right_angle:.2f}")
        self.right_limit_text.blockSignals(False)
        self.start_angle_text.blockSignals(True)
        self.start_angle_text.setText(
            f"{self.anim_robot[value-1].start_angle:.2f}")
        self.start_angle_text.blockSignals(False)

        self.anim = np.loadtxt(csv.split('\n'), delimiter=',')
        if len(self.anim.shape) == 1:
            self.anim = self.anim.reshape(self.anim.shape[0], 1)
        ANIM_COUNT_INTP = 10000
        axes = []
        for i in range(self.anim.shape[1]):
            axes.append(np.interp(np.linspace(0, 100, ANIM_COUNT_INTP), np.linspace(
                0, 100, self.anim.shape[0]), self.anim[:, i]))
        self.anim_temp = np.vstack(axes).T
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
        self.anim_count = 0

        self.anim_manual_sldr.blockSignals(True)
        self.timer.start()

    def update_anim(self):
        self.anim_robot.set_angles(tuple(self.anim_temp[self.anim_count]))

        self.anim_count += int(1000*self.anim_speed_sldr.value()/100)
        self.anim_manual_sldr.setValue(
            int(self.anim_count/self.anim_temp.shape[0]*100))
        if self.anim_count >= self.anim_temp.shape[0]:
            self.anim_manual_sldr.blockSignals(False)
            self.timer.stop()

    def manual_anim(self, value):
        self.anim_robot.set_angles(
            tuple(self.anim[int((self.anim.shape[0]-1)*value/100)]))

    def reset_animation(self):
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
        self.unlock_view()

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

    def export_xml(self):
        to_xml(self.file_choose_edit.text(), self.robot,
               self.obstacles, self.goal_point)

    def goal_point_connect(self):
        self.goal_point.create_goal_point()

    def create_goal_point_by_pos(self):
        x = 0
        y = 0
        angle = 0
        for joint in self.robot.joints:
            angle += joint.start_angle
            x += joint.length*math.cos(angle*math.pi/180)
            y += joint.length*math.sin(angle*math.pi/180)

        self.goal_point.create_by_coords(x, y, angle)

    def ZoomSliderChange(self, value):
        tr = self.graphicsView.transform()
        self.graphicsView.setTransform(QtGui.QTransform(
            value/100, 0, 0, -value/100, tr.dx(), tr.dy()))
        # self.graphicsView.scale(value/100,value/100)

    def create_poli(self):
        """Слот для кнопки "Создать полигон"
        создает новый полигон. Нажимая на сцену, можно создать его  точки
        """
        self.obstacles.create_obstacle()

    def on_joint_angle_change_by_mouse(self, joint_num, angle):
        self.robot[joint_num-1].update_start_angle(angle)
        if (joint_num-1) == self.current_joint:
            self.start_angle_text.blockSignals(True)
            self.start_angle_text.setText(f"{angle:.2f}")
            self.start_angle_text.blockSignals(False)

    def change_joint_start_angle(self, text):
        try:
            self.robot[self.current_joint].update_start_angle(float(text))
            self.start_angle_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.start_angle_text.setStyleSheet("color: red;")
            print(e)

    def change_joint_right_limit(self, text):
        try:
            self.robot[self.current_joint].update_right_limit(float(text))
            self.right_limit_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.right_limit_text.setStyleSheet("color: red;")
            print(e)

    def change_joint_left_limit(self, text):
        try:
            self.robot[self.current_joint].update_left_limit(float(text))
            self.left_limit_text.setStyleSheet("color: green;")
        except BaseException as e:
            self.left_limit_text.setStyleSheet("color: red;")
            print(e)

    def change_joint_length(self, text):
        try:
            self.robot[self.current_joint].update_length(float(text))

            self.joint_length_line_edit.setStyleSheet("color: green;")
        except BaseException as e:
            self.joint_length_line_edit.setStyleSheet("color: red;")
            print(e)

    def change_goal_point_radius(self, text):
        try:
            self.goal_point.update_radius(float(text))

            self.goal_point_delta_edit.setStyleSheet("color: green;")
        except BaseException as e:
            self.goal_point_delta_edit.setStyleSheet("color: red;")
            print(e)

    def change_robot_joints_change(self, value):
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

    def change_robot_joints_count(self, value):
        """Change joints count according to spinbox

        Args:
            value (int): spinbox value
        """
        self.robot.change_joint_number(value)
        self.robot_change_j_spin.setMaximum(value)

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

        # self.plot.fitInView(QRectF(0,0,1000,1000))
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
            self.joint_length_line_edit.setText(
                str(self.robot[value-1].length))
            self.joint_length_line_edit.blockSignals(False)
            self.left_limit_text.blockSignals(True)
            self.left_limit_text.setText(
                f"{self.robot[value-1].left_angle:.2f}")
            self.left_limit_text.blockSignals(False)
            self.right_limit_text.blockSignals(True)
            self.right_limit_text.setText(
                f"{self.robot[value-1].right_angle:.2f}")
            self.right_limit_text.blockSignals(False)
            self.start_angle_text.blockSignals(True)
            self.start_angle_text.setText(
                f"{self.robot[value-1].start_angle:.2f}")
            self.start_angle_text.blockSignals(False)
        except BaseException as e:
            print(e)

    def open_csv_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог открытия файла и записывает имя в соотвествующий виджет
        """
        fname = QFileDialog.getOpenFileName(
            self,
            "Открыть",
            "",
            "XML (*.xml);; All Files (*)",
        )
        # self.plot.fitInView(QRectF(0,0,1000,1000))
        self.csv_choose_edit.setText(fname[0])

    def open_folder_dialog(self):
        """Слот для кнопки "Обзор"
        открывает диалог открытия папки датасета
        """
        fname = QFileDialog.getOpenFileName(
            self,
            "Открыть",
            "",
            "csv (*.csv);; All Files (*)",
        )
        # self.plot.fitInView(QRectF(0,0,1000,1000))
        self.folder_choose_edit.setText(fname[0])

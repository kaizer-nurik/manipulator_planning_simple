# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'GUI.ui'
##
## Created by: Qt User Interface Compiler version 6.6.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractScrollArea, QApplication, QGraphicsView, QLabel,
    QLineEdit, QMainWindow, QMenuBar, QPushButton,
    QSizePolicy, QSlider, QSpinBox, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1114, 843)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.file_choose_btn = QPushButton(self.centralwidget)
        self.file_choose_btn.setObjectName(u"file_choose_btn")
        self.file_choose_btn.setGeometry(QRect(300, 60, 93, 28))
        self.graphicsView = QGraphicsView(self.centralwidget)
        self.graphicsView.setObjectName(u"graphicsView")
        self.graphicsView.setGeometry(QRect(400, 160, 600, 600))
        self.graphicsView.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.graphicsView.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.graphicsView.setSizeAdjustPolicy(QAbstractScrollArea.AdjustIgnored)
        self.graphicsView.setDragMode(QGraphicsView.NoDrag)
        self.graphicsView.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.graphicsView.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 90, 47, 13))
        self.file_choose_edit = QLineEdit(self.centralwidget)
        self.file_choose_edit.setObjectName(u"file_choose_edit")
        self.file_choose_edit.setGeometry(QRect(30, 20, 481, 21))
        self.file_choose_edit.setReadOnly(False)
        self.robot_joint_count_spin = QSpinBox(self.centralwidget)
        self.robot_joint_count_spin.setObjectName(u"robot_joint_count_spin")
        self.robot_joint_count_spin.setGeometry(QRect(640, 120, 42, 22))
        self.robot_joint_count_spin.setMinimum(1)
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(600, 90, 111, 20))
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(530, 30, 211, 21))
        self.robot_change_j_spin = QSpinBox(self.centralwidget)
        self.robot_change_j_spin.setObjectName(u"robot_change_j_spin")
        self.robot_change_j_spin.setGeometry(QRect(640, 60, 41, 21))
        self.robot_change_j_spin.setMinimum(1)
        self.robot_change_j_spin.setMaximum(1)
        self.joint_length_line_edit = QLineEdit(self.centralwidget)
        self.joint_length_line_edit.setObjectName(u"joint_length_line_edit")
        self.joint_length_line_edit.setGeometry(QRect(740, 60, 61, 20))
        self.left_limit_text = QLineEdit(self.centralwidget)
        self.left_limit_text.setObjectName(u"left_limit_text")
        self.left_limit_text.setGeometry(QRect(830, 30, 61, 20))
        self.right_limit_text = QLineEdit(self.centralwidget)
        self.right_limit_text.setObjectName(u"right_limit_text")
        self.right_limit_text.setGeometry(QRect(830, 120, 61, 20))
        self.label_12 = QLabel(self.centralwidget)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(740, 40, 47, 13))
        self.label_13 = QLabel(self.centralwidget)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(820, 10, 121, 16))
        self.label_14 = QLabel(self.centralwidget)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(820, 100, 121, 16))
        self.start_angle_text = QLineEdit(self.centralwidget)
        self.start_angle_text.setObjectName(u"start_angle_text")
        self.start_angle_text.setGeometry(QRect(830, 80, 61, 20))
        self.label_15 = QLabel(self.centralwidget)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(820, 60, 121, 16))
        self.export_btn = QPushButton(self.centralwidget)
        self.export_btn.setObjectName(u"export_btn")
        self.export_btn.setGeometry(QRect(140, 60, 93, 28))
        self.create_poli_btn = QPushButton(self.centralwidget)
        self.create_poli_btn.setObjectName(u"create_poli_btn")
        self.create_poli_btn.setGeometry(QRect(190, 230, 141, 28))
        self.zoom_slider = QSlider(self.centralwidget)
        self.zoom_slider.setObjectName(u"zoom_slider")
        self.zoom_slider.setGeometry(QRect(400, 130, 160, 22))
        self.zoom_slider.setMinimum(10)
        self.zoom_slider.setMaximum(200)
        self.zoom_slider.setPageStep(1)
        self.zoom_slider.setValue(100)
        self.zoom_slider.setOrientation(Qt.Horizontal)
        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(410, 110, 71, 16))
        self.create_goal_point_btn = QPushButton(self.centralwidget)
        self.create_goal_point_btn.setObjectName(u"create_goal_point_btn")
        self.create_goal_point_btn.setGeometry(QRect(170, 190, 181, 28))
        self.reset_scene_btn = QPushButton(self.centralwidget)
        self.reset_scene_btn.setObjectName(u"reset_scene_btn")
        self.reset_scene_btn.setGeometry(QRect(200, 270, 131, 28))
        self.csv_choose_edit = QLineEdit(self.centralwidget)
        self.csv_choose_edit.setObjectName(u"csv_choose_edit")
        self.csv_choose_edit.setGeometry(QRect(10, 510, 381, 21))
        self.csv_choose_edit.setReadOnly(False)
        self.csv_choose_btn = QPushButton(self.centralwidget)
        self.csv_choose_btn.setObjectName(u"csv_choose_btn")
        self.csv_choose_btn.setGeometry(QRect(280, 550, 93, 28))
        self.animate_btn = QPushButton(self.centralwidget)
        self.animate_btn.setObjectName(u"animate_btn")
        self.animate_btn.setGeometry(QRect(10, 550, 131, 28))
        self.anim_speed_sldr = QSlider(self.centralwidget)
        self.anim_speed_sldr.setObjectName(u"anim_speed_sldr")
        self.anim_speed_sldr.setGeometry(QRect(90, 620, 160, 18))
        self.anim_speed_sldr.setMinimum(10)
        self.anim_speed_sldr.setMaximum(200)
        self.anim_speed_sldr.setOrientation(Qt.Horizontal)
        self.label_5 = QLabel(self.centralwidget)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(90, 590, 63, 20))
        self.reset_btn = QPushButton(self.centralwidget)
        self.reset_btn.setObjectName(u"reset_btn")
        self.reset_btn.setGeometry(QRect(160, 550, 93, 28))
        self.anim_manual_sldr = QSlider(self.centralwidget)
        self.anim_manual_sldr.setObjectName(u"anim_manual_sldr")
        self.anim_manual_sldr.setGeometry(QRect(30, 770, 1061, 20))
        self.anim_manual_sldr.setMinimum(0)
        self.anim_manual_sldr.setMaximum(100)
        self.anim_manual_sldr.setOrientation(Qt.Horizontal)
        self.label_6 = QLabel(self.centralwidget)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(30, 750, 111, 16))
        self.create_goal_point_btn_pos = QPushButton(self.centralwidget)
        self.create_goal_point_btn_pos.setObjectName(u"create_goal_point_btn_pos")
        self.create_goal_point_btn_pos.setGeometry(QRect(130, 150, 261, 28))
        self.folder_choose_edit = QLineEdit(self.centralwidget)
        self.folder_choose_edit.setObjectName(u"folder_choose_edit")
        self.folder_choose_edit.setGeometry(QRect(10, 370, 381, 21))
        self.folder_choose_edit.setReadOnly(False)
        self.create_dataset_btn = QPushButton(self.centralwidget)
        self.create_dataset_btn.setObjectName(u"create_dataset_btn")
        self.create_dataset_btn.setGeometry(QRect(10, 410, 131, 28))
        self.folder_choose_btn = QPushButton(self.centralwidget)
        self.folder_choose_btn.setObjectName(u"folder_choose_btn")
        self.folder_choose_btn.setGeometry(QRect(280, 410, 93, 28))
        self.label_7 = QLabel(self.centralwidget)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(10, 310, 111, 20))
        self.dataset_dot_spin = QSpinBox(self.centralwidget)
        self.dataset_dot_spin.setObjectName(u"dataset_dot_spin")
        self.dataset_dot_spin.setGeometry(QRect(10, 340, 91, 22))
        self.dataset_dot_spin.setMinimum(1)
        self.dataset_dot_spin.setMaximum(1000000)
        self.label_16 = QLabel(self.centralwidget)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(10, 205, 201, 41))
        self.goal_point_delta_edit = QLineEdit(self.centralwidget)
        self.goal_point_delta_edit.setObjectName(u"goal_point_delta_edit")
        self.goal_point_delta_edit.setGeometry(QRect(20, 250, 61, 21))
        self.path_pdf_btn = QPushButton(self.centralwidget)
        self.path_pdf_btn.setObjectName(u"path_pdf_btn")
        self.path_pdf_btn.setGeometry(QRect(60, 650, 151, 28))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1114, 26))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.file_choose_btn.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u0437\u043e\u0440", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u044a\u0435\u043a\u0442\u044b", None))
        self.file_choose_edit.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u041f\u0443\u0442\u044c \u0434\u043e \u0444\u0430\u0439\u043b\u0430", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u041a\u043e\u043b\u0438\u0447\u0435\u0441\u0442\u0432\u043e \u0437\u0432\u0435\u043d\u044c\u0435\u0432", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u2116 \u0440\u0435\u0434\u0430\u043a\u0442\u0438\u0440\u0443\u0435\u043c\u043e\u0433\u043e \u0437\u0432\u0435\u043d\u0430", None))
        self.right_limit_text.setText("")
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u0414\u043b\u0438\u043d\u0430", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u041b\u0435\u0432\u043e\u0435 \u043e\u0433\u0440\u0430\u043d\u0438\u0447\u0435\u043d\u0438\u0435", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"\u041f\u0440\u0430\u0432\u043e\u0435 \u043e\u0433\u0440\u0430\u043d\u0438\u0447\u0435\u043d\u0438\u0435", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"\u041d\u0430\u0447\u0430\u043b\u044c\u043d\u044b\u0439 \u0443\u0433\u043e\u043b", None))
        self.export_btn.setText(QCoreApplication.translate("MainWindow", u"\u042d\u041a\u0421\u041f\u041e\u0420\u0422", None))
        self.create_poli_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u043f\u043e\u043b\u0438\u0433\u043e\u043d", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u041c\u0430\u0441\u0448\u0442\u0430\u0431", None))
        self.create_goal_point_btn.setText(QCoreApplication.translate("MainWindow", u"\u0417\u0430\u0434\u0430\u0442\u044c \u043a\u043e\u043d\u0435\u0447\u043d\u0443\u044e \u0442\u043e\u0447\u043a\u0443", None))
        self.reset_scene_btn.setText(QCoreApplication.translate("MainWindow", u"C\u0431\u0440\u043e\u0441\u0438\u0442\u044c \u0441\u0446\u0435\u043d\u0443", None))
        self.csv_choose_edit.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u041f\u0443\u0442\u044c \u0434\u043e \u0444\u0430\u0439\u043b\u0430", None))
        self.csv_choose_btn.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u0437\u043e\u0440", None))
        self.animate_btn.setText(QCoreApplication.translate("MainWindow", u"\u0410\u043d\u0438\u043c\u0438\u0440\u043e\u0432\u0430\u0442\u044c", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043a\u043e\u0440\u043e\u0441\u0442\u044c", None))
        self.reset_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"\u0420\u0443\u0447\u043d\u0430\u044f \u0430\u043d\u0438\u043c\u0430\u0446\u0438\u044f", None))
        self.create_goal_point_btn_pos.setText(QCoreApplication.translate("MainWindow", u"\u0417\u0430\u0434\u0430\u0442\u044c \u043a\u043e\u043d\u0435\u0447\u043d\u0443\u044e \u0442\u043e\u0447\u043a\u0443 \u043f\u043e \u043f\u043e\u043b\u043e\u0436\u0435\u043d\u0438\u044e", None))
        self.folder_choose_edit.setText("")
        self.folder_choose_edit.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u041f\u0443\u0442\u044c \u0434\u043e \u043f\u0430\u043f\u043a\u0438 \u0434\u0430\u0442\u0430\u0441\u0435\u0442\u0430", None))
        self.create_dataset_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c", None))
        self.folder_choose_btn.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u0437\u043e\u0440", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u041a\u043e\u043b\u0438\u0447\u0435\u0441\u0442\u0432\u043e \u0442\u043e\u0447\u0435\u043a", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>\u0420\u0430\u0434\u0438\u0443\u0441 \u043f\u043e\u0433\u0440\u0435\u0448\u043d\u043e\u0441\u0442\u0438</p><p>\u043a\u043e\u043d\u0435\u0447\u043d\u043e\u0439 \u0442\u043e\u0447\u043a\u0438</p></body></html>", None))
        self.goal_point_delta_edit.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.path_pdf_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0445\u0440\u0430\u043d\u0438\u0442\u044c \u043f\u0443\u0442\u044c \u0432 \u043f\u0434\u0444", None))
    # retranslateUi


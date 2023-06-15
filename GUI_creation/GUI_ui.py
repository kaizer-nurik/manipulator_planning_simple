# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'GUI.ui'
##
## Created by: Qt User Interface Compiler version 6.4.2
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
    QLineEdit, QListView, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QSlider, QSpinBox,
    QStatusBar, QWidget)

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
        self.poly_list = QListView(self.centralwidget)
        self.poly_list.setObjectName(u"poly_list")
        self.poly_list.setGeometry(QRect(10, 130, 161, 341))
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
        self.label_3.setGeometry(QRect(590, 29, 131, 21))
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
        self.label_13.setGeometry(QRect(820, 10, 111, 16))
        self.label_14 = QLabel(self.centralwidget)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(820, 100, 111, 16))
        self.start_angle_text = QLineEdit(self.centralwidget)
        self.start_angle_text.setObjectName(u"start_angle_text")
        self.start_angle_text.setGeometry(QRect(830, 80, 61, 20))
        self.label_15 = QLabel(self.centralwidget)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(820, 60, 111, 16))
        self.pushButton_5 = QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(140, 60, 93, 28))
        self.create_poli_btn = QPushButton(self.centralwidget)
        self.create_poli_btn.setObjectName(u"create_poli_btn")
        self.create_poli_btn.setGeometry(QRect(230, 170, 111, 28))
        self.poli_x_text = QLineEdit(self.centralwidget)
        self.poli_x_text.setObjectName(u"poli_x_text")
        self.poli_x_text.setGeometry(QRect(200, 270, 113, 20))
        self.poli_y_text = QLineEdit(self.centralwidget)
        self.poli_y_text.setObjectName(u"poli_y_text")
        self.poli_y_text.setGeometry(QRect(200, 310, 113, 20))
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
        self.label_4.setGeometry(QRect(410, 110, 51, 16))
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
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"\u042d\u041a\u0421\u041f\u041e\u0420\u0422", None))
        self.create_poli_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u043f\u043e\u043b\u0438\u0433\u043e\u043d", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u041c\u0430\u0441\u0448\u0442\u0430\u0431", None))
    # retranslateUi


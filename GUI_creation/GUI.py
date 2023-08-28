# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'GUI.ui'
##
## Created by: Qt User Interface Compiler version 6.5.2
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
from PySide6.QtWidgets import (QAbstractScrollArea, QApplication, QCheckBox, QGraphicsView,
    QLabel, QLineEdit, QMainWindow, QMenuBar,
    QPushButton, QRadioButton, QSizePolicy, QSlider,
    QSpinBox, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1172, 715)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.graphicsView = QGraphicsView(self.centralwidget)
        self.graphicsView.setObjectName(u"graphicsView")
        self.graphicsView.setGeometry(QRect(400, 60, 600, 600))
        self.graphicsView.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.graphicsView.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.graphicsView.setSizeAdjustPolicy(QAbstractScrollArea.AdjustIgnored)
        self.graphicsView.setDragMode(QGraphicsView.NoDrag)
        self.graphicsView.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.graphicsView.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.zoom_slider = QSlider(self.centralwidget)
        self.zoom_slider.setObjectName(u"zoom_slider")
        self.zoom_slider.setGeometry(QRect(390, 30, 160, 22))
        self.zoom_slider.setMinimum(10)
        self.zoom_slider.setMaximum(200)
        self.zoom_slider.setPageStep(1)
        self.zoom_slider.setValue(100)
        self.zoom_slider.setOrientation(Qt.Horizontal)
        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(400, 10, 71, 16))
        self.csv_choose_edit = QLineEdit(self.centralwidget)
        self.csv_choose_edit.setObjectName(u"csv_choose_edit")
        self.csv_choose_edit.setGeometry(QRect(10, 330, 381, 21))
        self.csv_choose_edit.setReadOnly(False)
        self.csv_choose_btn = QPushButton(self.centralwidget)
        self.csv_choose_btn.setObjectName(u"csv_choose_btn")
        self.csv_choose_btn.setGeometry(QRect(280, 370, 93, 28))
        self.animate_btn = QPushButton(self.centralwidget)
        self.animate_btn.setObjectName(u"animate_btn")
        self.animate_btn.setGeometry(QRect(10, 370, 131, 28))
        self.anim_speed_sldr = QSlider(self.centralwidget)
        self.anim_speed_sldr.setObjectName(u"anim_speed_sldr")
        self.anim_speed_sldr.setGeometry(QRect(90, 440, 160, 18))
        self.anim_speed_sldr.setMinimum(10)
        self.anim_speed_sldr.setMaximum(200)
        self.anim_speed_sldr.setOrientation(Qt.Horizontal)
        self.label_5 = QLabel(self.centralwidget)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(90, 410, 63, 20))
        self.reset_btn = QPushButton(self.centralwidget)
        self.reset_btn.setObjectName(u"reset_btn")
        self.reset_btn.setGeometry(QRect(160, 370, 93, 28))
        self.folder_choose_edit = QLineEdit(self.centralwidget)
        self.folder_choose_edit.setObjectName(u"folder_choose_edit")
        self.folder_choose_edit.setGeometry(QRect(10, 190, 381, 21))
        self.folder_choose_edit.setReadOnly(False)
        self.process_dataset_btn = QPushButton(self.centralwidget)
        self.process_dataset_btn.setObjectName(u"process_dataset_btn")
        self.process_dataset_btn.setGeometry(QRect(10, 230, 131, 28))
        self.folder_choose_btn = QPushButton(self.centralwidget)
        self.folder_choose_btn.setObjectName(u"folder_choose_btn")
        self.folder_choose_btn.setGeometry(QRect(280, 230, 93, 28))
        self.label_7 = QLabel(self.centralwidget)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(10, 130, 171, 20))
        self.dataset_dot_spin = QSpinBox(self.centralwidget)
        self.dataset_dot_spin.setObjectName(u"dataset_dot_spin")
        self.dataset_dot_spin.setGeometry(QRect(10, 160, 91, 22))
        self.dataset_dot_spin.setMinimum(1)
        self.dataset_dot_spin.setMaximum(1000000)
        self.dataset_dot_spin.setValue(1000)
        self.label_16 = QLabel(self.centralwidget)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(10, 25, 201, 41))
        self.goal_point_delta_edit = QLineEdit(self.centralwidget)
        self.goal_point_delta_edit.setObjectName(u"goal_point_delta_edit")
        self.goal_point_delta_edit.setGeometry(QRect(20, 70, 61, 21))
        self.heatbarView = QGraphicsView(self.centralwidget)
        self.heatbarView.setObjectName(u"heatbarView")
        self.heatbarView.setGeometry(QRect(1010, 60, 131, 601))
        self.heatbarView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.heatbarView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.heatbarView.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.radioButton = QRadioButton(self.centralwidget)
        self.radioButton.setObjectName(u"radioButton")
        self.radioButton.setGeometry(QRect(200, 60, 95, 20))
        self.radioButton.setChecked(True)
        self.radioButton_2 = QRadioButton(self.centralwidget)
        self.radioButton_2.setObjectName(u"radioButton_2")
        self.radioButton_2.setGeometry(QRect(200, 90, 95, 20))
        self.log_scale_checkBox = QCheckBox(self.centralwidget)
        self.log_scale_checkBox.setObjectName(u"log_scale_checkBox")
        self.log_scale_checkBox.setGeometry(QRect(120, 160, 191, 20))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1172, 26))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u041c\u0430\u0441\u0448\u0442\u0430\u0431", None))
        self.csv_choose_edit.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u041f\u0443\u0442\u044c \u0434\u043e \u0444\u0430\u0439\u043b\u0430", None))
        self.csv_choose_btn.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u0437\u043e\u0440", None))
        self.animate_btn.setText(QCoreApplication.translate("MainWindow", u"\u0410\u043d\u0438\u043c\u0438\u0440\u043e\u0432\u0430\u0442\u044c", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043a\u043e\u0440\u043e\u0441\u0442\u044c", None))
        self.reset_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u0431\u0440\u043e\u0441\u0438\u0442\u044c", None))
        self.folder_choose_edit.setText("")
        self.folder_choose_edit.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u041f\u0443\u0442\u044c \u0434\u043e \u043f\u0430\u043f\u043a\u0438 \u0434\u0430\u0442\u0430\u0441\u0435\u0442\u0430", None))
        self.process_dataset_btn.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c", None))
        self.folder_choose_btn.setText(QCoreApplication.translate("MainWindow", u"\u041e\u0431\u0437\u043e\u0440", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u041c\u0430\u043a\u0441 \u0417\u043d\u0430\u0447\u0435\u043d\u0438\u0435 \u0421\u043b\u043e\u0436\u043d\u043e\u0441\u0442\u0438", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>\u0420\u0430\u0434\u0438\u0443\u0441 \u043f\u043e\u0433\u0440\u0435\u0448\u043d\u043e\u0441\u0442\u0438</p><p>\u043a\u043e\u043d\u0435\u0447\u043d\u043e\u0439 \u0442\u043e\u0447\u043a\u0438</p></body></html>", None))
        self.goal_point_delta_edit.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.radioButton.setText(QCoreApplication.translate("MainWindow", u"\u0410*", None))
        self.radioButton_2.setText(QCoreApplication.translate("MainWindow", u"RRT", None))
        self.log_scale_checkBox.setText(QCoreApplication.translate("MainWindow", u"\u041b\u043e\u0433\u0430\u0440\u0438\u0444\u043c\u0438\u0447\u0435\u0441\u043a\u0438\u0439 \u043c\u0430\u0441\u0448\u0442\u0430\u0431", None))
    # retranslateUi


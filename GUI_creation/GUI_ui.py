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
from PySide6.QtWidgets import (QAbstractScrollArea, QApplication, QGraphicsView, QGridLayout,
    QLabel, QLineEdit, QListView, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QSpinBox,
    QStatusBar, QVBoxLayout, QWidget)

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
        self.graphicsView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.graphicsView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.graphicsView.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.listView = QListView(self.centralwidget)
        self.listView.setObjectName(u"listView")
        self.listView.setGeometry(QRect(10, 130, 161, 341))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 90, 47, 13))
        self.file_choose_edit = QLineEdit(self.centralwidget)
        self.file_choose_edit.setObjectName(u"file_choose_edit")
        self.file_choose_edit.setGeometry(QRect(30, 20, 481, 21))
        self.file_choose_edit.setReadOnly(False)
        self.verticalLayoutWidget = QWidget(self.centralwidget)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(210, 490, 131, 261))
        self.verticalLayout_3 = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.dot1 = QGridLayout()
        self.dot1.setObjectName(u"dot1")
        self.Y1_text = QLineEdit(self.verticalLayoutWidget)
        self.Y1_text.setObjectName(u"Y1_text")

        self.dot1.addWidget(self.Y1_text, 1, 1, 1, 1)

        self.Y1_label = QLabel(self.verticalLayoutWidget)
        self.Y1_label.setObjectName(u"Y1_label")

        self.dot1.addWidget(self.Y1_label, 0, 1, 1, 1)

        self.X1_text = QLineEdit(self.verticalLayoutWidget)
        self.X1_text.setObjectName(u"X1_text")

        self.dot1.addWidget(self.X1_text, 1, 0, 1, 1)

        self.X1_label = QLabel(self.verticalLayoutWidget)
        self.X1_label.setObjectName(u"X1_label")

        self.dot1.addWidget(self.X1_label, 0, 0, 1, 1)


        self.verticalLayout_3.addLayout(self.dot1)

        self.gridLayout_6 = QGridLayout()
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.Y2_text = QLineEdit(self.verticalLayoutWidget)
        self.Y2_text.setObjectName(u"Y2_text")

        self.gridLayout_6.addWidget(self.Y2_text, 1, 1, 1, 1)

        self.X2_label = QLabel(self.verticalLayoutWidget)
        self.X2_label.setObjectName(u"X2_label")

        self.gridLayout_6.addWidget(self.X2_label, 0, 0, 1, 1)

        self.X2_text = QLineEdit(self.verticalLayoutWidget)
        self.X2_text.setObjectName(u"X2_text")

        self.gridLayout_6.addWidget(self.X2_text, 1, 0, 1, 1)

        self.Y2_label = QLabel(self.verticalLayoutWidget)
        self.Y2_label.setObjectName(u"Y2_label")

        self.gridLayout_6.addWidget(self.Y2_label, 0, 1, 1, 1)


        self.verticalLayout_3.addLayout(self.gridLayout_6)

        self.gridLayout_7 = QGridLayout()
        self.gridLayout_7.setObjectName(u"gridLayout_7")
        self.Y3_label = QLabel(self.verticalLayoutWidget)
        self.Y3_label.setObjectName(u"Y3_label")

        self.gridLayout_7.addWidget(self.Y3_label, 0, 1, 1, 1)

        self.X3_label = QLabel(self.verticalLayoutWidget)
        self.X3_label.setObjectName(u"X3_label")

        self.gridLayout_7.addWidget(self.X3_label, 0, 0, 1, 1)

        self.X3_text = QLineEdit(self.verticalLayoutWidget)
        self.X3_text.setObjectName(u"X3_text")

        self.gridLayout_7.addWidget(self.X3_text, 1, 0, 1, 1)

        self.Y3_text = QLineEdit(self.verticalLayoutWidget)
        self.Y3_text.setObjectName(u"Y3_text")

        self.gridLayout_7.addWidget(self.Y3_text, 1, 1, 1, 1)


        self.verticalLayout_3.addLayout(self.gridLayout_7)

        self.gridLayout_8 = QGridLayout()
        self.gridLayout_8.setObjectName(u"gridLayout_8")
        self.X4_label = QLabel(self.verticalLayoutWidget)
        self.X4_label.setObjectName(u"X4_label")

        self.gridLayout_8.addWidget(self.X4_label, 0, 0, 1, 1)

        self.X4_text = QLineEdit(self.verticalLayoutWidget)
        self.X4_text.setObjectName(u"X4_text")

        self.gridLayout_8.addWidget(self.X4_text, 1, 0, 1, 1)

        self.Y4_label = QLabel(self.verticalLayoutWidget)
        self.Y4_label.setObjectName(u"Y4_label")

        self.gridLayout_8.addWidget(self.Y4_label, 0, 1, 1, 1)

        self.Y4_text = QLineEdit(self.verticalLayoutWidget)
        self.Y4_text.setObjectName(u"Y4_text")

        self.gridLayout_8.addWidget(self.Y4_text, 1, 1, 1, 1)


        self.verticalLayout_3.addLayout(self.gridLayout_8)

        self.R_label = QLabel(self.verticalLayoutWidget)
        self.R_label.setObjectName(u"R_label")

        self.verticalLayout_3.addWidget(self.R_label)

        self.R_text = QLineEdit(self.verticalLayoutWidget)
        self.R_text.setObjectName(u"R_text")

        self.verticalLayout_3.addWidget(self.R_text)

        self.verticalLayoutWidget_2 = QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(200, 130, 160, 100))
        self.verticalLayout_4 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.pushButton_2 = QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.verticalLayout_4.addWidget(self.pushButton_2)

        self.pushButton_3 = QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.verticalLayout_4.addWidget(self.pushButton_3)

        self.pushButton_4 = QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.verticalLayout_4.addWidget(self.pushButton_4)

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
        self.lineEdit_3 = QLineEdit(self.centralwidget)
        self.lineEdit_3.setObjectName(u"lineEdit_3")
        self.lineEdit_3.setGeometry(QRect(830, 30, 61, 20))
        self.lineEdit_12 = QLineEdit(self.centralwidget)
        self.lineEdit_12.setObjectName(u"lineEdit_12")
        self.lineEdit_12.setGeometry(QRect(830, 120, 61, 20))
        self.label_12 = QLabel(self.centralwidget)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(740, 40, 47, 13))
        self.label_13 = QLabel(self.centralwidget)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(820, 10, 111, 16))
        self.label_14 = QLabel(self.centralwidget)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(820, 100, 111, 16))
        self.lineEdit_13 = QLineEdit(self.centralwidget)
        self.lineEdit_13.setObjectName(u"lineEdit_13")
        self.lineEdit_13.setGeometry(QRect(830, 80, 61, 20))
        self.label_15 = QLabel(self.centralwidget)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(820, 60, 111, 16))
        self.pushButton_5 = QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(140, 60, 93, 28))
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
        self.Y1_label.setText(QCoreApplication.translate("MainWindow", u"Y1", None))
        self.X1_label.setText(QCoreApplication.translate("MainWindow", u"X1", None))
        self.X2_label.setText(QCoreApplication.translate("MainWindow", u"X2", None))
        self.Y2_label.setText(QCoreApplication.translate("MainWindow", u"Y2", None))
        self.Y3_label.setText(QCoreApplication.translate("MainWindow", u"Y3", None))
        self.X3_label.setText(QCoreApplication.translate("MainWindow", u"X3", None))
        self.X4_label.setText(QCoreApplication.translate("MainWindow", u"X4", None))
        self.Y4_label.setText(QCoreApplication.translate("MainWindow", u"Y4", None))
        self.R_label.setText(QCoreApplication.translate("MainWindow", u"R", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u041a\u0432\u0430\u0434\u0440\u0430\u0442", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u041a\u0440\u0443\u0433", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"\u0421\u043e\u0437\u0434\u0430\u0442\u044c \u043b\u0438\u043d\u0438\u044e", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u041a\u043e\u043b\u0438\u0447\u0435\u0441\u0442\u0432\u043e \u0437\u0432\u0435\u043d\u044c\u0435\u0432", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u2116 \u0440\u0435\u0434\u0430\u043a\u0442\u0438\u0440\u0443\u0435\u043c\u043e\u0433\u043e \u0437\u0432\u0435\u043d\u0430", None))
        self.lineEdit_12.setText("")
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"\u0414\u043b\u0438\u043d\u0430", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u041b\u0435\u0432\u043e\u0435 \u043e\u0433\u0440\u0430\u043d\u0438\u0447\u0435\u043d\u0438\u0435", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"\u041f\u0440\u0430\u0432\u043e\u0435 \u043e\u0433\u0440\u0430\u043d\u0438\u0447\u0435\u043d\u0438\u0435", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"\u041d\u0430\u0447\u0430\u043b\u044c\u043d\u044b\u0439 \u0443\u0433\u043e\u043b", None))
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"\u042d\u041a\u0421\u041f\u041e\u0420\u0422", None))
    # retranslateUi


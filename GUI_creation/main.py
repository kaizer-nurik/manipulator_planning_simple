import sys
from PySide6.QtWidgets import QApplication
import pyqtgraph as pg
from MainWindow import MainWindow



def main():
    """
        Точка входа
    """
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()

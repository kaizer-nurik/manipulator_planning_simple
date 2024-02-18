import sys

from PySide6.QtWidgets import QApplication, QMainWindow
from MainWindow import MainWindow


"""
Processes whole dataset structure. requires special folder structure.
"""

def main():
    """
        Точка входа
    """
    global app 
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()

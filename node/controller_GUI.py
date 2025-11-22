#!/usr/bin/env python3

from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

import sys

class Controller_App(QtWidgets.QMainWindow):

    def __init__(self):
        super(Controller_App, self).__init__()
        loadUi("./Controller_App.ui", self)

        # self.browse_button.clicked.connect(self.SLOT_browse_button)
        # self.toggle_cam_button.clicked.connect(self.SLOT_toggle_camera)

        


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    my_app = Controller_App()
    my_app.show()
    sys.exit(app.exec_())



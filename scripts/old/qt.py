# -*- coding: utf-8 -*-

import pathlib
from PyQt5 import QtCore, QtGui, uic
from PyQt5.QtWidgets import QWidget, QFileDialog, QMainWindow, QApplication
from PyQt5.QtCore import QTimer

## *********************************************************************
def openDialog(folder_path, file_ext):

    if type(folder_path) is not str:
        folder_path = '/home'

    p_path = pathlib.Path(folder_path)
    if p_path.is_dir:
        pass
        
    fname = QFileDialog.getOpenFileName(parent   = None,
                                        caption  = 'Open file',
                                        directory= folder_path,
                                        filter   = file_ext)
    print(fname)





class testPyQt(QMainWindow):
    def __init__(self):
        super(testPyQt, self).__init__()

        folderPath = os.path.dirname(__file__)
        uifilePath = os.path.join(folderPath, "form.ui")
        Ui_MainWindow = uic.loadUiType(uifilePath, self)[0]

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # setup connection
        self.ui.pushButton.clicked.connect(self.onClickButton)

    @QtCore.pyqtSlot()
    def onClickButton(self):
        self.ui.label.setText("Pushed!!")

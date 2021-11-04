#!/usr/bin/python3
# -*- coding: utf-8 -*-
##
# @file test_logger.py
# @brief logger.pyファイルのテスト

import sys
import pathlib
from utils.logger import *
import roslib.packages

class test():
    def __init__(self):
        self.timeStamp=5
        self.a = 1
        self.b = 2
    def __str__(self):
        return "time={0}\na = {1}\nb = {2}".format(self.timeStamp,self.a, self.b)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    print(roslib.packages.get_pkg_dir("moveit_utils"))

    folderPath = pathlib.Path(__file__).parent.__str__()

    path = "/home/ubuntu/catkin_ws/src/python_utils/scripts/a.pickle"

    A = test()
    
    saveData(A, folderPath)

    B = openData()
    print(B)


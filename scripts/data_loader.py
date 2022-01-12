#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
from qt_dialog import *
from simplot import *
import numpy as np

def process(file_path):
    # load data
    header, log_data, file_path = csv_loader(file_path=file_path, row_of_label=2)
    print("[load file] {0}".format(file_path))
    data = splitData(header, log_data, show=False)

    # draw figure
    fig, ax = figure(6,1)
    grid_on(ax)
    
    ax[2][0].axhline(0.45)
    for i in range(6):
        ax[i][0].plot(data["time"], data["a"][i])

    # save figure
    filename, ext = os.path.splitext(file_path)
    fig.savefig(filename + ".png")

if __name__ == '__main__':
    app = QApplication(sys.argv)

    names = getFileNames(folder_path="", file_ext="*.csv", recursive=False)
    for name in names:
        process(name)
    
    exit()
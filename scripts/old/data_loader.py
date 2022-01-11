#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
from qt_dialog import *
from simplot import *
import numpy as np


if __name__ == '__main__':
    #print(sys.path)
    app = QApplication(sys.argv)
    header, data = csv_loader(file_path="", row_of_label=2)
    header = setHeader(header)
    print(header)
    print("-----")
    print(data[0])

    fig, ax = figure(6,1)
    grid_on(ax)
    
    ax[2][0].axhline(0.45)
    for i in range(6):
        ax[i][0].plot(data[0],data[i+1])
    plt.show()
    
    exit()



"""
    data_set = np.loadtxt(
    fname=filepath, #読み込むファイルのパスと名前
    dtype="float", #floatで読み込む
    delimiter=",", #csvなのでカンマで区切る
    skiprows=1,
    unpack=True
    )
    print(data_set)
    print(os.path.expanduser('~'))"""
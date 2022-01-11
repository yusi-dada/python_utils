# -*- coding: utf-8 -*-
##
# @simplot.py
# @brief データの描画操作

from matplotlib import pyplot as plt
import numpy as np

## *********************************************************************
# @brief 図作成
# @param row サブプロット行数
# @param col サブプロット列数
# @param size 図サイズ
# @retval fig 図プロパティ
# @retval ax 軸プロパティ
def figure(row=1, col=1, size=(10,4)):
    fig, ax = plt.subplots(nrows=row, ncols=col, figsize=size) 
    fig.tight_layout()
    ax = np.reshape(ax, [row,col])
    return fig, ax

## *********************************************************************
# @brief グリッドON
# @param ax 軸プロパティ
def grid_on(ax):
    ax = np.reshape(ax, [-1,])
    for i in range(len(ax)):
        ax[i].grid(True)

## *********************************************************************
# @brief データのヘッダを整形
# @details ヘッダに含まれる空白及び[単位]を消去
# @param header [str] ヘッダリスト
# @return 整形後ヘッダリスト
def setHeader(header):
    for i, lbl in enumerate(header):
        tmp = lbl.split("[")
        header[i] = tmp[0].replace(" ","")
    label = None
    for i, lbl in enumerate(header):
        if not lbl=="":
            label = lbl
            idx_of_label = i
            counter = 1
        else:
            if label is not None:
                header[idx_of_label] = label+"_"+str(0)
                header[i] = label+"_"+str(counter)
                counter = counter+1
    return header

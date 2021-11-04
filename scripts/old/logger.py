# -*- coding: utf-8 -*-

import numpy as np
import csv
import copy
import inspect

##
# @brief データログクラス
class logger(dict):
    
    def __init__(self):
        pass

    def info(self):
        ##
        # @brief 取得されたログの情報を表示
        #  
        print("--- show log info ---")
        for key in self.keys():
            row , col = self[key].shape
            print("[{0}] : row={1}, col={2}".format(key, row, col))
        print("---------------------")

    def append(self, key, val):
        ##
        # @brief データの追加
        # @param key データの連想配列キー(str)
        # @paran val データ本体(スカラ、リスト、np.array)
        #
        val = np.atleast_2d(val)
        if key in self.keys():
            self[key] = np.r_[self[key], val]
        else:
            self[key] = val

    def appendClass(self, cls, appendList):
        ##
        # @brief クラスに含まれるプロパティの値をログ
        # @param cls ログを取得するクラスインスタンス
        # @param appendList 取得するプロパティ名リスト
        #
        keys_ = cls.__dict__.keys()
        vals_ = cls.__dict__.values()

        for key, val in zip(keys_, vals_):
            if key in appendList:
                self.append(key, val)

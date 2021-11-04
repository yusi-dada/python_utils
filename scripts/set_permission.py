#!/usr/bin/python3
# -*- coding: utf-8 -*-
##
# @file set_permission.py
# @brief exeフォルダ内のpythonスクリプトに実行権限を付与

from utils.permission import *

if __name__ == '__main__':
    grant_execute_permission(__file__)

    


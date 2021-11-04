# -*- coding: utf-8 -*-
##
# @file permission.py
# @brief pythonファイルの実行権限を設定

import pathlib

## *********************************************************************
# @brief ファイルリストの取得
# @param file_path [str] ファイルが存在するフォルダパス
# @param file_ext [str] 拡張子 (e.g. '*.[Ss][Tt][Ll]')
# @return file_names ファイル名リスト
def get_file_list(file_path, file_ext):
    p_path = pathlib.Path(file_path)
    files  = p_path.glob(file_ext)
    file_names = [p.absolute() for p in files if p.is_file()]
    return file_names

## *********************************************************************
# @brief 実行スクリプトファイルパス内に含まれるpythonスクリプトの権限を変更
# @param filePath (str) 実行スクリプトファイルパス
def grant_execute_permission(filePath):
    folderPath = pathlib.Path(filePath).parent
    list = get_file_list(folderPath, '*.[Pp][Yy]')

    for i, f in enumerate(list):
        if i==0:
            print("change file permission to [rwx rwx r-x]")
            print(" [folder path] {0}".format(list[0].parent))
        pathlib.Path(f).chmod(0b111111101)
        print("   {0} : {1}".format(i,f.name))

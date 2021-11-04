# -*- coding: utf-8 -*-
##
# @file logger.py
# @brief ディレクトリ操作汎用関数

import os, pickle
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QApplication

## *********************************************************************
# @brief ファイルオープンダイアログの生成
# @param folder_path [str] 初期表示ディレクトリパス
# @param file_ext [str] 拡張子
# @return fname 選択ファイルパス
# @note QApplicationを実行しておく必要あり
def openDialog(folder_path, file_ext):
    if not os.path.isdir(folder_path):
        folder_path = os.getcwd()
    options  = QFileDialog.Options()
    options |= QFileDialog.DontUseNativeDialog
    fname, _ = QFileDialog.getOpenFileName(
                parent    = None,
                caption   = 'Open file',
                directory = folder_path,
                filter    = file_ext,
                options   = options)
    return fname

## *********************************************************************
# @brief ファイル保存ダイアログの生成
# @param folder_path [str] 初期表示ディレクトリパス
# @param file_ext [str] 拡張子
# @retval fname 選択ファイルパス
# @retval overwrite 上書き許可フラグ(同名ファイルあればダイアログ中で確認)
# @note QApplicationを実行しておく必要あり
def saveDialog(folder_path, file_ext):
    if not os.path.isdir(folder_path):
        folder_path = os.getcwd()
    options  = QFileDialog.Options()
    options |= QFileDialog.DontUseNativeDialog
    fname, _ = QFileDialog.getSaveFileName(
                parent    = None,
                caption   = 'Save file',
                directory = folder_path,
                filter    = file_ext,
                options   = options)

    overwrite = False
    if fname != "":
        _, ext  = os.path.splitext(fname)
        _, ext_ = os.path.splitext(file_ext)
        if ext == ext_:
            overwrite = True
        else:
            fname = fname + ext_
    return fname, overwrite

## *********************************************************************
# @brief ファイル読込み（pickle利用）
# @param file_path (str) ファイルパス
# @param ext (str) 指定拡張子
# @retval data 読込みデータ
# @retval None (失敗時)
# @note file_pathにフォルダパスを指定するとダイアログの初期パスに設定
def openData(file_path="", ext=".pickle"):
    dialog = False
    _, ext_ = os.path.splitext(file_path)
    if not os.path.isfile(file_path):
        print("[loadData] wrong file directory.")
        dialog = True
    elif ext_ != ext:
        print("[loadData] wrong extension.")
        dialog = True
    
    if dialog:
        file_path = openDialog(file_path, '*'+ext)
        if not os.path.isfile(file_path):
            print("[loadData] canceled.")
            return None
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)
            print("[loadData] success: {0}".format(file_path))
            return data
    except Exception as e:
        print("[loadData] {0}".format(e))
        return None
        
## *********************************************************************
# @brief ファイルへデータ保存（pickle利用）
# @param data 保存データ
# @param file_path (str) ファイルパス
# @param ext (str) 指定拡張子
# @retval True 成功
# @retval False 失敗
# @note file_pathにフォルダパスを指定するとダイアログの初期パスに設定
def saveData(data, file_path="", ext=".pickle"):
    dialog = False
    dirname = os.path.dirname(file_path)
    if not os.path.isdir(dirname):
        print("[saveData] wrong folder directory.")
        dialog = True
    else:
        _, ext_ = os.path.splitext(file_path)
        if ext_ != ext:
            print("[saveData] wrong extension.")
            dialog = True
        else:
            if os.path.isfile(file_path):
                ans = QMessageBox.question(None, "確認", "上書きしますか？", QMessageBox.Ok, QMessageBox.Cancel)
                if ans!=QMessageBox.Ok:
                    print("[saveData] canceled")
                    return False      
    
    if dialog:
        file_path, _ = saveDialog(file_path, '*'+ext)
        if not os.path.isfile(file_path):
            print("[saveData] canceled")
            return False
    try:
        with open(file_path, 'wb') as f:
            pickle.dump(data, f)
            print("[saveData] success: {0}".format(file_path))
            return True
    except Exception as e:
        print("[saveData] {0}".format(e))
        return False


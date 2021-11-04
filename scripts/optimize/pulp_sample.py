#!/usr/bin/python3
# -*- coding: utf-8 -*-

from pulp import *

if __name__ == '__main__':

    # 問題・変数定義
    prob = LpProblem("ex1", LpMinimize)
    x1 = LpVariable("x1", lowBound=0, cat=LpContinuous)
    x2 = LpVariable("x2", lowBound=0, cat=LpContinuous)
    
    # 目的関数
    prob += 2*x1+3*x2

    # 制約式
    prob += 3*x1 + 4*x2 >= 1

    # 定式化確認
    print(prob)


    status = prob.solve()
    print("End status: ", LpStatus[status])
    print("Obj function: ", value(prob.objective))
    for var in prob.variables():
        print(var.name,":",var.varValue)
    

    pass
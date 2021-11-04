#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import cvxpy

if __name__ == '__main__':

    
    # 変数定義
    A = np.array([[3,2],[2,6]])
    b = np.array([5,8])
    c = np.array([2,3])
    x = cvxpy.Variable(2)

    # 目的関数
    objective = cvxpy.Minimize(c.T@x)

    # 制約式
    constraints = [A@x >= b, x>=0]

    # 定式化
    prob = cvxpy.Problem(objective, constraints)

    prob.solve()

    # 結果表示
    print('\nThe optimal value is ', prob.value)
    print('\nA solution x is ', x.value)
    print('\nA dual solution is ', prob.constraints[0].dual_value)

    pass
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time : 2022/3/31 23:22
# @Author : Zhang Chi
# @Email : zhangch287@mail2.sysu.edu.cn
# @File : kinematics.py
# @Software: PyCharm


import numpy as np
import math


def DH_Mat(a, alpha, d, theta):
    """
    DH_Mat
    :param a:
    :param alpha: rad
    :param d:
    :param theta: rad
    :return:
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)
    T = [[cos_theta, -cos_alpha * sin_theta, sin_alpha * sin_theta, a * cos_theta],
         [sin_theta, cos_alpha * cos_theta, -sin_alpha * cos_theta, a * sin_theta],
         [0, sin_alpha, cos_alpha, d],
         [0, 0, 0, 1]]
    return np.array(T)


def Forward_Kinematics(qq, DH_Table):
    """
    Forward_Kinematics
    :param qq:
    :param DH_Table:
    :return:
    """
    Ti = []
    for i, q in enumerate(qq):
        DH_Para = DH_Table[i]
        Tem_Ti = DH_Mat(DH_Para[0], DH_Para[1], DH_Para[2], DH_Para[3] + q)
        if i == 0:
            Ti.append(Tem_Ti)
        else:
            Ti.append(Ti[i - 1] @ Tem_Ti)
    return Ti


def Jacob_Mat_0(Ti):
    """
    Jacob_Mat_0
    :param Ti:
    :return:
    """
    n = len(Ti)
    z = np.zeros((3, n))
    r = np.zeros((3, n))
    crs_zr = np.zeros((3, n))
    for i, T in enumerate(Ti):
        if i == 0:
            z[:, i] = np.array([0, 0, 1]).T
            r[:, i] = Ti[n - 1][0:3, 3]
        else:
            z[:, i] = Ti[i - 1][0:3, 2]
            r[:, i] = Ti[n - 1][0:3, 3] - Ti[i - 1][0:3, 3]
        crs_zr[:, i] = np.cross(z[:, i], r[:, i])
    J0 = np.concatenate((crs_zr, z), axis=0)
    return J0


def Inverse_Kinematics(Td, qq0, DH_Table):
    efs = 1e-10
    qq = qq0
    deltaQ = 1
    lmax = 0
    l_limit = 1000
    LimitFlag = 0

    while deltaQ > efs and LimitFlag == 0:
        Ti = Forward_Kinematics(qq, DH_Table)
        Tn = Ti[-1]
        dp = Td[0:3, 3] - Tn[0:3, 3]
        dR = 0.5 * (np.cross(Tn[0:3, 0], Td[0:3, 0]) + np.cross(Tn[0:3, 1], Td[0:3, 1]) + np.cross(Tn[0:3, 2],
                                                                                                   Td[0:3, 2]))
        dA = np.concatenate((dp.reshape((3, 1)), dR.reshape((3, 1))), axis=0)
        J0 = Jacob_Mat_0(Ti)
        dq = np.linalg.pinv(J0) @ dA
        qq = qq + dq
        deltaQ = np.linalg.norm(dq)
        lmax = lmax + 1
        if lmax > l_limit:
            LimitFlag = 1
            print('Solution wouldn''t converge')

    return qq, LimitFlag


# def InverseTransMat(T):
#     T_inv = np.eye(4)
#     T_inv[0:3, 0:3] = T[0:3, 0:3].T
#     T_inv[0:3, 3] = (T_inv[0:3, 0:3]*).reshape(1, 3)


def EulerZYX2RotMat(zyx):
    c1 = math.cos(zyx[0])
    s1 = math.sin(zyx[0])
    c2 = math.cos(zyx[1])
    s2 = math.sin(zyx[1])
    c3 = math.cos(zyx[2])
    s3 = math.sin(zyx[2])

    R = np.array([[c1 * c2, c1 * s2 * s3 - s1 * c3, c1 * s2 * c3 + s1 * s3],
                  [s1 * c2, s1 * s2 * s3 + c1 * c3, s1 * s2 * c3 - c1 * s3],
                  [-s2, c2 * s3, c2 * c3]])

    return R


def SixDimPose2TransMat(pos, zyx):
    T = np.eye(4)
    T[0:3, 0:3] = EulerZYX2RotMat(zyx)
    T[0:3, 3] = pos.reshape(1, 3)

    return T


if __name__ == '__main__':
    pos_tmp = np.array([10, 20, 30]).reshape(3, 1)
    zyx_tmp = np.array([20, 10, 80]).reshape(3, 1) * math.pi / 180
    R_tmp = EulerZYX2RotMat(zyx_tmp)
    print(R_tmp)
    T_tmp = SixDimPose2TransMat(pos_tmp, zyx_tmp)
    print(T_tmp)

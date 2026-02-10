# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time : 2023/4/20 18:51
# @Author : Zhang Chi
# @Email : zhangch287@mail2.sysu.edu.cn
# @File : Acupuncture_Robot_Software.py
# @Software: PyCharm

import os
import re
import sys
import cv2
import time
import math
import numpy as np
import pyrealsense2 as rs

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer

from CPS import CPSClient
# from ForceSensor import ForceSensor
from kinematics import *
from visual_perception import locate_acupoint

Deg2Rad = math.pi / 180
Rad2Deg = 180 / math.pi


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


class Ui_MainWindow(object):
    def __init__(self):
        np.set_printoptions(suppress=True)

        self.flag_save_image = False

        # ====================================== 相机配置 ===========================================
        self.cam_serial_num_global = '220222067428'
        self.cam_serial_num_handeye = '220322060052'

        self.image_size = (1280, 720)
        self.image_show_size = (int(self.image_size[0] / 2), int(self.image_size[1] / 2))

        self.Lap_fu = 656.0875
        self.Lap_fv = 655.5052
        self.Lap_u0 = 336.9377
        self.Lap_v0 = 359.9387

        self.flag_global_camera_connect = False
        self.flag_handeye_camera_connect = False
        self.flag_get_video_stream = False

        self.global_color_image = None
        self.global_depth_image = None
        self.global_depth_colormap = None
        self.handeye_color_image = None
        self.handeye_depth_image = None
        self.handeye_depth_colormap = None

        self.global_camera_pipeline = rs.pipeline()
        self.global_camera_config = rs.config()
        self.handeye_camera_pipeline = rs.pipeline()
        self.handeye_camera_config = rs.config()

        self.GetVideoStreamTimer = QTimer()
        self.GetVideoStreamTimer.timeout.connect(self.onGetVideoStreamTimer)
        # ====================================== 相机配置 ===========================================

        # ====================================== 机械臂配置 ===========================================
        self.file_saved_robot = os.path.join('Config_Software', 'saved_robot.txt')
        self.flag_robot_connect = False

        self.my_robot = CPSClient()

        self.robot_ip = '192.168.0.10'
        self.robot_port = 10003

        self.robot_dof = 6
        self.robot_speed = 200

        self.joint_angles = [0, 0, 0, 0, 0, 0]
        self.joint_angles_min = [-360, -135, -150, -360, -147, -360]
        self.joint_angles_max = [360, 135, 150, 360, 147, 360]

        # [a, alpha, d, offset]
        self.DH_Table = [[0, math.pi / 2, 192.5, 0],
                         [266, math.pi, 0, math.pi / 2],
                         [0, math.pi / 2, 0, math.pi / 2],
                         [0, -math.pi / 2, 324, 0],
                         [0, math.pi / 2, 0, 0],
                         [0, 0, 155, 0]]

        self.qq0 = np.array([1.55131845, 0.02303835, -1.33037722, -0.03717551, -1.77194552, 0.1460317]).reshape(6, 1)
        # ====================================== 机械臂配置 ===========================================

        # ====================================== 坐标系配置 ===========================================
        self.End_T_Needle = np.array([[1, 0, 0, 0],
                                      [0, 1, 0, 0],
                                      [0, 0, 1, 234],
                                      [0, 0, 0, 1]])
        self.Needle_T_End = np.linalg.inv(self.End_T_Needle)
        # print(self.End_T_Needle)
        # print(self.Needle_T_End)
        # ====================================== 坐标系配置 ===========================================

        # ======================================= 标定配置 ============================================
        self.dir_calib = 'Calibration_Data'
        self.dir_calib_global = os.path.join(self.dir_calib, 'global')
        self.dir_calib_handeye = os.path.join(self.dir_calib, 'handeye')
        self.dir_calib_robot = os.path.join(self.dir_calib, 'robot')

        self.pos_calib0 = np.array([0, -550, 30]).reshape(3, 1)
        self.zyx_calib0 = np.array([90, -160, 0]).reshape(3, 1) * math.pi / 180
        self.Base_T_Needle_Calib0 = SixDimPose2TransMat(self.pos_calib0, self.zyx_calib0)

        self.Base_T_End_Calib0 = self.Base_T_Needle_Calib0 @ self.Needle_T_End
        self.joint_angles_calib0, flag_ik = \
            Inverse_Kinematics(self.Base_T_End_Calib0, self.qq0, self.DH_Table)
        self.joint_angles_calib0 = self.joint_angles_calib0.reshape(1, 6)[0] * Rad2Deg

        print('Base_T_Needle_Calib0:\n', self.Base_T_Needle_Calib0)
        print('Base_T_End_Calib0:\n', self.Base_T_End_Calib0)
        print('joint_angles_calib0:\n', self.joint_angles_calib0)
        # ======================================= 标定配置 ============================================

        # ===================================== 规划与控制配置 ==========================================
        self.joint_angles_plan_and_control0 = np.array([109.931, 26.192, -26.273, -0.006, -118.522, -2.019])

        self.state_plan_and_control = 0
        self.count_plan_and_control = 0

        self.flag_plan_and_control = False
        self.PlanAndControlTimer = QTimer()
        self.PlanAndControlTimer.timeout.connect(self.onPlanAndControlTimer)
        # ===================================== 规划与控制配置 ==========================================

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1900, 980)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        MainWindow.setCentralWidget(self.centralwidget)

        # ============================================ 显示模块 ============================================
        self.GlobalRGBImageShowLabel = QtWidgets.QLabel(self.centralwidget)
        self.GlobalRGBImageShowLabel.setGeometry(
            QtCore.QRect(20, 20, self.image_show_size[0], self.image_show_size[1]))
        self.GlobalRGBImageShowLabel.setObjectName("GlobalRGBImageShowLabel")
        self.GlobalRGBImageShowLabel.setFrameShape(QtWidgets.QFrame.Box)

        self.GlobalDepthImageShowLabel = QtWidgets.QLabel(self.centralwidget)
        self.GlobalDepthImageShowLabel.setGeometry(
            QtCore.QRect(660, 20, self.image_show_size[0], self.image_show_size[1]))
        self.GlobalDepthImageShowLabel.setObjectName("GlobalDepthImageShowLabel")
        self.GlobalDepthImageShowLabel.setFrameShape(QtWidgets.QFrame.Box)

        self.HandeyeRGBImageShowLabel = QtWidgets.QLabel(self.centralwidget)
        self.HandeyeRGBImageShowLabel.setGeometry(
            QtCore.QRect(20, 400, self.image_show_size[0], self.image_show_size[1]))
        self.HandeyeRGBImageShowLabel.setObjectName("HandeyeRGBImageShowLabel")
        self.HandeyeRGBImageShowLabel.setFrameShape(QtWidgets.QFrame.Box)

        self.HandeyeDepthImageShowLabel = QtWidgets.QLabel(self.centralwidget)
        self.HandeyeDepthImageShowLabel.setGeometry(
            QtCore.QRect(660, 400, self.image_show_size[0], self.image_show_size[1]))
        self.HandeyeDepthImageShowLabel.setObjectName("HandeyeDepthImageShowLabel")
        self.HandeyeDepthImageShowLabel.setFrameShape(QtWidgets.QFrame.Box)
        # ============================================ 显示模块 ============================================

        # ========================================== 视觉模块 ==========================================
        self.VisionDebugGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.VisionDebugGroup.setGeometry(QtCore.QRect(1320, 20, 560, 250))
        self.VisionDebugGroup.setObjectName("VisionDebugGroup")

        # =============================== 深度相机 ===============================
        self.CameraGroup = QtWidgets.QGroupBox(self.VisionDebugGroup)
        self.CameraGroup.setGeometry(QtCore.QRect(20, 30, 520, 100))
        self.CameraGroup.setObjectName("CameraGroup")

        self.GlobalCameraStateLabel = QtWidgets.QLabel(self.CameraGroup)
        self.GlobalCameraStateLabel.setGeometry(QtCore.QRect(20, 20, 160, 25))
        self.GlobalCameraStateLabel.setObjectName("GlobalCameraStateLabel")

        self.HandeyeCameraStateLabel = QtWidgets.QLabel(self.CameraGroup)
        self.HandeyeCameraStateLabel.setGeometry(QtCore.QRect(20, 50, 160, 25))
        self.HandeyeCameraStateLabel.setObjectName("HandeyeCameraStateLabel")

        self.ConnectCameraButton = QtWidgets.QPushButton(self.CameraGroup)
        self.ConnectCameraButton.setGeometry(QtCore.QRect(160, 20, 80, 25))
        self.ConnectCameraButton.setObjectName("ConnectCameraButton")
        self.ConnectCameraButton.clicked.connect(self.onClickConnectCameraButton)

        self.DisconnectCameraButton = QtWidgets.QPushButton(self.CameraGroup)
        self.DisconnectCameraButton.setGeometry(QtCore.QRect(160, 50, 80, 25))
        self.DisconnectCameraButton.setObjectName("DisconnectCameraButton")
        self.DisconnectCameraButton.clicked.connect(self.onClickDisconnectCameraButton)

        self.CaptureImageButton = QtWidgets.QPushButton(self.CameraGroup)
        self.CaptureImageButton.setGeometry(QtCore.QRect(250, 20, 80, 25))
        self.CaptureImageButton.setObjectName("CaptureImageButton")
        self.CaptureImageButton.clicked.connect(self.onClickCaptureImageButton)

        self.GetVideoStreamButton = QtWidgets.QPushButton(self.CameraGroup)
        self.GetVideoStreamButton.setGeometry(QtCore.QRect(250, 50, 80, 25))
        self.GetVideoStreamButton.setObjectName("GetVideoStreamButton")
        self.GetVideoStreamButton.clicked.connect(self.onClickGetVideoStreamButton)
        # =============================== 深度相机 ===============================

        # ============================== 图像加载 ==============================
        self.LoadImageGroup = QtWidgets.QGroupBox(self.VisionDebugGroup)
        self.LoadImageGroup.setGeometry(QtCore.QRect(20, 150, 520, 85))
        self.LoadImageGroup.setObjectName("LoadImageGroup")
        self.OpenImageButton = QtWidgets.QPushButton(self.LoadImageGroup)
        self.OpenImageButton.setGeometry(QtCore.QRect(10, 20, 100, 25))
        self.OpenImageButton.setObjectName("OpenImageButton")
        self.OpenImageButton.clicked.connect(self.onClickOpenImageButton)
        # ============================== 图像加载 ==============================
        # ========================================== 视觉模块 ==========================================

        # ========================================= 力传感器模块 ========================================
        self.ForceSensorDebugGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.ForceSensorDebugGroup.setGeometry(QtCore.QRect(1320, 300, 560, 180))
        self.ForceSensorDebugGroup.setObjectName("ForceSensorDebugGroup")
        # ========================================= 力传感器模块 ========================================

        # ========================================= 机械臂模块 =========================================
        self.ManipulatorDebugGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.ManipulatorDebugGroup.setGeometry(QtCore.QRect(1320, 500, 560, 160))
        self.ManipulatorDebugGroup.setObjectName("ManipulatorDebugGroup")

        self.RobotStateLabel = QtWidgets.QLabel(self.ManipulatorDebugGroup)
        self.RobotStateLabel.setGeometry(QtCore.QRect(20, 20, 120, 25))
        self.RobotStateLabel.setObjectName("RobotStateLabel")

        self.ConnectRobotButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.ConnectRobotButton.setGeometry(QtCore.QRect(140, 20, 100, 25))
        self.ConnectRobotButton.setObjectName("ConnectRobotButton")
        self.ConnectRobotButton.clicked.connect(self.onClickConnectRobotButton)

        self.StopRobotTaskButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.StopRobotTaskButton.setGeometry(QtCore.QRect(250, 20, 100, 25))
        self.StopRobotTaskButton.setObjectName("StopRobotTaskButton")
        self.StopRobotTaskButton.clicked.connect(self.onClickStopRobotTaskButton)

        self.RobotSpeedLabel = QtWidgets.QLabel(self.ManipulatorDebugGroup)
        self.RobotSpeedLabel.setGeometry(QtCore.QRect(20, 58, 55, 25))
        self.RobotSpeedLabel.setObjectName("RobotSpeedLabel")

        self.RobotSpeedEdit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.RobotSpeedEdit.setGeometry(QtCore.QRect(75, 58, 45, 25))
        self.RobotSpeedEdit.setObjectName("RobotSpeedEdit")

        self.CopyJointAnglesButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.CopyJointAnglesButton.setGeometry(QtCore.QRect(140, 58, 100, 25))
        self.CopyJointAnglesButton.setObjectName("CopyJointAnglesButton")
        self.CopyJointAnglesButton.clicked.connect(self.onClickCopyJointAnglesButton)

        self.SaveJointAnglesButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.SaveJointAnglesButton.setGeometry(QtCore.QRect(250, 58, 100, 25))
        self.SaveJointAnglesButton.setObjectName("SaveJointAnglesButton")
        self.SaveJointAnglesButton.clicked.connect(self.onClickSaveJointAnglesButton)

        self.GetJointAnglesButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.GetJointAnglesButton.setGeometry(QtCore.QRect(20, 92, 100, 25))
        self.GetJointAnglesButton.setObjectName("GetJointAnglesButton")
        self.GetJointAnglesButton.clicked.connect(self.onClickGetJointAnglesButton)

        self.JointAnglesLabel = QtWidgets.QLabel(self.ManipulatorDebugGroup)
        self.JointAnglesLabel.setGeometry(QtCore.QRect(110 + 40, 92, 390, 25))
        self.JointAnglesLabel.setObjectName("JointAnglesLabel")

        self.SetJointAnglesButton = QtWidgets.QPushButton(self.ManipulatorDebugGroup)
        self.SetJointAnglesButton.setGeometry(QtCore.QRect(20, 125, 100, 25))
        self.SetJointAnglesButton.setObjectName("SetJointAnglesButton")
        self.SetJointAnglesButton.clicked.connect(self.onClickSetJointAnglesButton)

        self.Joint1Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint1Edit.setGeometry(QtCore.QRect(110 + 40, 125, 55, 25))
        self.Joint1Edit.setObjectName("Joint1Edit")

        self.Joint2Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint2Edit.setGeometry(QtCore.QRect(170 + 40, 125, 55, 25))
        self.Joint2Edit.setObjectName("Joint2Edit")

        self.Joint3Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint3Edit.setGeometry(QtCore.QRect(230 + 40, 125, 55, 25))
        self.Joint3Edit.setObjectName("Joint2Edit")

        self.Joint4Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint4Edit.setGeometry(QtCore.QRect(290 + 40, 125, 55, 25))
        self.Joint4Edit.setObjectName("Joint2Edit")

        self.Joint5Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint5Edit.setGeometry(QtCore.QRect(350 + 40, 125, 55, 25))
        self.Joint5Edit.setObjectName("Joint2Edit")

        self.Joint6Edit = QtWidgets.QLineEdit(self.ManipulatorDebugGroup)
        self.Joint6Edit.setGeometry(QtCore.QRect(410 + 40, 125, 55, 25))
        self.Joint6Edit.setObjectName("Joint2Edit")
        # ========================================= 机械臂模块 =========================================

        # ========================================== 标定模块 ==========================================
        self.CalibrationGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.CalibrationGroup.setGeometry(QtCore.QRect(1320, 680, 560, 100))
        self.CalibrationGroup.setObjectName("CalibrationGroup")

        self.InitializeCalibrationPoseButton = QtWidgets.QPushButton(self.CalibrationGroup)
        self.InitializeCalibrationPoseButton.setGeometry(QtCore.QRect(20, 30, 100, 25))
        self.InitializeCalibrationPoseButton.setObjectName("InitializeCalibrationPoseButton")
        self.InitializeCalibrationPoseButton.clicked.connect(self.onClickInitializeCalibrationPoseButton)

        self.ChangeAttitudeButton = QtWidgets.QPushButton(self.CalibrationGroup)
        self.ChangeAttitudeButton.setGeometry(QtCore.QRect(130, 30, 100, 25))
        self.ChangeAttitudeButton.setObjectName("ChangeAttitudeButton")
        self.ChangeAttitudeButton.clicked.connect(self.onClickChangeAttitudeButton)

        self.CaptureCalibrationDataButton = QtWidgets.QPushButton(self.CalibrationGroup)
        self.CaptureCalibrationDataButton.setGeometry(QtCore.QRect(240, 30, 100, 25))
        self.CaptureCalibrationDataButton.setObjectName("CaptureCalibrationDataButton")
        self.CaptureCalibrationDataButton.clicked.connect(self.onClickCaptureCalibrationDataButton)
        # ========================================== 标定模块 ==========================================

        # ======================================== 规划与控制模块 ========================================
        self.PlanAndControlGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.PlanAndControlGroup.setGeometry(QtCore.QRect(1320, 800, 560, 100))
        self.PlanAndControlGroup.setObjectName("PlanAndControlGroup")

        self.InitializePlanAndControlPoseButton = QtWidgets.QPushButton(self.PlanAndControlGroup)
        self.InitializePlanAndControlPoseButton.setGeometry(QtCore.QRect(20, 30, 100, 25))
        self.InitializePlanAndControlPoseButton.setObjectName("InitializePlanAndControlPoseButton")
        self.InitializePlanAndControlPoseButton.clicked.connect(self.onClickInitializePlanAndControlPoseButton)

        self.PlanAndControlButton = QtWidgets.QPushButton(self.PlanAndControlGroup)
        self.PlanAndControlButton.setGeometry(QtCore.QRect(130, 30, 100, 25))
        self.PlanAndControlButton.setObjectName("PlanAndControlButton")
        self.PlanAndControlButton.clicked.connect(self.onClickPlanAndControlButton)
        # ======================================== 规划与控制模块 ========================================

        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1800, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.set_initial_image()

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("MainWindow", "Acupuncture Robot Software"))

        font_group_box = QtGui.QFont()
        font_group_box.setBold(True)
        font_group_box.setPointSize(9)
        font_simple = QtGui.QFont()
        font_simple.setBold(False)
        font_simple.setPointSize(9)

        self.VisionDebugGroup.setTitle(_translate("MainWindow", "视觉感知"))
        self.VisionDebugGroup.setFont(font_group_box)

        self.CameraGroup.setTitle(_translate("MainWindow", "RGB-D相机"))
        self.CameraGroup.setFont(font_group_box)

        self.GlobalCameraStateLabel.setText(_translate("MainWindow", "全局相机: 未连接"))
        self.GlobalCameraStateLabel.setFont(font_simple)
        self.GlobalCameraStateLabel.setStyleSheet("color:red")

        self.HandeyeCameraStateLabel.setText(_translate("MainWindow", "手眼相机: 未连接"))
        self.HandeyeCameraStateLabel.setFont(font_simple)
        self.HandeyeCameraStateLabel.setStyleSheet("color:red")

        self.ConnectCameraButton.setText(_translate("MainWindow", "连接相机"))
        self.ConnectCameraButton.setFont(font_simple)

        self.DisconnectCameraButton.setText(_translate("MainWindow", "断开相机"))
        self.DisconnectCameraButton.setFont(font_simple)

        self.CaptureImageButton.setText(_translate("MainWindow", "采集图像"))
        self.CaptureImageButton.setFont(font_simple)

        self.GetVideoStreamButton.setText(_translate("MainWindow", "打开流"))
        self.GetVideoStreamButton.setFont(font_simple)

        self.LoadImageGroup.setTitle(_translate("MainWindow", "图像"))
        self.LoadImageGroup.setFont(font_group_box)
        self.OpenImageButton.setText(_translate("MainWindow", "Open"))
        self.OpenImageButton.setFont(font_simple)

        self.ForceSensorDebugGroup.setTitle(_translate("MainWindow", "接触力感知"))
        self.ForceSensorDebugGroup.setFont(font_group_box)

        self.ManipulatorDebugGroup.setTitle(_translate("MainWindow", "针灸机械臂"))
        self.ManipulatorDebugGroup.setFont(font_group_box)

        self.ConnectRobotButton.setText(_translate("MainWindow", "连接机械臂"))
        self.ConnectRobotButton.setFont(font_simple)

        self.RobotStateLabel.setText(_translate("MainWindow", "机械臂: 未连接"))
        self.RobotStateLabel.setFont(font_simple)
        self.RobotStateLabel.setStyleSheet("color:red")

        self.RobotSpeedLabel.setText(_translate("MainWindow", "速度:"))
        self.RobotSpeedLabel.setFont(font_simple)

        self.RobotSpeedEdit.setText(_translate("MainWindow", f"{self.robot_speed}"))
        self.RobotSpeedEdit.setFont(font_simple)

        self.StopRobotTaskButton.setText(_translate("MainWindow", "停止运动"))
        self.StopRobotTaskButton.setFont(font_simple)

        self.CopyJointAnglesButton.setText(_translate("MainWindow", "同步关节角"))
        self.CopyJointAnglesButton.setFont(font_simple)

        self.SaveJointAnglesButton.setText(_translate("MainWindow", "保存关节角"))
        self.SaveJointAnglesButton.setFont(font_simple)

        self.GetJointAnglesButton.setText(_translate("MainWindow", "获取关节角"))
        self.GetJointAnglesButton.setFont(font_simple)
        joint_angles_text = f"[{self.joint_angles[0]:.2f}, {self.joint_angles[1]:.2f}, {self.joint_angles[2]:.2f}, " \
                            f"{self.joint_angles[3]:.2f}, {self.joint_angles[4]:.2f}, {self.joint_angles[5]:.2f}]"
        self.JointAnglesLabel.setText(_translate("MainWindow", joint_angles_text))
        self.JointAnglesLabel.setFont(font_simple)

        self.SetJointAnglesButton.setText(_translate("MainWindow", "设置关节角"))
        self.SetJointAnglesButton.setFont(font_simple)
        self.Joint1Edit.setText(_translate("MainWindow", f"{self.joint_angles[0]:.2f}"))
        self.Joint1Edit.setFont(font_simple)
        self.Joint2Edit.setText(_translate("MainWindow", f"{self.joint_angles[1]:.2f}"))
        self.Joint2Edit.setFont(font_simple)
        self.Joint3Edit.setText(_translate("MainWindow", f"{self.joint_angles[2]:.2f}"))
        self.Joint3Edit.setFont(font_simple)
        self.Joint4Edit.setText(_translate("MainWindow", f"{self.joint_angles[3]:.2f}"))
        self.Joint4Edit.setFont(font_simple)
        self.Joint5Edit.setText(_translate("MainWindow", f"{self.joint_angles[4]:.2f}"))
        self.Joint5Edit.setFont(font_simple)
        self.Joint6Edit.setText(_translate("MainWindow", f"{self.joint_angles[5]:.2f}"))
        self.Joint6Edit.setFont(font_simple)

        # ========================================== 标定模块 ==========================================
        self.CalibrationGroup.setTitle(_translate("MainWindow", "系统标定"))
        self.CalibrationGroup.setFont(font_group_box)

        self.InitializeCalibrationPoseButton.setText(_translate("MainWindow", "初始化位姿"))
        self.InitializeCalibrationPoseButton.setFont(font_simple)

        self.ChangeAttitudeButton.setText(_translate("MainWindow", "修改姿态"))
        self.ChangeAttitudeButton.setFont(font_simple)

        self.CaptureCalibrationDataButton.setText(_translate("MainWindow", "采集数据"))
        self.CaptureCalibrationDataButton.setFont(font_simple)
        # ========================================== 标定模块 ==========================================

        # ======================================== 规划与控制模块 ========================================
        self.PlanAndControlGroup.setTitle(_translate("MainWindow", "规划与控制"))
        self.PlanAndControlGroup.setFont(font_group_box)

        self.InitializePlanAndControlPoseButton.setText(_translate("MainWindow", "初始化位姿"))
        self.InitializePlanAndControlPoseButton.setFont(font_simple)

        self.PlanAndControlButton.setText(_translate("MainWindow", "开始规划"))
        self.PlanAndControlButton.setFont(font_simple)
        # ======================================== 规划与控制模块 ========================================

    def set_initial_image(self):
        initial_global_color_image = cv2.imread('Config_Software/initial_image/initial_image1.JPG')
        initial_global_depth_image = cv2.imread('Config_Software/initial_image/initial_image2.JPG')
        initial_handeye_color_image = cv2.imread('Config_Software/initial_image/initial_image3.JPG')
        initial_handeye_depth_image = cv2.imread('Config_Software/initial_image/initial_image4.JPG')

        self.show_image(initial_global_color_image, 1)
        self.show_image(initial_global_depth_image, 2)
        self.show_image(initial_handeye_color_image, 3)
        self.show_image(initial_handeye_depth_image, 4)

    def check_global_camera_connect(self):
        if self.flag_global_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '未连接全局相机!')
            msg_box.exec_()
            return False
        else:
            return True

    def check_handeye_camera_connect(self):
        if self.flag_handeye_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '未连接手眼相机!')
            msg_box.exec_()
            return False
        else:
            return True

    def check_robot_connect(self):
        if self.flag_robot_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '未连接针灸机械臂!')
            msg_box.exec_()
            return False
        else:
            return True

    def get_camera_frame_global(self):
        global_frames = self.global_camera_pipeline.wait_for_frames()
        global_color_frame = global_frames.get_color_frame()
        global_depth_frame = global_frames.get_depth_frame()

        if not global_color_frame or not global_depth_frame:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '全局相机数据获取失败!')
            msg_box.exec_()
            return False

        self.global_color_image = np.asanyarray(global_color_frame.get_data())
        self.global_depth_image = np.asanyarray(global_depth_frame.get_data())
        self.global_depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(self.global_depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return True

    def get_camera_frame_handeye(self):
        handeye_frames = self.handeye_camera_pipeline.wait_for_frames()
        handeye_color_frame = handeye_frames.get_color_frame()
        handeye_depth_frame = handeye_frames.get_depth_frame()

        if not handeye_color_frame or not handeye_depth_frame:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '手眼相机数据获取失败!')
            msg_box.exec_()
            return False

        self.handeye_color_image = np.asanyarray(handeye_color_frame.get_data())
        self.handeye_depth_image = np.asanyarray(handeye_depth_frame.get_data())
        self.handeye_depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(self.handeye_depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return True

    def show_image(self, img, flag):
        image_temp = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_temp = cv2.resize(image_temp, self.image_show_size)
        image_temp = QtGui.QImage(image_temp, image_temp.shape[1], image_temp.shape[0], QtGui.QImage.Format_RGB888)
        image_temp = QtGui.QPixmap(image_temp)
        if flag == 1:
            self.GlobalRGBImageShowLabel.setPixmap(image_temp)
        elif flag == 2:
            self.GlobalDepthImageShowLabel.setPixmap(image_temp)
        elif flag == 3:
            self.HandeyeRGBImageShowLabel.setPixmap(image_temp)
        elif flag == 4:
            self.HandeyeDepthImageShowLabel.setPixmap(image_temp)

    def show_current_joint_angles(self):
        _translate = QtCore.QCoreApplication.translate
        joint_angles_text = f"[{self.joint_angles[0]:.2f}, {self.joint_angles[1]:.2f}, {self.joint_angles[2]:.2f}, " \
                            f"{self.joint_angles[3]:.2f}, {self.joint_angles[4]:.2f}, {self.joint_angles[5]:.2f}]"
        self.JointAnglesLabel.setText(_translate("MainWindow", joint_angles_text))

    def show_desired_joint_angles(self, joint_angles_desired):
        _translate = QtCore.QCoreApplication.translate
        self.Joint1Edit.setText(_translate("MainWindow", f"{joint_angles_desired[0]:.2f}"))
        self.Joint2Edit.setText(_translate("MainWindow", f"{joint_angles_desired[1]:.2f}"))
        self.Joint3Edit.setText(_translate("MainWindow", f"{joint_angles_desired[2]:.2f}"))
        self.Joint4Edit.setText(_translate("MainWindow", f"{joint_angles_desired[3]:.2f}"))
        self.Joint5Edit.setText(_translate("MainWindow", f"{joint_angles_desired[4]:.2f}"))
        self.Joint6Edit.setText(_translate("MainWindow", f"{joint_angles_desired[5]:.2f}"))

    def get_robot_joint_angles(self):
        result = []
        if self.my_robot.HRIF_ReadActJointPos(0, 0, result) == 0:
            self.joint_angles = [float(result[0]), float(result[1]), float(result[2]),
                                 float(result[3]), float(result[4]), float(result[5])]
            self.show_current_joint_angles()
            return True
        else:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '读取关节角失败!')
            msg_box.exec_()
            return False

    def is_robot_motion_done(self):
        result = []
        if self.my_robot.HRIF_IsMotionDone(0, 0, result) == 0:
            return result[0]
        else:
            return False

    def onClickConnectCameraButton(self):
        _translate = QtCore.QCoreApplication.translate

        if self.flag_global_camera_connect is True and self.flag_handeye_camera_connect is True:
            return
        else:
            connect_devices = rs.context().devices
            if len(connect_devices) == 0:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '未检测到相机!')
                msg_box.exec_()
                return
            sn_connect_devices = []
            for d in connect_devices:
                sn_connect_devices.append(d.get_info(rs.camera_info.serial_number))

            if self.cam_serial_num_global in sn_connect_devices:
                # 参数获取与设置
                self.global_camera_config.enable_device(self.cam_serial_num_global)
                self.global_camera_config.enable_stream(
                    rs.stream.depth, self.image_size[0], self.image_size[1], rs.format.z16, 30)
                self.global_camera_config.enable_stream(
                    rs.stream.color, self.image_size[0], self.image_size[1], rs.format.bgr8, 30)
                self.global_camera_pipeline.start(self.global_camera_config)

                # QtWidgets.QMessageBox.information(None, "提示", "全局相机已连接", QtWidgets.QMessageBox.Yes)
                self.flag_global_camera_connect = True
                self.GlobalCameraStateLabel.setText(_translate("MainWindow", "全局相机: 已连接"))
                self.GlobalCameraStateLabel.setStyleSheet("color:green")

            if self.cam_serial_num_handeye in sn_connect_devices:
                # 参数获取与设置
                self.handeye_camera_config.enable_device(self.cam_serial_num_handeye)
                self.handeye_camera_config.enable_stream(
                    rs.stream.depth, self.image_size[0], self.image_size[1], rs.format.z16, 30)
                self.handeye_camera_config.enable_stream(
                    rs.stream.color, self.image_size[0], self.image_size[1], rs.format.bgr8, 30)
                self.handeye_camera_pipeline.start(self.handeye_camera_config)

                # QtWidgets.QMessageBox.information(None, "提示", "手眼相机已连接", QtWidgets.QMessageBox.Yes)
                self.flag_handeye_camera_connect = True
                self.HandeyeCameraStateLabel.setText(_translate("MainWindow", "手眼相机: 已连接"))
                self.HandeyeCameraStateLabel.setStyleSheet("color:green")

    def onClickDisconnectCameraButton(self):
        _translate = QtCore.QCoreApplication.translate

        if self.flag_global_camera_connect is True:
            self.global_camera_pipeline.stop()

            # QtWidgets.QMessageBox.information(None, "提示", "全局相机已断开", QtWidgets.QMessageBox.Yes)
            self.flag_global_camera_connect = False
            self.GlobalCameraStateLabel.setText(_translate("MainWindow", "全局相机: 未连接"))
            self.GlobalCameraStateLabel.setStyleSheet("color:red")

        if self.flag_handeye_camera_connect is True:
            self.handeye_camera_pipeline.stop()

            # QtWidgets.QMessageBox.information(None, "提示", "手眼相机已断开", QtWidgets.QMessageBox.Yes)
            self.flag_handeye_camera_connect = False
            self.HandeyeCameraStateLabel.setText(_translate("MainWindow", "手眼相机: 未连接"))
            self.HandeyeCameraStateLabel.setStyleSheet("color:red")

    def onClickCaptureImageButton(self):
        if self.flag_global_camera_connect is False and self.flag_handeye_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '所有相机均未连接!')
            msg_box.exec_()
            return

        if self.flag_global_camera_connect is True:
            if self.get_camera_frame_global() is False:
                return
            self.show_image(self.global_color_image, 1)
            self.show_image(self.global_depth_colormap, 2)
            cv2.imwrite('global.jpg', self.global_color_image)

        if self.flag_handeye_camera_connect is True:
            if self.get_camera_frame_handeye() is False:
                return
            self.show_image(self.handeye_color_image, 3)
            self.show_image(self.handeye_depth_colormap, 4)
            cv2.imwrite('handeye.jpg', self.handeye_color_image)

    def onGetVideoStreamTimer(self):
        if self.flag_global_camera_connect is True:
            if self.get_camera_frame_global() is False:
                return
            self.show_image(self.global_color_image, 1)
            self.show_image(self.global_depth_colormap, 2)

        if self.flag_handeye_camera_connect is True:
            if self.get_camera_frame_handeye() is False:
                return
            self.show_image(self.handeye_color_image, 3)
            self.show_image(self.handeye_depth_colormap, 4)

    def onClickGetVideoStreamButton(self):
        if self.flag_global_camera_connect is False and self.flag_handeye_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '所有相机均未连接!')
            msg_box.exec_()
            return

        _translate = QtCore.QCoreApplication.translate
        if self.flag_get_video_stream is False:
            self.flag_get_video_stream = True
            self.GetVideoStreamTimer.start(30)
            self.GetVideoStreamButton.setText(_translate("MainWindow", "关闭流"))
        else:
            self.GetVideoStreamTimer.stop()
            self.flag_get_video_stream = False
            self.GetVideoStreamButton.setText(_translate("MainWindow", "打开流"))

    def onClickOpenImageButton(self):
        file_name = 'R-C.jpg'
        self.global_color_image = cv2.imread(file_name)
        self.show_image(self.global_color_image, 1)

    def onClickConnectRobotButton(self):
        _translate = QtCore.QCoreApplication.translate
        if self.flag_robot_connect is False:
            if self.my_robot.HRIF_Connect(0, self.robot_ip, self.robot_port) == 0:
                QtWidgets.QMessageBox.information(None, "提示", "机械臂已连接", QtWidgets.QMessageBox.Yes)
                self.flag_robot_connect = True
                self.ConnectRobotButton.setText(_translate("MainWindow", "断开机械臂"))
                self.RobotStateLabel.setText(_translate("MainWindow", "机械臂: 已连接"))
                self.RobotStateLabel.setStyleSheet("color:green")
            else:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '连接机械臂失败!')
                msg_box.exec_()
                return
        else:
            if self.my_robot.HRIF_DisConnect(0) == 0:
                QtWidgets.QMessageBox.information(None, "提示", "机械臂已断开", QtWidgets.QMessageBox.Yes)
                self.flag_robot_connect = False
                self.ConnectRobotButton.setText(_translate("MainWindow", "连接机械臂"))
                self.RobotStateLabel.setText(_translate("MainWindow", "机械臂: 未连接"))
                self.RobotStateLabel.setStyleSheet("color:red")
            else:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '断开机械臂失败!')
                msg_box.exec_()
                return

    def onClickStopRobotTaskButton(self):
        if self.check_robot_connect() is False:
            return
        self.my_robot.HRIF_GrpStop(0, 0)

    def onClickCopyJointAnglesButton(self):
        self.show_desired_joint_angles(self.joint_angles)

    def onClickSaveJointAnglesButton(self):
        qq = self.joint_angles
        with open(self.file_saved_robot, 'a') as f:
            f.write(f'{qq[0]}, {qq[1]}, {qq[2]}, {qq[3]}, {qq[4]}, {qq[5]}\n')

    def onClickGetJointAnglesButton(self):
        if self.check_robot_connect() is False:
            return

        if self.get_robot_joint_angles() is False:
            return

    def onClickSetJointAnglesButton(self):
        if self.check_robot_connect() is False:
            return

        if self.RobotSpeedEdit.text().isdigit() is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '速度输入错误!')
            msg_box.exec_()
            return
        self.robot_speed = int(self.RobotSpeedEdit.text())
        if self.robot_speed < 0 or self.robot_speed > 200:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '速度超出限制(0~200)')
            msg_box.exec_()
            return

        if is_number(self.Joint1Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节1输入错误!')
            msg_box.exec_()
            return
        joint_angle1 = float(self.Joint1Edit.text())
        if joint_angle1 < self.joint_angles_min[0] or joint_angle1 > self.joint_angles_max[0]:
            msg = f'关节1超出限制({self.joint_angles_min[0]}~{self.joint_angles_max[0]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', msg)
            msg_box.exec_()
            return

        if is_number(self.Joint2Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节2输入错误!')
            msg_box.exec_()
            return
        joint_angle2 = float(self.Joint2Edit.text())
        if joint_angle2 < self.joint_angles_min[1] or joint_angle2 > self.joint_angles_max[1]:
            msg = f'关节2超出限制({self.joint_angles_min[1]}~{self.joint_angles_max[1]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.Joint3Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节3输入错误!')
            msg_box.exec_()
            return
        joint_angle3 = float(self.Joint3Edit.text())
        if joint_angle3 < self.joint_angles_min[2] or joint_angle3 > self.joint_angles_max[2]:
            msg = f'关节3超出限制({self.joint_angles_min[2]}~{self.joint_angles_max[2]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', msg)
            msg_box.exec_()
            return

        if is_number(self.Joint4Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节4输入错误!')
            msg_box.exec_()
            return
        joint_angle4 = float(self.Joint4Edit.text())
        if joint_angle4 < self.joint_angles_min[3] or joint_angle4 > self.joint_angles_max[3]:
            msg = f'关节4超出限制({self.joint_angles_min[3]}~{self.joint_angles_max[3]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', msg)
            msg_box.exec_()
            return

        if is_number(self.Joint5Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节5输入错误!')
            msg_box.exec_()
            return
        joint_angle5 = float(self.Joint5Edit.text())
        if joint_angle5 < self.joint_angles_min[4] or joint_angle5 > self.joint_angles_max[4]:
            msg = f'关节5超出限制({self.joint_angles_min[4]}~{self.joint_angles_max[4]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', msg)
            msg_box.exec_()
            return

        if is_number(self.Joint6Edit.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', '关节6输入错误!')
            msg_box.exec_()
            return
        joint_angle6 = float(self.Joint6Edit.text())
        if joint_angle6 < self.joint_angles_min[5] or joint_angle6 > self.joint_angles_max[5]:
            msg = f'关节6超出限制({self.joint_angles_min[5]}~{self.joint_angles_max[5]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, '警告', msg)
            msg_box.exec_()
            return

        joint_angles_desired = [joint_angle1, joint_angle2, joint_angle3, joint_angle4, joint_angle5, joint_angle6]

        sTcpName = "TCP"
        sUcsName = "Base"
        dVelocity = 50
        dAcc = 50
        dRadius = 50
        nIsUseJoint = 1
        nIsSeek = 0
        nIOBit = 0
        nIOState = 0
        strCmdID = "0"
        nRet = self.my_robot.HRIF_MoveJ(
            0, 0, joint_angles_desired, joint_angles_desired, sTcpName, sUcsName, dVelocity, dAcc,
            dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID)

    def onClickInitializeCalibrationPoseButton(self):
        if self.check_robot_connect() is False:
            return

        joint_angles_desired = self.joint_angles_calib0
        self.show_desired_joint_angles(joint_angles_desired)

        dVelocity = 40
        dAcc = 50
        nRet = self.my_robot.HRIF_MoveJ(
            0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")

    def onClickChangeAttitudeButton(self):
        if self.check_robot_connect() is False:
            return

        z_update = np.random.uniform(low=-20.0, high=20.0)
        y_update = np.random.uniform(low=-20.0, high=20.0)
        x_update = np.random.uniform(low=-20.0, high=20.0)
        zyx_update = np.array([z_update, y_update, x_update]).reshape(3, 1) * math.pi / 180
        T_Update = SixDimPose2TransMat(np.zeros(3), zyx_update)
        Base_T_Needle_Calib = self.Base_T_Needle_Calib0 @ T_Update
        Base_T_End_Calib = Base_T_Needle_Calib @ self.Needle_T_End
        joint_angles_calib, flag_ik = Inverse_Kinematics(Base_T_End_Calib, self.qq0, self.DH_Table)
        if flag_ik == 1:
            return
        joint_angles_calib = joint_angles_calib.reshape(1, 6)[0] * 180 / math.pi
        self.show_desired_joint_angles(joint_angles_calib)

        # print('Base_T_Needle_Calib:\n', Base_T_Needle_Calib)
        # print('Base_T_End_Calib:\n', Base_T_End_Calib)
        # print('joint_angles_calib:\n', joint_angles_calib)

        dVelocity = 40
        dAcc = 50
        nRet = self.my_robot.HRIF_MoveJ(
            0, 0, joint_angles_calib, joint_angles_calib, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")

    def onClickCaptureCalibrationDataButton(self):
        if os.path.exists(self.dir_calib) is False:
            os.mkdir(self.dir_calib)
            os.mkdir(self.dir_calib_global)
            os.mkdir(self.dir_calib_handeye)
            os.mkdir(self.dir_calib_robot)

        calibration_index = 1
        files = os.listdir(self.dir_calib_global)
        if len(files):
            last_file = files[-1]
            last_name, suffix = os.path.splitext(last_file)
            calibration_index = int(last_name) + 1

        if self.check_global_camera_connect() is False or self.check_handeye_camera_connect() is False \
                or self.check_robot_connect() is False:
            return

        if self.get_camera_frame_global() is False or self.get_camera_frame_handeye() is False \
                or self.get_robot_joint_angles() is False:
            return

        path_global = os.path.join(self.dir_calib_global, f'{calibration_index:05d}.jpg')
        cv2.imwrite(path_global, self.global_color_image)
        path_handeye = os.path.join(self.dir_calib_handeye, f'{calibration_index:05d}.jpg')
        cv2.imwrite(path_handeye, self.handeye_color_image)
        path_robot = os.path.join(self.dir_calib_robot, f'{calibration_index:05d}.txt')
        with open(path_robot, 'w') as f:
            f.write(f'{self.joint_angles[0]} {self.joint_angles[1]} {self.joint_angles[2]} '
                    f'{self.joint_angles[3]} {self.joint_angles[4]} {self.joint_angles[5]}\n')

    def onClickInitializePlanAndControlPoseButton(self):
        if self.check_robot_connect() is False:
            return

        joint_angles_desired = self.joint_angles_plan_and_control0
        self.show_desired_joint_angles(joint_angles_desired)

        dVelocity = 40
        dAcc = 50
        nRet = self.my_robot.HRIF_MoveJ(
            0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")

        self.state_plan_and_control = 1

    def onClickPlanAndControlButton(self):
        # if self.check_global_camera_connect() is False or self.check_handeye_camera_connect() is False \
        #         or self.check_robot_connect() is False:
        #     return

        _translate = QtCore.QCoreApplication.translate
        if self.flag_plan_and_control is False:
            self.count_plan_and_control = 0
            self.flag_plan_and_control = True
            self.PlanAndControlTimer.start(30)
            self.PlanAndControlButton.setText(_translate("MainWindow", "结束规划"))
        else:
            self.PlanAndControlTimer.stop()
            self.flag_plan_and_control = False
            self.PlanAndControlButton.setText(_translate("MainWindow", "开始规划"))

    def onPlanAndControlTimer(self):
        self.count_plan_and_control += 1

        if self.flag_global_camera_connect is True:
            if self.get_camera_frame_global() is False:
                return
            self.show_image(self.global_depth_colormap, 2)

            # Localization
            hsv_lower = np.array([0, 80, 80])
            hsv_upper = np.array([5, 255, 255])
            iou_acupoint = [450, 600, 550, 700]
            min_contour_area = 40
            max_contour_area = 200
            # hsv_lower = np.array([15, 80, 80])
            # hsv_upper = np.array([34, 255, 255])
            # iou_acupoint = [100, 700, 400, 720]
            # min_contour_area = 40
            # max_contour_area = 100
            contours_ellipse_global = locate_acupoint(
                self.global_color_image, hsv_lower, hsv_upper, iou_acupoint, min_contour_area, max_contour_area)
            img_ellipse_global = self.global_color_image.copy()
            cv2.drawContours(img_ellipse_global, contours_ellipse_global, -1, (0, 0, 255), -1)
            # cv2.drawContours(img_ellipse_global, contours_ellipse_global, -1, (0, 0, 255), 3)
            self.show_image(img_ellipse_global, 1)

        if self.flag_handeye_camera_connect is True:
            if self.get_camera_frame_handeye() is False:
                return
            self.show_image(self.handeye_depth_colormap, 4)

            # Localization
            hsv_lower = np.array([15, 80, 80])
            hsv_upper = np.array([34, 255, 255])
            contours_ellipse_handeye = locate_acupoint(self.handeye_color_image, hsv_lower, hsv_upper, None, 50, 2000)
            img_ellipse_handeye = self.handeye_color_image.copy()
            cv2.drawContours(img_ellipse_handeye, contours_ellipse_handeye, -1, (0, 0, 255), 3)
            self.show_image(img_ellipse_handeye, 3)

        # if self.state_plan_and_control == 1 and self.count_plan_and_control == 81:
        #     joint_angles_desired = np.array([87.586, 7.656, -39.558, -0.083, -119.762, -2.43])
        #     self.show_desired_joint_angles(joint_angles_desired)
        #
        #     dVelocity = 40
        #     dAcc = 50
        #     nRet = self.my_robot.HRIF_MoveJ(
        #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
        #
        # if self.state_plan_and_control == 1 and self.count_plan_and_control > 101 \
        #         and self.is_robot_motion_done() is True:
        #     self.state_plan_and_control = 2
        #     self.count_plan_and_control = 0
        #
        # if self.state_plan_and_control == 2 and self.count_plan_and_control == 81:
        #     joint_angles_desired = np.array([85.714, 4.318, -64.691, 0.04, -110.88, -4.218])
        #     self.show_desired_joint_angles(joint_angles_desired)
        #
        #     dVelocity = 40
        #     dAcc = 50
        #     nRet = self.my_robot.HRIF_MoveJ(
        #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
        #
        #     self.state_plan_and_control = 0


if __name__ == "__main__":
    envpath = '/home/ros/anaconda3/envs/pyqt5/lib/python3.9/site-packages/cv2/qt/plugins/platforms'
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = envpath
    
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

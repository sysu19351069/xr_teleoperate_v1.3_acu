from cgitb import text
from email.charset import QP
import os
import re
import sys
import cv2
import time
import math
import numpy as np
import pyrealsense2 as rs
import threading
import rospy
from std_msgs.msg import String, Float32MultiArray
import json
import pyqtgraph as pg

from Ui_Acupuncture import Ui_MainWindow
# from PyQt5.QtGui import QPalette
# from PyQt5.QtCore import Qt, QProcess
# from PyQt5.Qt import * ## 会出现Oleinitialize Failed
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from CPS import CPSClient
from kinematics import *
from visual_perception import locate_acupoint
import Force_Sensor.transducer as FS_tr
import Force_Sensor.Motor1 as FS_motor
import visual_perception as VP
import UInode


Deg2Rad = math.pi / 180
Rad2Deg = 180 / math.pi


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


class MyMainForm(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        np.set_printoptions(suppress=True)

        self.flag_save_image = False

        self.save_index = 4

        # ====================================== 相机配置 ===========================================
        self.cam_serial_num_global = '220222067428'
        self.cam_serial_num_local = '220322060052'

        self.image_size = (1280, 720)
        self.image_show_size = (int(self.image_size[0] / 2), int(self.image_size[1] / 2))

        self.Lap_fu = 656.0875
        self.Lap_fv = 655.5052
        self.Lap_u0 = 336.9377
        self.Lap_v0 = 359.9387

        self.flag_global_camera_connect = False
        self.flag_local_camera_connect = False
        self.flag_get_video_stream = False
        self.flag_save_video_stream = False

        self.global_color_image = None
        self.global_depth_image = None
        self.global_depth_colormap = None
        self.local_color_image = None
        self.local_depth_image = None
        self.local_depth_colormap = None

        self.global_camera_pipeline = rs.pipeline()
        self.global_camera_config = rs.config()
        self.local_camera_pipeline = rs.pipeline()
        self.local_camera_config = rs.config()

        self.GetVideoStreamTimer = QTimer()
        self.GetVideoStreamTimer.timeout.connect(self.onGetVideoStreamTimer)

        self.SaveVideoStreamTimer = QTimer()
        self.SaveVideoStreamTimer.timeout.connect(self.onSaveVideoStreamTimer)
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

        # # ====================================== 坐标系配置 ===========================================
        # self.End_T_Needle = np.array([[1, 0, 0, 0],
        #                               [0, 1, 0, 0],
        #                               [0, 0, 1, 234],
        #                               [0, 0, 0, 1]])
        # self.Needle_T_End = np.linalg.inv(self.End_T_Needle)
        # # print(self.End_T_Needle)
        # # print(self.Needle_T_End)
        # # ====================================== 坐标系配置 ===========================================
        #
        # # ======================================= 标定配置 ============================================
        # self.dir_calib = 'Calibration_Data'
        # self.dir_calib_global = os.path.join(self.dir_calib, 'global')
        # self.dir_calib_local = os.path.join(self.dir_calib, 'local')
        # self.dir_calib_robot = os.path.join(self.dir_calib, 'robot')
        #
        # self.pos_calib0 = np.array([0, -550, 30]).reshape(3, 1)
        # self.zyx_calib0 = np.array([90, -160, 0]).reshape(3, 1) * math.pi / 180
        # self.Base_T_Needle_Calib0 = SixDimPose2TransMat(self.pos_calib0, self.zyx_calib0)
        #
        # self.Base_T_End_Calib0 = self.Base_T_Needle_Calib0 @ self.Needle_T_End
        # self.joint_angles_calib0, flag_ik = \
        #     Inverse_Kinematics(self.Base_T_End_Calib0, self.qq0, self.DH_Table)
        # self.joint_angles_calib0 = self.joint_angles_calib0.reshape(1, 6)[0] * Rad2Deg
        #
        # print('Base_T_Needle_Calib0:\n', self.Base_T_Needle_Calib0)
        # print('Base_T_End_Calib0:\n', self.Base_T_End_Calib0)
        # print('joint_angles_calib0:\n', self.joint_angles_calib0)
        # # ======================================= 标定配置 ============================================
        #
        # # ===================================== 规划与控制配置 ==========================================
        # self.joint_angles_plan_and_control0 = np.array([109.931, 26.192, -26.273, -0.006, -118.522, -2.019])
        #
        # self.state_plan_and_control = 0
        # self.count_plan_and_control = 0
        #
        # self.flag_plan_and_control = False
        # self.PlanAndControlTimer = QTimer()
        # self.PlanAndControlTimer.timeout.connect(self.onPlanAndControlTimer)
        # # ===================================== 规划与控制配置 ==========================================

        # ====================================== 力传感器模块配置 ===========================================
        self.flag_force_torque_connect = 0
        # ====================================== 力传感器模块配置 ===========================================

        # ====================================== 电机模块配置 ===========================================
        # ====================================== 电机模块配置 ===========================================

        # ====================================== 针灸控制模块配置 ===========================================
        self.flag_acu_run = 0
        # ====================================== 针灸控制模块配置 ===========================================

        # ===================================== 初始化 ==========================================
        super(MyMainForm, self).__init__(parent)

        self.setupUi(self)
        self.title = "Acupuncture Robot UI"
        self.setWindowTitle(self.title)
        # self.setStyleSheet("#centralwidget{border-image:url(./Config_Software/initial_image/background.png)}")      

        self.labelGlobalSta.setStyleSheet("color:red")
        self.labelLocalSta.setStyleSheet("color:red")

        initial_global_color_image = cv2.imread('/home/ros/Acupuncture_Robot_Software/Config_Software/initial_image/initial_image1.JPG')
        initial_local_color_image = cv2.imread('/home/ros/Acupuncture_Robot_Software/Config_Software/initial_image/initial_image5.jpg')

        self.show_image(initial_global_color_image, 1)
        self.show_image(initial_local_color_image, 2)

        self.labelArm.setStyleSheet("color:red")
        joint_angles_text = f"[{self.joint_angles[0]:.2f}, {self.joint_angles[1]:.2f}, {self.joint_angles[2]:.2f}, " \
                            f"{self.joint_angles[3]:.2f}, {self.joint_angles[4]:.2f}, {self.joint_angles[5]:.2f}]"
        self.labelAngles.setText(joint_angles_text)
        self.editAngle1.setText(f"{self.joint_angles[0]:.2f}")
        self.editAngle2.setText(f"{self.joint_angles[1]:.2f}")
        self.editAngle3.setText(f"{self.joint_angles[2]:.2f}")
        self.editAngle4.setText(f"{self.joint_angles[3]:.2f}")
        self.editAngle5.setText(f"{self.joint_angles[4]:.2f}")
        self.editAngle6.setText(f"{self.joint_angles[5]:.2f}")

        self.x_range = np.arange(1000)
        self.set_graph_ui()  # 设置绘图窗口

        self.comboBoxAcu.addItem('天枢',0)
        self.comboBoxAcu.addItem('大横',1)
        self.comboBoxAcu.addItem('上巨虚',2)
        self.comboBoxAcu.addItem('足三里',3)

        self.comboBoxNe.addItem('直刺90度',0)
        self.comboBoxNe.addItem('斜刺45度',1)
        self.comboBoxNe.addItem('平刺15度',2)

        self.comboBoxStick.addItem('Plug', 0)
        self.comboBoxStick.addItem('Lift', 1)

        self.comboBoxRota.addItem('Pos', 0)
        self.comboBoxRota.addItem('Neg', 1)

        self.editStick.setPlaceholderText("mm")
        self.editRota.setPlaceholderText("r")

        self.editV1.setPlaceholderText("mm/s")
        self.editV2.setPlaceholderText("r/s")
        # ===================================== 初始化 ==========================================

        # ===================================== 视觉处理模块信号 ==========================================
        self.buttonConCam.clicked.connect(self.onClickConnectCameraButton)
        self.buttonDisCam.clicked.connect(self.onClickDisconnectCameraButton)
        self.buttonImg.clicked.connect(self.onClickCaptureImageButton)
        self.buttonVideo.clicked.connect(self.onClickGetVideoStreamButton)
        self.buttonSave.clicked.connect(self.onClickSaveVideo)
        # ===================================== 视觉处理模块信号 ==========================================

        # ===================================== 机械臂调试模块信号 ==========================================
        self.buttonConArm.clicked.connect(self.onClickConnectRobotButton)
        self.buttonStopArm.clicked.connect(self.onClickStopRobotTaskButton)
        self.buttonCopyAngles.clicked.connect(self.onClickCopyJointAnglesButton)
        self.buttonSaveAngles.clicked.connect(self.onClickSaveJointAnglesButton)
        self.buttonObtainAngles.clicked.connect(self.onClickGetJointAnglesButton)
        self.buttonSetAngles.clicked.connect(self.onClickSetJointAnglesButton)
        self.buttonRobotReset.clicked.connect(self.onClickResetRobotButton)
        # ===================================== 机械臂调试模块信号 ==========================================

        # ===================================== 针灸控制模块信号 ==========================================
        self.buttonStab.clicked.connect(self.onClickStabButton)
        self.buttonReset.clicked.connect(self.onClickResetButton)

        self.buttonRoscore.clicked.connect(self.onClickRoscoreButton)
        self.buttonRosrun.clicked.connect(self.onClickRosrunButton)
        
        self.buttonPubAndSub.clicked.connect(self.onClickPubAndSub)
        # ===================================== 针灸控制模块信号 ==========================================

        # ===================================== 力传感器处理模块 ==========================================
        self.update_force_torque_thread = UpdateForceTorqueThread()  # 创建更新波形数据线程
        thread = threading.Thread(target=self.update_force_torque_thread.sub)
        thread.start()
        self.buttonCollect.clicked.connect(self.onClickCollectButton)
        self.update_force_torque_thread._signal_update_force.connect(self.update_force_thread_slot)  # 绑定回调事件
        self.update_force_torque_thread._signal_update_torque.connect(self.update_torque_thread_slot)  # 绑定回调事件
        # ===================================== 力传感器处理模块 ==========================================

        # ===================================== 电机处理模块 ==========================================
        # ===================================== 电机处理模块 ==========================================

        # ===================================== 针刺处理模块 ==========================================
        self.buttonStick.clicked.connect(self.onClickStick)
        self.buttonRota.clicked.connect(self.onClickRotate)
        self.buttonStop.clicked.connect(self.onClickStop)

        self.buttonAcu.clicked.connect(self.onClickAcu)
        self.buttonNe.clicked.connect(self.onClickNe)
        # ===================================== 针刺处理模块 ==========================================

    # ===================================== 其它类内函数 ==========================================
    def check_global_camera_connect(self):
        if self.flag_global_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Not Connect Global Camera!')
            msg_box.exec_()
            return False
        else:
            return True

    def check_local_camera_connect(self):
        if self.flag_local_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Not Connect Local Camera!')
            msg_box.exec_()
            return False
        else:
            return True

    def check_robot_connect(self):
        if self.flag_robot_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Not Connect Acupuncture Robotic Arm!')
            msg_box.exec_()
            return False
        else:
            return True

    def get_camera_frame_global(self):
        global_frames = self.global_camera_pipeline.wait_for_frames()
        global_color_frame = global_frames.get_color_frame()
        global_depth_frame = global_frames.get_depth_frame()

        if not global_color_frame or not global_depth_frame:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Global Camera Fail!')
            msg_box.exec_()
            return False

        self.global_color_image = np.asanyarray(global_color_frame.get_data())
        self.global_depth_image = np.asanyarray(global_depth_frame.get_data())
        self.global_depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(self.global_depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return True

    def get_camera_frame_local(self):
        local_frames = self.local_camera_pipeline.wait_for_frames()
        local_color_frame = local_frames.get_color_frame()
        local_depth_frame = local_frames.get_depth_frame()

        if not local_color_frame or not local_depth_frame:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Local Camera Fail!')
            msg_box.exec_()
            return False

        self.local_color_image = np.asanyarray(local_color_frame.get_data())
        self.local_depth_image = np.asanyarray(local_depth_frame.get_data())

        self.local_depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(self.local_depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return True

    def show_image(self, img, flag):
        image_temp = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_temp = cv2.resize(image_temp, self.image_show_size)

        image_temp = QtGui.QImage(image_temp, image_temp.shape[1], image_temp.shape[0], QtGui.QImage.Format_RGB888)
        image_temp = QtGui.QPixmap(image_temp)

        if flag == 1:
            self.labelGlobalCam.setScaledContents(True)
            self.labelGlobalCam.setPixmap(image_temp)
        elif flag == 2:
            self.labelLocalCam.setScaledContents(True)
            self.labelLocalCam.setPixmap(image_temp)

    def show_current_joint_angles(self):
        joint_angles_text = f"[{self.joint_angles[0]:.2f}, {self.joint_angles[1]:.2f}, {self.joint_angles[2]:.2f}, " \
                            f"{self.joint_angles[3]:.2f}, {self.joint_angles[4]:.2f}, {self.joint_angles[5]:.2f}]"
        self.labelAngles.setText(joint_angles_text)

    def show_desired_joint_angles(self, joint_angles_desired):
        self.editAngle1.setText(f"{joint_angles_desired[0]:.2f}")
        self.editAngle2.setText(f"{joint_angles_desired[1]:.2f}")
        self.editAngle3.setText(f"{joint_angles_desired[2]:.2f}")
        self.editAngle4.setText(f"{joint_angles_desired[3]:.2f}")
        self.editAngle5.setText(f"{joint_angles_desired[4]:.2f}")
        self.editAngle6.setText(f"{joint_angles_desired[5]:.2f}")

    def get_robot_joint_angles(self):
        result = []
        if self.my_robot.HRIF_ReadActJointPos(0, 0, result) == 0:
            self.joint_angles = [float(result[0]), float(result[1]), float(result[2]),
                                 float(result[3]), float(result[4]), float(result[5])]

            joint_angles_txt = str(result[0])+ ' ' + str(result[1])+ ' ' + str(result[2])+ ' ' + str(result[3])+ ' ' + str(result[4])+ ' ' + str(result[5]) 
            with open('/home/ros/Acupuncture_Robot_Software/try.txt', 'w') as f:
                f.write(joint_angles_txt)

            self.show_current_joint_angles()
            return True
        else:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Read Angles Fail!')
            msg_box.exec_()
            return False

    def is_robot_motion_done(self):
        result = []
        if self.my_robot.HRIF_IsMotionDone(0, 0, result) == 0:
            return result[0]
        else:
            return False
    # ===================================== 其它类内函数 ==========================================

    # ===================================== 视觉处理模块槽函数 ==========================================
    def onClickConnectCameraButton(self):
        if self.flag_global_camera_connect is True and self.flag_local_camera_connect is True:
            return
        else:
            connect_devices = rs.context().devices
            if len(connect_devices) == 0:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Not Detect Camera!')
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
                self.labelGlobalSta.setText("Connected")
                self.labelGlobalSta.setStyleSheet("color:green")

            if self.cam_serial_num_local in sn_connect_devices:
                # 参数获取与设置
                self.local_camera_config.enable_device(self.cam_serial_num_local)
                self.local_camera_config.enable_stream(
                    rs.stream.depth, self.image_size[0], self.image_size[1], rs.format.z16, 30)
                self.local_camera_config.enable_stream(
                    rs.stream.color, self.image_size[0], self.image_size[1], rs.format.bgr8, 30)
                self.local_camera_pipeline.start(self.local_camera_config)

                # QtWidgets.QMessageBox.information(None, "提示", "手眼相机已连接", QtWidgets.QMessageBox.Yes)
                self.flag_local_camera_connect = True
                self.labelLocalSta.setText("Connected")
                self.labelLocalSta.setStyleSheet("color:green")

    def onClickDisconnectCameraButton(self):
        if self.flag_global_camera_connect is True:
            self.global_camera_pipeline.stop()

            self.flag_global_camera_connect = False
            self.labelGlobalSta.setText("Not Connected")
            self.labelGlobalSta.setStyleSheet("color:red")

        if self.flag_local_camera_connect is True:
            self.local_camera_pipeline.stop()

            self.flag_local_camera_connect = False
            self.labelLocalSta.setText("Not Connected")
            self.labelLocalSta.setStyleSheet("color:red")

    def onClickCaptureImageButton(self):
        if self.flag_global_camera_connect is False and self.flag_local_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'All Cameras Not Connected!')
            msg_box.exec_()
            return

        if self.flag_global_camera_connect is True:
            if self.get_camera_frame_global() is False:
                return
            self.show_image(self.global_color_image, 1)
            cv2.imwrite('./Source/CaptureImage/global.jpg', self.global_color_image)

        if self.flag_local_camera_connect is True:
            if self.get_camera_frame_local() is False:
                return
            self.show_image(self.local_color_image, 2)
            cv2.imwrite('./Source/CaptureImage/local.jpg', self.local_color_image)

    def onGetVideoStreamTimer(self):
        if self.flag_global_camera_connect is True:
            if self.get_camera_frame_global() is False:
                return
            self.show_image(self.global_color_image, 1)

        if self.flag_local_camera_connect is True:
            if self.get_camera_frame_local() is False:
                return
            self.show_image(self.local_color_image, 2)

    def onClickGetVideoStreamButton(self):
        if self.flag_global_camera_connect is False and self.flag_local_camera_connect is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'All Cameras Not Connect!')
            msg_box.exec_()
            return

        if self.flag_get_video_stream is False:
            self.flag_get_video_stream = True
            self.GetVideoStreamTimer.start(30)
            self.buttonVideo.setText("Close Stream")
        else:
            self.GetVideoStreamTimer.stop()
            self.flag_get_video_stream = False
            self.buttonVideo.setText("Open Stream")
    
    def onClickSaveVideo(self):
        if self.flag_save_video_stream is False:
            self.flag_save_video_stream = True
            self.fourcc = cv2.VideoWriter.fourcc(*'MJPG')
            self.outGlobalRGB = cv2.VideoWriter('./Source/CaptureVideo/globalRGB'+str(self.save_index)+'.avi', self.fourcc, 20.0, (1280,720))
            self.outLocalRGB = cv2.VideoWriter('./Source/CaptureVideo/localRGB'+str(self.save_index)+'.avi', self.fourcc, 20.0, (1280,720))
            self.outGlobalDepth = cv2.VideoWriter('./Source/CaptureVideo/globalDepth'+str(self.save_index)+'.avi', self.fourcc, 20.0, (1280,720))
            self.outLocalDepth = cv2.VideoWriter('./Source/CaptureVideo/localDepth'+str(self.save_index)+'.avi', self.fourcc, 20.0, (1280,720))
            self.SaveVideoStreamTimer.start(30)
            self.buttonSave.setText("Stop")
        else:
            self.SaveVideoStreamTimer.stop()
            self.flag_save_video_stream = False
            self.buttonSave.setText("Save")

            self.save_index += 1

    def onSaveVideoStreamTimer(self):
        self.outGlobalRGB.write(self.global_color_image)
        self.outLocalRGB.write(self.local_color_image)
        self.outGlobalDepth.write(self.global_depth_colormap)
        self.outLocalDepth.write(self.local_depth_colormap)
    # ===================================== 视觉处理模块槽函数 ==========================================

    # ===================================== 机械臂调试模块槽函数 ==========================================
    def onClickConnectRobotButton(self):
        if self.flag_robot_connect is False:
            if self.my_robot.HRIF_Connect(0, self.robot_ip, self.robot_port) == 0:
                QtWidgets.QMessageBox.information(None, "Hint", "Robotic Arm Has Connected", QtWidgets.QMessageBox.Yes)
                self.flag_robot_connect = True
                self.buttonConArm.setText("Disconnect")
                self.labelArm.setText("State: Connected")
                self.labelArm.setStyleSheet("color:green")
            else:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Connect Arm Fail!')
                msg_box.exec_()
                return
        else:
            if self.my_robot.HRIF_DisConnect(0) == 0:
                QtWidgets.QMessageBox.information(None, "Hint", "Robotic Arm Has Disconnected", QtWidgets.QMessageBox.Yes)
                self.flag_robot_connect = False
                self.buttonConArm.setText("Connect")
                self.labelArm.setText("State: Unconnected")
                self.labelArm.setStyleSheet("color:red")
            else:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Disconnect Arm Fail!')
                msg_box.exec_()
                return

    def onClickStopRobotTaskButton(self):
        if self.check_robot_connect() is False:
            return
        self.my_robot.HRIF_GrpStop(0, 0)

        # thread = threading.Thread(target=UInode.talkerStopRobot)
        # thread.start()
        pass

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

        if self.editSpeed.text().isdigit() is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Speed Error!')
            msg_box.exec_()
            return
        self.robot_speed = int(self.editSpeed.text())
        if self.robot_speed < 0 or self.robot_speed > 200:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Speed Exceed(0~200)')
            msg_box.exec_()
            return

        if is_number(self.editAngle1.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle1 Error!')
            msg_box.exec_()
            return
        joint_angle1 = float(self.editAngle1.text())
        if joint_angle1 < self.joint_angles_min[0] or joint_angle1 > self.joint_angles_max[0]:
            msg = f'Angle1 Exceed({self.joint_angles_min[0]}~{self.joint_angles_max[0]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.editAngle2.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle2 Error!')
            msg_box.exec_()
            return
        joint_angle2 = float(self.editAngle2.text())
        if joint_angle2 < self.joint_angles_min[1] or joint_angle2 > self.joint_angles_max[1]:
            msg = f'Angle2 Exceed({self.joint_angles_min[1]}~{self.joint_angles_max[1]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.editAngle3.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle3 Error!')
            msg_box.exec_()
            return
        joint_angle3 = float(self.editAngle3.text())
        if joint_angle3 < self.joint_angles_min[2] or joint_angle3 > self.joint_angles_max[2]:
            msg = f'Angle3 Exceed({self.joint_angles_min[2]}~{self.joint_angles_max[2]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.editAngle4.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle4 Error!')
            msg_box.exec_()
            return
        joint_angle4 = float(self.editAngle4.text())
        if joint_angle4 < self.joint_angles_min[3] or joint_angle4 > self.joint_angles_max[3]:
            msg = f'Angle4 Exceed({self.joint_angles_min[3]}~{self.joint_angles_max[3]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.editAngle5.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle5 Error!')
            msg_box.exec_()
            return
        joint_angle5 = float(self.editAngle5.text())
        if joint_angle5 < self.joint_angles_min[4] or joint_angle5 > self.joint_angles_max[4]:
            msg = f'Angle5 Exceed({self.joint_angles_min[4]}~{self.joint_angles_max[4]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        if is_number(self.editAngle6.text()) is False:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', 'Angle6 Error!')
            msg_box.exec_()
            return
        joint_angle6 = float(self.editAngle6.text())
        if joint_angle6 < self.joint_angles_min[5] or joint_angle6 > self.joint_angles_max[5]:
            msg = f'Angle6 Exceed({self.joint_angles_min[5]}~{self.joint_angles_max[5]})'
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, 'Warning', msg)
            msg_box.exec_()
            return

        joint_angles_desired = [joint_angle1, joint_angle2, joint_angle3, joint_angle4, joint_angle5, joint_angle6]
        # joint_angles_desired = str(joint_angle1)+" "+str(joint_angle2)+" "+str(joint_angle3)+" "+str(joint_angle4)+" "+str(joint_angle5)+" "+str(joint_angle6)

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

        # node.talkerRobotAngles(joint_angles_desired)
        # # 通过话题通信发布设置的关节角
        # # thread = threading.Thread(target=UInode.talkerRobotAngles, args=(joint_angles_desired,))
        # # thread.start()
        # with open('/home/ros/AcuRobot/acupoint_position.txt', 'w') as f:
        #     f.write("")
        # with open('/home/ros/AcuRobot/acupoint_posture.txt', 'w') as f:
        #     f.write("")
        # with open('/home/ros/AcuRobot/reset_joint.txt', 'w') as f:
        #     f.write(joint_angles_desired)
    
    def onClickResetRobotButton(self):
        with open('/home/ros/Acupuncture_Robot_Software/try.txt', 'a') as f:
            f.write('reset')
    # ===================================== 机械臂调试模块槽函数 ==========================================

    # ===================================== 针灸控制模块槽函数 ==========================================
    def onClickStabButton(self):
        # # 连续获取相机的图像，展示在显示框，并保存视频
        # self.onClickConnectCameraButton()
        # self.onClickGetVideoStreamButton()
        # self.onClickSaveVideo()

        # # 连接机械臂
        # self.onClickConnectRobotButton()

        # # 连续采集力传感器的数据并保存位移、速度、力与力矩
        # self.onClickCollectButton()

        # 控制机械臂到指定位姿，进行针灸操作，最后移开机械臂

        # indexAcu = self.comboBoxAcu.currentIndex()
        # indexNe = self.comboBoxNe.currentIndex()

        if self.flag_acu_run == 0:
            thread = threading.Thread(target=UInode.talkerStabAndStop, args=(1,))
            thread.start()
            self.flag_acu_run = 1
            self.buttonStab.setText('Stop')
        elif self.flag_acu_run == 1:
            thread = threading.Thread(target=UInode.talkerStabAndStop, args=(0,))
            thread.start()
            self.flag_acu_run = 0
            self.buttonStab.setText('Go on')

    def onClickResetButton(self):
        thread = threading.Thread(target=UInode.talkerStabAndStop, args=(-1,))
        thread.start()
        self.buttonStab.setText('Stab')

        # self.onClickSaveVideo()
        # self.onClickCollectButton()

    def onClickRoscoreButton(self):
        QProcess.startDetached("/home/ros/Acupuncture_Robot_Software/bash/roscore.sh")

    def onClickRosrunButton(self):
        with open('/home/ros/AcuRobot/acupoint_position.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/acupoint_posture.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/sensing_force.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/reset_joint.txt', 'w') as f:
            f.write("")
        QProcess.startDetached("/home/ros/Acupuncture_Robot_Software/bash/rosrun.sh")

    def onClickPubAndSub(self):
        QProcess.startDetached("/home/ros/Acupuncture_Robot_Software/bash/pubAndSubNode.sh")
    # ===================================== 针灸控制模块槽函数 ==========================================

    # ===================================== 力传感器处理模块函数 ==========================================
    # 绘制力与力矩随时间变化曲线
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True, background='w') # antialias=True开启曲线抗锯齿
        win1 = pg.GraphicsLayoutWidget()
        win2 = pg.GraphicsLayoutWidget()

        self.plotViewF.addWidget(win1)
        self.plotViewT.addWidget(win2)

        self.f_plot_view = win1.addPlot(title="Force") 
        self.f_plot_view.setLabel('left', text='Force', units='N', color='#000000') # y轴设置函数
        self.f_plot_view.showGrid(x=True, y=True)
        self.f_plot_view.setLogMode(x=False, y=False) # False代表线性坐标轴，True代表对数坐标轴
        self.f_plot_view.setLabel('bottom', text='Time', units='s', color='#000000') # x轴设置函数
        self.f_plot_view.setLimits(xMin=0, xMax=1024, yMin=-20, yMax=20) # 横纵坐标范围

        self.t_plot_view = win2.addPlot(title="Torque") 
        self.t_plot_view.setLabel('left', text='Torque', units='Nm', color='#000000')  
        self.t_plot_view.showGrid(x=True, y=True)  
        self.t_plot_view.setLogMode(x=False, y=False)  
        self.t_plot_view.setLabel('bottom', text='Time', units='s', color='#000000') 
        self.t_plot_view.setLimits(xMin=0, xMax=1024, yMin=-20, yMax=20)

    def onClickCollectButton(self):
        if self.flag_force_torque_connect == 0:
            self.flag_force_torque_connect = 1
            self.buttonCollect.setText('Stop')
            self.update_force_torque_thread.start()

        elif self.flag_force_torque_connect == 1:
            self.flag_force_torque_connect = 2
            self.update_force_torque_thread.pause()
            self.buttonCollect.setText('Go On')

        elif self.flag_force_torque_connect == 2:
            self.flag_force_torque_connect = 1
            self.buttonCollect.setText('Stop')
            self.update_force_torque_thread.resume()

    def update_force_thread_slot(self, data):
        # 线程回调函数
        data = json.loads(data)
        data = np.array(data['force'])
        self.f_plot_view.plot(self.x_range, data, pen='b', name='动态波形', clear=True) 

    def update_torque_thread_slot(self, data):
        data = json.loads(data)
        data = np.array(data['torque'])
        self.t_plot_view.plot(self.x_range, data, pen='b', name='动态波形', clear=True) 
    # ===================================== 力传感器处理模块槽函数 ==========================================

    # ===================================== 电机处理模块槽函数 ==========================================
    # ===================================== 电机处理模块槽函数 ==========================================

    # ===================================== 针刺处理模块槽函数 ==========================================
    def onClickStick(self):
        indexStick = self.comboBoxStick.currentIndex() # 0为plug，1为lift
        distanceStick = float(self.editStick.text())
        vStick = float(self.editV1.text())
        thread = threading.Thread(target=UInode.talkerStick, args=(indexStick, distanceStick, vStick,))
        thread.start()
        
    def onClickRotate(self):
        indexRotate = self.comboBoxRota.currentIndex() # 0为正转，1为反转
        circleRotate = float(self.editRota.text())
        vRotate = float(self.editV2.text())
        thread = threading.Thread(target=UInode.talkerRotate, args=(indexRotate, circleRotate, vRotate,))
        thread.start()

    def onClickStop(self):
        thread = threading.Thread(target=UInode.talkerMotorStop)
        thread.start()

    def onClickAcu(self):
        Acu = self.editAcu1.text() + " " + self.editAcu2.text() + " " + self.editAcu3.text()
        with open('/home/ros/AcuRobot/acupoint_posture.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/reset_joint.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/acupoint_position.txt', 'w') as f:
            f.write(Acu)
    
    def onClickNe(self):
        Ne = self.editNe1.text() + " " + self.editNe2.text() + " " + self.editNe3.text()
        with open('/home/ros/AcuRobot/acupoint_position.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/reset_joint.txt', 'w') as f:
            f.write("")
        with open('/home/ros/AcuRobot/acupoint_posture.txt', 'w') as f:
            f.write(Ne)
    # ===================================== 针刺处理模块槽函数 ==========================================


# ===================================== 力传感器处理线程类 ==========================================
# 使用线程不断更新力与力矩数据
class UpdateForceTorqueThread(QThread):
    _signal_update_force = pyqtSignal(str)  # 信号
    _signal_update_torque = pyqtSignal(str)  # 信号

    def __init__(self, parent=None):
        super(UpdateForceTorqueThread, self).__init__(parent)
        self.qmut = QMutex()
        self.is_exit = False
        self.x_range = 1000
        self.isCancel = False
        self.isPause = False
        self.cond = QWaitCondition()
        self.currentForce = 0
        self.currentTorque = 0
        self.num = 0

        # 顺便保存位移与速度
        self.currentPos1 = 0
        self.currentPos2 = 0
        self.currentVel1 = 0
        self.currentVel2 = 0

    def sub(self):
        try:
            rospy.init_node('ui_node', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException as e:
            pass
        rospy.Subscriber('/LocAndFT', Float32MultiArray, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, msg):
        self.currentForce = msg.data[4]
        self.currentTorque = msg.data[5]

        self.currentPos1 = msg.data[0]/2000 # mm
        self.currentPos2 = msg.data[1]/2000 # mm
        self.currentVel1 = msg.data[2]
        self.currentVel2 = msg.data[3]
        self.currentVel1 = self.translate(self.currentVel1)
        self.currentVel2 = self.translate(self.currentVel2)

        rospy.loginfo(msg.data) 

    def run(self):
        F_list = np.zeros((self.x_range), dtype=int)
        F_list = F_list.tolist()
        T_list = np.zeros((self.x_range), dtype=int)
        T_list = T_list.tolist()

        while True:
            self.qmut.lock()
            if self.is_exit:
                break
            if self.isPause:
                self.cond.wait(self.qmut)
            self.qmut.unlock()
            if self.isCancel:
                self.isCancel = False
                break

            F, T = self.currentForce, self.currentTorque

            del F_list[0]
            F_list.append(F)
            del T_list[0]
            T_list.append(T)

            # # 保存位移1、位移2、速度1、速度2、力与力矩
            # tmp = str(self.currentPos1)+' '+str(self.currentPos2)+' '+str(self.currentVel1)+' '+str(self.currentVel2)+' '+str(F)+' '+str(T)+'\n'
            # with open('/home/ros/Acupuncture_Robot_Software/Source/SavePosVelForceTorque/pos_vel_force_torque'+str(self.save_index)+'.txt', 'a') as f:
            #     f.write(tmp)

            self._signal_update_force.emit(json.dumps({'force': F_list}))  # 发送信号给槽函数
            self._signal_update_torque.emit(json.dumps({'torque': T_list}))  # 发送信号给槽函数

            time.sleep(0.1)  # 调节采样频率

        self.qmut.unlock()

    def cancel(self):
        print("停止收集力与力矩")
        # self.terminate()
        self.isCancel = True

    def pause(self):
        self.qmut.lock()
        print("暂停收集")
        self.isPause = True
        self.qmut.unlock()

    def resume(self):
        self.qmut.lock()
        self.isPause = False
        self.cond.wakeAll()
        self.qmut.unlock()

    def translate(self, vel):
        binnum = bin(int(vel))
        result = 0
        binstr = str(binnum)
        binstr = binstr.strip()
        if binstr[:2] == '0b':
            if binstr[2] == '_':
                binstr = binstr[3:]
            else:
                binstr = binstr[2:]
        if binstr[0] == '_':
            print('error')
        elif binstr[0] == '0':
            result = int(binstr, base=2)
            return result
        elif binstr[0] == '1':
            a = int(binstr, base=2)
            binstr = binstr.replace('_', '')
            result = a - 2**len(binstr)
            return result
        else:
            print('error')
# ===================================== 力传感器处理线程类 ==========================================


# ===================================== 电机处理线程类 ==========================================
# ===================================== 电机处理线程类 ==========================================




    # def onClickInitializeCalibrationPoseButton(self):
    #     if self.check_robot_connect() is False:
    #         return
    #
    #     joint_angles_desired = self.joint_angles_calib0
    #     self.show_desired_joint_angles(joint_angles_desired)
    #
    #     dVelocity = 40
    #     dAcc = 50
    #     nRet = self.my_robot.HRIF_MoveJ(
    #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
    #
    # def onClickChangeAttitudeButton(self):
    #     if self.check_robot_connect() is False:
    #         return
    #
    #     z_update = np.random.uniform(low=-20.0, high=20.0)
    #     y_update = np.random.uniform(low=-20.0, high=20.0)
    #     x_update = np.random.uniform(low=-20.0, high=20.0)
    #     zyx_update = np.array([z_update, y_update, x_update]).reshape(3, 1) * math.pi / 180
    #     T_Update = SixDimPose2TransMat(np.zeros(3), zyx_update)
    #     Base_T_Needle_Calib = self.Base_T_Needle_Calib0 @ T_Update
    #     Base_T_End_Calib = Base_T_Needle_Calib @ self.Needle_T_End
    #     joint_angles_calib, flag_ik = Inverse_Kinematics(Base_T_End_Calib, self.qq0, self.DH_Table)
    #     if flag_ik == 1:
    #         return
    #     joint_angles_calib = joint_angles_calib.reshape(1, 6)[0] * 180 / math.pi
    #     self.show_desired_joint_angles(joint_angles_calib)
    #
    #     # print('Base_T_Needle_Calib:\n', Base_T_Needle_Calib)
    #     # print('Base_T_End_Calib:\n', Base_T_End_Calib)
    #     # print('joint_angles_calib:\n', joint_angles_calib)
    #
    #     dVelocity = 40
    #     dAcc = 50
    #     nRet = self.my_robot.HRIF_MoveJ(
    #         0, 0, joint_angles_calib, joint_angles_calib, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
    #
    # def onClickCaptureCalibrationDataButton(self):
    #     if os.path.exists(self.dir_calib) is False:
    #         os.mkdir(self.dir_calib)
    #         os.mkdir(self.dir_calib_global)
    #         os.mkdir(self.dir_calib_local)
    #         os.mkdir(self.dir_calib_robot)
    #
    #     calibration_index = 1
    #     files = os.listdir(self.dir_calib_global)
    #     if len(files):
    #         last_file = files[-1]
    #         last_name, suffix = os.path.splitext(last_file)
    #         calibration_index = int(last_name) + 1
    #
    #     if self.check_global_camera_connect() is False or self.check_local_camera_connect() is False \
    #             or self.check_robot_connect() is False:
    #         return
    #
    #     if self.get_camera_frame_global() is False or self.get_camera_frame_local() is False \
    #             or self.get_robot_joint_angles() is False:
    #         return
    #
    #     path_global = os.path.join(self.dir_calib_global, f'{calibration_index:05d}.jpg')
    #     cv2.imwrite(path_global, self.global_color_image)
    #     path_local = os.path.join(self.dir_calib_local, f'{calibration_index:05d}.jpg')
    #     cv2.imwrite(path_local, self.local_color_image)
    #     path_robot = os.path.join(self.dir_calib_robot, f'{calibration_index:05d}.txt')
    #     with open(path_robot, 'w') as f:
    #         f.write(f'{self.joint_angles[0]} {self.joint_angles[1]} {self.joint_angles[2]} '
    #                 f'{self.joint_angles[3]} {self.joint_angles[4]} {self.joint_angles[5]}\n')
    #
    # def onClickInitializePlanAndControlPoseButton(self):
    #     if self.check_robot_connect() is False:
    #         return
    #
    #     joint_angles_desired = self.joint_angles_plan_and_control0
    #     self.show_desired_joint_angles(joint_angles_desired)
    #
    #     dVelocity = 40
    #     dAcc = 50
    #     nRet = self.my_robot.HRIF_MoveJ(
    #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
    #
    #     self.state_plan_and_control = 1
    #
    # def onClickPlanAndControlButton(self):
    #     # if self.check_global_camera_connect() is False or self.check_local_camera_connect() is False \
    #     #         or self.check_robot_connect() is False:
    #     #     return
    #
    #     _translate = QtCore.QCoreApplication.translate
    #     if self.flag_plan_and_control is False:
    #         self.count_plan_and_control = 0
    #         self.flag_plan_and_control = True
    #         self.PlanAndControlTimer.start(30)
    #         self.PlanAndControlButton.setText(_translate("MainWindow", "结束规划"))
    #     else:
    #         self.PlanAndControlTimer.stop()
    #         self.flag_plan_and_control = False
    #         self.PlanAndControlButton.setText(_translate("MainWindow", "开始规划"))
    #
    # def onPlanAndControlTimer(self):
    #     self.count_plan_and_control += 1
    #
    #     if self.flag_global_camera_connect is True:
    #         if self.get_camera_frame_global() is False:
    #             return
    #         self.show_image(self.global_depth_colormap, 2)
    #
    #         # Localization
    #         hsv_lower = np.array([0, 80, 80])
    #         hsv_upper = np.array([5, 255, 255])
    #         iou_acupoint = [450, 600, 550, 700]
    #         min_contour_area = 40
    #         max_contour_area = 200
    #         # hsv_lower = np.array([15, 80, 80])
    #         # hsv_upper = np.array([34, 255, 255])
    #         # iou_acupoint = [100, 700, 400, 720]
    #         # min_contour_area = 40
    #         # max_contour_area = 100
    #         contours_ellipse_global = locate_acupoint(
    #             self.global_color_image, hsv_lower, hsv_upper, iou_acupoint, min_contour_area, max_contour_area)
    #         img_ellipse_global = self.global_color_image.copy()
    #         cv2.drawContours(img_ellipse_global, contours_ellipse_global, -1, (0, 0, 255), -1)
    #         # cv2.drawContours(img_ellipse_global, contours_ellipse_global, -1, (0, 0, 255), 3)
    #         self.show_image(img_ellipse_global, 1)
    #
    #     if self.flag_local_camera_connect is True:
    #         if self.get_camera_frame_local() is False:
    #             return
    #         self.show_image(self.local_depth_colormap, 4)
    #
    #         # Localization
    #         hsv_lower = np.array([15, 80, 80])
    #         hsv_upper = np.array([34, 255, 255])
    #         contours_ellipse_local = locate_acupoint(self.local_color_image, hsv_lower, hsv_upper, None, 50, 2000)
    #         img_ellipse_local = self.local_color_image.copy()
    #         cv2.drawContours(img_ellipse_local, contours_ellipse_local, -1, (0, 0, 255), 3)
    #         self.show_image(img_ellipse_local, 3)
    #
    #     # if self.state_plan_and_control == 1 and self.count_plan_and_control == 81:
    #     #     joint_angles_desired = np.array([87.586, 7.656, -39.558, -0.083, -119.762, -2.43])
    #     #     self.show_desired_joint_angles(joint_angles_desired)
    #     #
    #     #     dVelocity = 40
    #     #     dAcc = 50
    #     #     nRet = self.my_robot.HRIF_MoveJ(
    #     #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
    #     #
    #     # if self.state_plan_and_control == 1 and self.count_plan_and_control > 101 \
    #     #         and self.is_robot_motion_done() is True:
    #     #     self.state_plan_and_control = 2
    #     #     self.count_plan_and_control = 0
    #     #
    #     # if self.state_plan_and_control == 2 and self.count_plan_and_control == 81:
    #     #     joint_angles_desired = np.array([85.714, 4.318, -64.691, 0.04, -110.88, -4.218])
    #     #     self.show_desired_joint_angles(joint_angles_desired)
    #     #
    #     #     dVelocity = 40
    #     #     dAcc = 50
    #     #     nRet = self.my_robot.HRIF_MoveJ(
    #     #         0, 0, joint_angles_desired, joint_angles_desired, "TCP", "Base", dVelocity, dAcc, 50, 1, 0, 0, 0, "0")
    #     #
    #     #     self.state_plan_and_control = 0


if __name__ == '__main__':
    envpath = '/home/ros/anaconda3/envs/pyqt5/lib/python3.9/site-packages/cv2/qt/plugins/platforms'
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = envpath

    app = QtWidgets.QApplication(sys.argv)
    MyWin = MyMainForm()
    MyWin.setWindowIcon(QIcon("/home/ros/Acupuncture_Robot_Software/Config_Software/initial_image/computer.png"))
    MyWin.show()
    sys.exit(app.exec_())

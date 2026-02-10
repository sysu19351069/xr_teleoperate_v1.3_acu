#!/usr/bin/env python
# coding:utf-8

import threading
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import math
import serial
import serial.tools.list_ports
import crcmod
import binascii
import time
import Force_Sensor.motor as FS_m
import Force_Sensor.transducer as FS_t


# 负责接受速度并控制电机运动的节点
class SubNode:
    def callback(self, msg):
        self.v1 = msg.data[0]
        self.v2 = msg.data[1]
        rospy.loginfo(msg.data) 

        Mode = 2
        self.write_data(Mode)     
      

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('sub_node', anonymous=True)
        rospy.Subscriber('/MotorV', Float32MultiArray, self.callback, queue_size=1)
        self.flag_pre = None

        FS_m.status()
        motor1 = "/dev/ttyUSB3"
        motor2 = "/dev/ttyUSB2"
        print(motor1, motor2)
        self.motor1 = FS_m.init(motor1)
        self.motor2 = FS_m.init(motor2)

        # self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        # self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])

        self.dirCur1 = 0
        self.dirCur2 = 0             # 用于记录当前方向
        self.lCur1 = 0
        self.lCur2 = 0             # 用于记录当前方向

        self.v1 = 0
        self.v2 = 0             # 用于记录发布的速度
        
        rospy.spin()
        
    def write_data(self, Mode):
        if Mode == 0:                                                                           # 停止运行
            print("end")
        elif Mode == 1:                                                                         # 设置绝对位置
            self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
            self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        elif Mode == 2:                                                                         # 运行模式
            # 指令处理
            Dir1 = FS_m.dirGet(self.v1)                                                                   # 通过速度正负得到方向(1, 0, -1)
            Dir2 = FS_m.dirGet(self.v2)
            print("V1: %s, Dir1: %s, V2: %s, Dir2:%s" % (self.v1, Dir1, self.v2, Dir2))                   # 打印当前速度和方向
            ordV1, ordV2 = FS_m.vConvert(math.fabs(self.v1), math.fabs(self.v2))                               # 速度指令转换
            ordDir1, ordDir2 = FS_m.dirConvert(Dir1, Dir2)                                           # 方向指令转换

            # 启动电机
            # 判断电机1是否需要变向
            if self.dirCur1 != Dir1:                                                                 # 若需要变向
                self.motor1.write([0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34])                  # 先停止
                time.sleep(0.005)                                                               # 写指令间隔5ms, 同下
                self.motor1.write(ordV1)                                                             # 设置速度
                time.sleep(0.005)
                self.motor1.write(ordDir1)                                                           # 按指定方向启动
                time.sleep(0.005)
            else:                                                                               # 若不需要变向
                self.motor1.write(ordV1)
                time.sleep(0.005)
            # 判断电机2是否需要变向
            if self.dirCur2 != Dir2:
                self.motor2.write([0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07])                  # 先停止
                time.sleep(0.005)
                self.motor2.write(ordV2)                                                             # 设置速度
                time.sleep(0.005)
                self.motor2.write(ordDir2)                                                           # 按指定方向启动
                time.sleep(0.005)
            else:
                self.motor2.write(ordV2)
                time.sleep(0.005)

            # # 读取当前位置和实际速度
            # a = FS_m.readLocation(self.motor1)                                                       # 将之前接受的信息读出, 清空接收缓冲区
            # b = FS_m.readLocation(self.motor2)
            # self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                      # 发出读取位置指令
            # self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            # time.sleep(0.005)                                                                   # 延时5ms
            # lCur1 = FS_m.readLocation(self.motor1)                                                        # 读取当前绝对位置, 单位1/100000圈
            # lCur2 = FS_m.readLocation(self.motor2)
            # self.motor1.write([0x01, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xCD])                      # 发出读取速度指令
            # self.motor2.write([0x02, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xFE])
            # time.sleep(0.005)
            # vCur1 = FS_m.readSpeed(self.motor1)
            # vCur2 = FS_m.readSpeed(self.motor2)
            # print("lCur1: ", lCur1, "vCur1: ", vCur1)
            # print("lCur2: ", lCur2, "vCur2: ", vCur2)


if __name__ == '__main__':
    try:
        nodeWrite = SubNode()
    except rospy.ROSInterruptException:
        pass
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


# 负责发布消息的节点
# 发布两个电机的位置信息、力传感器的力与力矩
class PubNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pub_node', anonymous=True)

        # 创建发布者对象
        self.pub = rospy.Publisher('/LocAndF', Float32MultiArray, queue_size=1)

        FS_m.status()
        motor1 = "/dev/ttyUSB0"
        motor2 = "/dev/ttyUSB4"
        print(motor1, motor2)
        self.motor1 = FS_m.init(motor1)
        self.motor2 = FS_m.init(motor2)

        self.dirCur1 = 0
        self.dirCur2 = 0             # 用于记录当前方向
        self.lCur1 = 0
        self.lCur2 = 0             # 用于记录当前方向

        self.v1 = 0
        self.v2 = 0             # 用于记录发布的速度

        trans = "/dev/ttyUSB0"
        self.trans = FS_t.init(trans)
        self.pF = 0 
        self.pT = 0
        
        # 创建一个Float32MultiArray消息对象
        self.msg = Float32MultiArray()

        # 创建信号量
        self.semaphore = threading.Semaphore(1)
        
    def publish_data(self):
        rate = rospy.Rate(100)  # 设置发布频率为100Hz
        while not rospy.is_shutdown():
            self.semaphore.acquire()

            # 读取当前位置和实际速度
            a = FS_m.readLocation(self.motor1)                                                       # 将之前接受的信息读出, 清空接收缓冲区
            b = FS_m.readLocation(self.motor2)
            self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                      # 发出读取位置指令
            self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            time.sleep(0.005)                                                                   # 延时5ms
            self.lCur1 = FS_m.readLocation(self.motor1)                                                        # 读取当前绝对位置, 单位1/100000圈
            self.lCur2 = FS_m.readLocation(self.motor2)
            self.motor1.write([0x01, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xCD])                      # 发出读取速度指令
            self.motor2.write([0x02, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xFE])
            time.sleep(0.005)
            self.v1 = FS_m.readSpeed(self.motor1)
            self.v2 = FS_m.readSpeed(self.motor2)

            F, T, S = FS_t.read_data(self.trans)
            if S is True:
                F = F / 100 / (1.63200 * 5) * 49 + 4.443
                T = T / 100 / (1.25845 * 5) * 0.5 - 0.0165
            else:
                F, T = self.pF, self.pT
            self.pF, self.pT = F, T

            # 设置消息的数据，依次为电机1的位置，电机2的位置，力，力矩
            self.msg.data = [self.lCur1, self.lCur2, self.v1, self.v2, F, T]  
            self.pub.publish(self.msg)
            rospy.loginfo(self.msg.data)

            self.semaphore.release()

            rate.sleep()

    def run(self):
        # 创建一个线程来执行发布数据的函数
        publish_thread = threading.Thread(target=self.publish_data)
        publish_thread.start()
        # 进入ROS循环
        rospy.spin()


if __name__ == '__main__':
    try:
        nodePub = PubNode()
        nodePub.run()
    except rospy.ROSInterruptException:
        pass
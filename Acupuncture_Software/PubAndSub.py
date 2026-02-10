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
import Force_Sensor.Motor1 as FS_m
import Force_Sensor.transducer as FS_t


class PubSubNode:
    def __init__(self):
        rospy.init_node('pub_sub_node', anonymous=True)

        self.pub = rospy.Publisher('/LocAndFT', Float32MultiArray, queue_size=1)

        FS_m.status()
        motor1 = "/dev/ttyUSB0"
        motor2 = "/dev/ttyUSB1"
        print(motor1, motor2)
        self.motor1 = FS_m.init(motor1)
        self.motor2 = FS_m.init(motor2)

        tran = "/dev/ttyUSB2"
        self.tran = FS_t.init(tran)

        self.dirCur1 = 0
        self.dirCur2 = 0         
        self.lCur1 = 0
        self.lCur2 = 0     

        self.v1 = 0
        self.v2 = 0     

        self.pF = 0
        self.pT = 0
        self.F = 0
        self.T = 0
        
        self.msg = Float32MultiArray()

        self.semaphore = threading.Semaphore(1)

        self.flag = 0  # 用于优先设置速度

        # 电机1设置
        # 设置正限位
        self.motor1.write([0x01, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x5F, 0x90, 0x00, 0x01, 0x27, 0x72])#正限位90000
        time.sleep(0.01)
        # 设置负限位
        self.motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x74, 0x0B])#负限位0
        time.sleep(0.01)
        # 设置当前位置
        self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0xB2, 0x62])#80000
        time.sleep(0.01)

        # 电机2设置
        # 设置正限位
        self.motor2.write([0x02, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x4E, 0x20, 0x00, 0x00, 0xED, 0x2D])#正限位20000
        time.sleep(0.01)
        # 设置负限位
        self.motor2.write([0x02, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x4F])#负限位0
        time.sleep(0.01)
        # 设置当前位置
        self.motor2.write([0x02, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x27, 0x10, 0x00, 0x00, 0x7B, 0x1F])#10000
        time.sleep(0.01)  

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
        
    def publish_data(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.flag == 0:
                self.semaphore.acquire()

                a = FS_m.readLocation(self.motor1)                                                      
                b = FS_m.readLocation(self.motor2)
                self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                    
                self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
                time.sleep(0.005)                                                              
                self.lCur1 = FS_m.readLocation(self.motor1)                                            
                self.lCur2 = FS_m.readLocation(self.motor2)
                self.motor1.write([0x01, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xCD])            
                self.motor2.write([0x02, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xFE])
                time.sleep(0.005)
                self.v1 = FS_m.readSpeed(self.motor1)
                self.v2 = FS_m.readSpeed(self.motor2)

                F, T, S = FS_t.read_data(self.tran)
                if S is True:
                    self.F = F / 100 / (1.63200 * 5) * 49
                    self.T = T / 100 / (1.25845 * 5) * 0.5
                else:
                    self.F, self.T = self.pF, self.pT
                self.pF, self.pT = F, T

                if self.v1 > 3000:
                    self.v1 = self.translate(self.v1)

                self.msg.data = [self.lCur1, self.lCur2, self.v1, self.v2, self.F, self.T]

                # # 保存 l/2000 mm; v/30 +- mm/s; F N; T Nm
                # tmp = str(self.msg.data)+'\n'
                # with open('/home/ros/Acupuncture_Robot_Software/l1_l2_v1_v2_F_T'+'4.txt', 'a') as f:
                #     f.write(tmp)  

                self.pub.publish(self.msg)
                rospy.loginfo(self.msg.data)

                self.semaphore.release()

                rate.sleep()
            else:
                pass

    def callbackVelocity(self, msg):
        self.flag = 1
        self.semaphore.acquire()

        self.v1 = msg.data[0]
        self.v2 = msg.data[1]
        rospy.loginfo(msg.data) 

        self.write_data()  
         
        self.semaphore.release()
        self.flag = 0  
    
    def callbackMove(self, msg):
        self.flag = 1
        self.semaphore.acquire()

        # 运行到初始位置
        self.motor1.write([0x01, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0x31, 0x09]) # 绝对位置80000
        time.sleep(0.1)
        self.motor2.write([0x02, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0x27, 0x10, 0x00, 0x00, 0xF8, 0x74]) # 绝对位置10000
         
        self.semaphore.release() 
        self.flag = 0
        
    def subscribe_data(self):
        rospy.Subscriber('/MotorV', Float32MultiArray, self.callbackVelocity, queue_size=1)
        rospy.Subscriber('/MovePos', Float32MultiArray, self.callbackMove, queue_size=1)
        rospy.spin()
    
    def write_data(self):                                                       
        Dir1 = FS_m.dirGet(-self.v1)                                                                  
        Dir2 = FS_m.dirGet(self.v2)
        print("V1: %s, Dir1: %s, V2: %s, Dir2:%s" % (self.v1, Dir1, self.v2, Dir2))                
        ordV1, ordV2 = FS_m.vConvert(math.fabs(self.v1), math.fabs(self.v2))                     
        ordDir1, ordDir2 = FS_m.dirConvert(Dir1, Dir2)                        

        if self.dirCur1 != Dir1:                                                                 
            self.motor1.write([0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34])                
            time.sleep(0.005)                                                         
            self.motor1.write(ordV1)                                                    
            time.sleep(0.005)
            self.motor1.write(ordDir1)                                                
            time.sleep(0.005)
        else:                                                                     
            self.motor1.write(ordV1)
            time.sleep(0.005)
        if self.dirCur2 != Dir2:
            self.motor2.write([0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07])             
            time.sleep(0.005)
            self.motor2.write(ordV2)                                                    
            time.sleep(0.005)
            self.motor2.write(ordDir2)                                                  
            time.sleep(0.005)
        else:
            self.motor2.write(ordV2)
            time.sleep(0.005)

    def run(self):
        publish_thread = threading.Thread(target=self.publish_data)
        publish_thread.start()
        subscribe_thread = threading.Thread(target=self.subscribe_data)
        subscribe_thread.start()


if __name__ == '__main__':
    try:
        nodePubAndSub = PubSubNode()
        nodePubAndSub.run()
    except rospy.ROSInterruptException:
        pass
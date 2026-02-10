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


# 获取所有串口设备实例。
# 如果没找到串口设备，则输出：“无串口设备。”
# 如果找到串口设备，则依次输出每个设备对应的串口号和描述信息。
def status():
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])


# 初始化串口
def init(com):
    ser = serial.Serial(port=com,                    # 端口号
                        baudrate=115200,                # 波特率
                        bytesize=serial.EIGHTBITS,      # 数据位
                        parity=serial.PARITY_NONE,      # 校验位
                        stopbits=serial.STOPBITS_ONE,   # 停止位
                        timeout=0.1                     # 响应时间范围
                        )  # 打开对应COM，将波特率等参数

    if ser.isOpen():  # 判断串口是否成功打开
        print("打开串口成功。")
        print(ser.name)  # 输出串口号
    else:
        print("打开串口失败。")

    return ser


# 计算对应的十六进制CRC校验码
def crc16Add(read):
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    data = read.replace(" ", "")
    readcrcout = hex(crc16(binascii.unhexlify(data))).upper()
    str_list = list(readcrcout)
    if len(str_list) == 5:
        str_list.insert(2, '0')
    if len(str_list) == 4:
        str_list.insert(2, '0')
        str_list.insert(2, '0')
    crc_data = "".join(str_list)
    crc_data_withoutspace = (crc_data[4:] + crc_data[2:4]).strip()
    return crc_data_withoutspace


# 将所需的速度转换为对应的指令
def vConvert(v1, v2):
    # 导轨电机速度转换, 将V导轨(mm/s)转换为V电机(r/min), 向下取整
    ph = 2          # 导轨导程为2mm, 若更换导轨, 则修改该值
    v1 = math.floor(v1 / ph * 60)
    # 捻针电机速度转换, 将r/s转换为r/min, 向下取整
    v2 = math.floor(v2 * 60)

    # 检验速度合法性, 转换后的范围应在0~3000
    if v1 < 0 or v1 > 3000 or v2 < 0 or v2 > 3000:
        print("v非法输入")
        print("速度输入范围为: , 请重新输入")
        return None

    # 指令构成: XX(站号) + XX(功能码) + XX XX(寄存器地址) + XX XX(寄存器值, 即数据) + XX XX(CRC校验码)
    # 电机1 指令转换
    if v1 == 0:
        ord1_hex = [0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34]
    else:
        v1_hex_str = "{:04X}".format(v1)[0:2] + ' ' + "{:04X}".format(v1)[2:]       # 将v1转成对应16进制字符串
        crc1_hex_str = crc16Add("01 06 00 9A" + v1_hex_str)                         # 计算该指令的crc校验码
        # 将字符串类型转换为两个字节的十进制数
        # 如: v1 = 257, v1_hex_str = "01 01", v1_hex = [1, 1]
        # 如: v1 = 255, v1_hex_str = "00 FF", v1_hex = [0, 255]
        v1_hex = [int(v1_hex_str[0:2], 16), int(v1_hex_str[3:], 16)]                # 将v1转成对应两个字节的10进制数, 注意区别与16进制字符串的区别
        crc1_hex = [int(crc1_hex_str[0:2], 16), int(crc1_hex_str[2:], 16)]          # 将crc校验码转成对应两个字节的10进制数, 同上
        ord1_hex = [0x01, 0x06, 0x00, 0x9A] + v1_hex + crc1_hex                     # 整合这条指令
    # 电机2 指令转换
    if v2 == 0:
        ord2_hex = [0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07]
    else:
        v2_hex_str = "{:04X}".format(v2)[0:2] + ' ' + "{:04X}".format(v2)[2:]       # 将v2转成对应16进制字符串
        crc2_hex_str = crc16Add("02 06 00 9A" + v2_hex_str)
        v2_hex = [int(v2_hex_str[0:2], 16), int(v2_hex_str[3:], 16)]                # 将v2转成对应两个字节的10进制数, 注意区别与16进制字符串的区别
        crc2_hex = [int(crc2_hex_str[0:2], 16), int(crc2_hex_str[2:], 16)]          # 将crc校验码转成对应两个字节的10进制数, 同上
        ord2_hex = [0x02, 0x06, 0x00, 0x9A] + v2_hex + crc2_hex

    return ord1_hex, ord2_hex


# 将所需的方向转换为对应的指令
def dirConvert(dir1, dir2):
    # 检验方向合法性, dir对应的取值应为0(前进) 或 1(后退)
    # if not (dir1 == -1 or dir1 == 1 or dir2 == -1 or dir2 == 1):
    #     print("dir非法输入")
    #     print("dir输入为0或1, 0表示后退, 1表示前进, 请重新输入")
    #     return None

    # 指令转换
    # 电机1 指令转换
    if dir1 == 1:                      # 正向启动
        ord1_hex = [0x01, 0x06, 0x00, 0xC8, 0x00, 0x01, 0xC9, 0xF4]
    elif dir1 == -1:                   # 反向启动
        ord1_hex = [0x01, 0x06, 0x00, 0xC8, 0x01, 0x01, 0xC8, 0x64]
    else:                              # 立即停止
        ord1_hex = [0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34]
    # 电机2 指令转换
    if dir2 == 1:                      # 正向启动
        ord2_hex = [0x02, 0x06, 0x00, 0xC8, 0x00, 0x01, 0xC9, 0xC7]
    elif dir2 == -1:                   # 反向启动
        ord2_hex = [0x02, 0x06, 0x00, 0xC8, 0x01, 0x01, 0xC8, 0x57]
    else:                              # 立即停止
        ord2_hex = [0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07]

    return ord1_hex, ord2_hex


# 通过速度正负获取方向
def dirGet(v):
    if math.fabs(v) < 1e-6:     # 若速度等于零, dir = 0, 表示立即停止
        return 0
    elif v > 0:                 # 若速度大于零, dir = 1, 表示前进/顺时针旋转
        return 1
    elif v < 0:                 # 若速度小于零, dir = -1, 表示后退/逆时针旋转
        return -1


# 获取当前位置
def readLocation(ser):
    com_input = ser.read_all()
    if com_input:               # 如果读取结果非空, 判断读取结果是否合法
        # print('com_input:', com_input)
        com_input = binascii.hexlify(com_input)
        # print('com_input:', com_input)
        readdata = com_input.decode(encoding='utf-8', errors='ignore')
        readdata = readdata.replace(' ', '')        # 去除空格
        # print('readdata: ', readdata)
        if len(readdata) == 18:                     # 比对指令长度, 检查读取指令的合法性
            local = eval('0x' + readdata[6:12])
            return local
        else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
            return -2
        return local
    else:                       # 若读取结果为空, 则返回-1, 表示读取失败
        return -1


# 获取当前实际速度
def readSpeed(ser):
    com_input = ser.read_all()
    if com_input:               # 如果读取结果非空, 判断读取结果是否合法
        com_input = binascii.hexlify(com_input)
        readdata = com_input.decode(encoding='utf-8', errors='ignore')
        readdata = readdata.replace(' ', '')        # 去除空格
        if len(readdata) == 14:                     # 比对指令长度, 检查读取指令的合法性
            local = eval('0x' + readdata[6:10])
            return local
        else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
            return -2
        return local
    else:                       # 若读取结果为空, 则返回-1, 表示读取失败
        return -1


# v: 单位mm/s, dir取0为前进, 取1为后退, dirCur表示指令发送前当前时刻电机的方向
# 关于速度需要有更多的注意事项:
# 1. 电机最大启动速度不超过 240r/min, 对应2mm导程导轨启动速度不超过8mm/s, 否则可能会出现卡转等现象
# 2. 电机最大运行速度不超过 3000r/min, 对应2mm导程导轨启动速度不超过100mm/s, 否则可能会出现卡转等现象
# 3. 电机最大停止速度不超过 600r/min, 对应2mm导程导轨启动速度不超过20mm/s, 否则可能会出现卡转等现象
# 4. 同理, 在加速和减速过程中, 相应的加速度不宜过大, 尤其是在负载高的情况下, 否则可能出现异常情况

class Motor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('motor_readwrite_node', anonymous=True)
        
        # 创建发布者对象
        self.pub = rospy.Publisher('/MotorRead', Float32MultiArray, queue_size=1)

        rospy.Subscriber('/MotorWrite', Float32MultiArray, self.callback, queue_size=1)

        status()
#        motor1 = input("")
#        motor2 = input("")
        motor1 = "/dev/ttyUSB0"
        motor2 = "/dev/ttyUSB2"
        print(motor1, motor2)
        self.motor1 = init(motor1)
        self.motor2 = init(motor2)

        self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])

        self.dirCur1 = 0
        self.dirCur2 = 0             # 用于记录当前方向
        self.lCur1 = 0
        self.lCur2 = 0             # 用于记录当前方向

        self.v1 = 0
        self.v2 = 0             # 用于记录发布的速度
        
        # 创建一个Float32MultiArray消息对象
        self.msg = Float32MultiArray()

        # 创建信号量
        self.semaphore = threading.Semaphore(1)
        
    def publish_data(self):
        rate = rospy.Rate(10)  # 设置发布频率为10Hz
        while not rospy.is_shutdown():

            self.semaphore.acquire()

            # 读取当前位置和实际速度
            a = readLocation(self.motor1)                                                       # 将之前接受的信息读出, 清空接收缓冲区
            b = readLocation(self.motor2)
            self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                      # 发出读取位置指令
            self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            time.sleep(0.005)                                                                   # 延时5ms
            self.lCur1 = readLocation(self.motor1)                                                        # 读取当前绝对位置, 单位1/100000圈
            self.lCur2 = readLocation(self.motor2)
            
            
#            print("here in publish data")
            self.msg.data = [self.lCur1, self.lCur2]  # 设置消息的数据
            self.pub.publish(self.msg)

            self.semaphore.release()

            rate.sleep()

    def write_data(self, Mode):
        if Mode == 0:                                                                           # 停止运行
            print("正在结束运行")
        elif Mode == 1:                                                                         # 设置绝对位置
            self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
            self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        elif Mode == 2:                                                                         # 运行模式
            # 指令处理
            Dir1 = dirGet(-self.v1)                                                                   # 通过速度正负得到方向(1, 0, -1)
            Dir2 = dirGet(self.v2)
            print("V1: %s, Dir1: %s, V2: %s, Dir2:%s" % (self.v1, Dir1, self.v2, Dir2))                   # 打印当前速度和方向
            ordV1, ordV2 = vConvert(math.fabs(self.v1), math.fabs(self.v2))                               # 速度指令转换
            ordDir1, ordDir2 = dirConvert(Dir1, Dir2)                                           # 方向指令转换

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

    def callback(self, msg):
        # 获取信号量，如果获取失败，则阻塞当前线程
        self.semaphore.acquire()
        
        self.v1 = msg.data[0]
        self.v2 = msg.data[1]
#        self.v2 = 0.5
#        print("V1: %s, V2: %s" % (self.v1, self.v2)) 
        Mode = 2

        self.write_data(Mode)
        
        # 释放信号量，允许发布数据的线程继续执行
        self.semaphore.release()

    def run(self):
        # 创建一个线程来执行发布数据的函数
        publish_thread = threading.Thread(target=self.publish_data)
        publish_thread.start()
        
        # 进入ROS循环
        rospy.spin()
    

# 负责发布消息的节点
# 发布两个电机的位置信息、力传感器的力与力矩
class MotorRead:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('motor_read_node', anonymous=True)

        # 创建发布者对象
        self.pub = rospy.Publisher('/MotorRead', Float32MultiArray, queue_size=1)

        status()
        motor1 = "/dev/ttyUSB1"
        motor2 = "/dev/ttyUSB3"
        print(motor1, motor2)
        self.motor1 = init(motor1)
        self.motor2 = init(motor2)

        self.dirCur1 = 0
        self.dirCur2 = 0             # 用于记录当前方向
        self.lCur1 = 0
        self.lCur2 = 0             # 用于记录当前方向

        self.v1 = 0
        self.v2 = 0             # 用于记录发布的速度
        
        # 创建一个Float32MultiArray消息对象
        self.msg = Float32MultiArray()

        # 创建信号量
        self.semaphore = threading.Semaphore(1)
        
    def publish_data(self):
        rate = rospy.Rate(10)  # 设置发布频率为10Hz
        while not rospy.is_shutdown():

            self.semaphore.acquire()

            # hello_str = "hello world %s" % rospy.get_time()
            # rospy.loginfo(hello_str)
            # self.pub.publish(hello_str)

            # 读取当前位置和实际速度
            a = readLocation(self.motor1)                                                       # 将之前接受的信息读出, 清空接收缓冲区
            b = readLocation(self.motor2)
            self.motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                      # 发出读取位置指令
            self.motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            time.sleep(0.005)                                                                   # 延时5ms
            self.lCur1 = readLocation(self.motor1)                                                        # 读取当前绝对位置, 单位1/100000圈
            self.lCur2 = readLocation(self.motor2)
            
            
# #            print("here in publish data")
            self.msg.data = [self.lCur1, self.lCur2]  # 设置消息的数据
            self.pub.publish(self.msg)
            print(self.lCur1, self.lCur2)

            self.semaphore.release()

            rate.sleep()

    def run(self):
        # 创建一个线程来执行发布数据的函数
        publish_thread = threading.Thread(target=self.publish_data)
        publish_thread.start()
        
        # 进入ROS循环
        rospy.spin()

class MotorWrite:
    def callback(self, msg):
        # # 获取信号量，如果获取失败，则阻塞当前线程
        # self.semaphore.acquire()
        # rospy.loginfo(rospy.get_caller_id()+"I heard %s", msg.data)     
        self.v1 = msg.data[0]
        self.v2 = msg.data[1]
        self.flag = msg.data[2]      
        # print('1')

        # print("V1: %s, V2: %s, flag: %s" % (self.v1, self.v2, self.flag)) 
        if self.flag_pre == self.flag:
            pass
        else:
            self.flag_pre = self.flag
            rospy.loginfo(msg.data)

#         Mode = 2
#         self.write_data(Mode)     
        # # 释放信号量，允许发布数据的线程继续执行
        # self.semaphore.release()
      

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('motor_write_node', anonymous=True)
        rospy.Subscriber('/MotorWrite', Float32MultiArray, self.callback, queue_size=1)
        self.flag_pre = None

        status()
        motor1 = "/dev/ttyUSB1"
        motor2 = "/dev/ttyUSB0"
        print(motor1, motor2)
        self.motor1 = init(motor1)
        self.motor2 = init(motor2)

        # self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        # self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])

        self.dirCur1 = 0
        self.dirCur2 = 0             # 用于记录当前方向
        self.lCur1 = 0
        self.lCur2 = 0             # 用于记录当前方向

        self.v1 = 0
        self.v2 = 0             # 用于记录发布的速度
        
        # # 创建一个Float32MultiArray消息对象
        # self.msg = Float32MultiArray()

        # # 创建信号量
        # self.semaphore = threading.Semaphore(1)
        rospy.spin()
        
    def write_data(self, Mode):
        if Mode == 0:                                                                           # 停止运行
            print("正在结束运行")
        elif Mode == 1:                                                                         # 设置绝对位置
            self.motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
            self.motor2.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])
        elif Mode == 2:                                                                         # 运行模式
            # 指令处理
            Dir1 = dirGet(-self.v1)                                                                   # 通过速度正负得到方向(1, 0, -1)
            Dir2 = dirGet(self.v2)
            print("V1: %s, Dir1: %s, V2: %s, Dir2:%s" % (self.v1, Dir1, self.v2, Dir2))                   # 打印当前速度和方向
            ordV1, ordV2 = vConvert(math.fabs(self.v1), math.fabs(self.v2))                               # 速度指令转换
            ordDir1, ordDir2 = dirConvert(Dir1, Dir2)                                           # 方向指令转换

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

    

if __name__ == '__main__':
    try:
        # node = Motor()
        # node.run()
        nodeRead = MotorWrite()
    except rospy.ROSInterruptException:
        pass

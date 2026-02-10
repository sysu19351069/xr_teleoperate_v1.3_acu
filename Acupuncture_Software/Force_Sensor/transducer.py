# -*- coding:utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
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
            print(comport.device, comport.description)


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


# 计算CRC校验码
def crc16_add(read):
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    data = read.replace(" ", "")
    readcrcout = hex(crc16(binascii.unhexlify(data))).upper()
    str_list = list(readcrcout)
    if len(str_list) == 5:
        str_list.insert(2, '0')
    crc_data = "".join(str_list)
    return crc_data[4:] + ' ' + crc_data[2:4]


# 两字节二进制整数正负转换
def int4bytes_convert(num, bits=32):
    num_max = pow(2, bits - 1)
    if num >= num_max:
        num = -((num ^ 0xffffffff)+1)
        return num
    else:
        return num


# 读取传感器数据: 力和扭矩
def read_data(ser):
    ser.write([0x01, 0x03, 0x01, 0xCE, 0x00, 0x04, 0x24, 0x0A])
    time.sleep(0.005)
    data_input = ser.read_all()
    if data_input:                  # 如果读取结果非空, 判断读取结果是否合法
        data_input = binascii.hexlify(data_input)
        data = data_input.decode(encoding='utf-8', errors='ignore')
        data = data.replace(' ', '')                    # 去除空格
        if len(data) == 26:                             # 比对指令长度, 检查读取指令的合法性
            f = eval('0x' + data[6:14])
            t = eval('0x' + data[14:22])
            f = int4bytes_convert(f)                     # 正负转换
            t = int4bytes_convert(t)                     # 正负转换
            return f, t, True
        else:                                           # 若校验码错误, 则表示读取数据乱码, 返回-2
            return 0, 0, False
    else:                           # 若读取结果为空, 则返回-1, 表示读取失败
        return 0, 0, False


if __name__ == '__main__':
    status()
    equip = input("")
    equip = init(equip)
    pF, pT = 0, 0
    while 1:
        F, T, S = read_data(equip)
        if S is True:
            F = F / 100 / (1.63200 * 5) * 49
            T = T / 100 / (1.25845 * 5) * 0.5
            print("F=", F, "T=", T)
        else:
            F, T = pF, pT
        pF, pT = F, T
        time.sleep(0.1)             # 调节采样频率



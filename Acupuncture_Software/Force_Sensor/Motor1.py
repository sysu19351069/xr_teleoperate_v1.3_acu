# -*- coding:utf-8 -*-
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
    if len(str_list) == 3:
        str_list.insert(2, '0')
        str_list.insert(2, '0')
        str_list.insert(2, '0')
    crc_data = "".join(str_list)
    return crc_data[4:] + ' ' + crc_data[2:4]


# 有符号十进制数转换为十六进制字符串
def f_signedIntToHex(signedInt, numBits):
    # 确保输入是一个负数
    if signedInt < 0:
        # 计算补码
        mask = (1 << numBits) - 1  # 创建一个具有指定位数的全1掩码
        complement = mask + signedInt + 1  # 计算补码
    else:
        complement = signedInt
    # 将补码转换为十六进制字符串
    hexStr = hex(complement)[2:].upper().zfill(numBits // 4)
    # 返回十六进制字符串
    return hexStr


# hex转换
def hexConvert(addr, param, numbits):
    # 改写法的缺陷：无法做到有符号数的转换，且难以保证位数不同情况下的切换
    # hex_str = "{:04X}".format(param)[0:2] + ' ' + "{:04X}".format(param)[2:]  # 将s转成对应16进制字符串
    # 改进后
    if numbits == 16:
        hex_str = f_signedIntToHex(param, 16)
        param_hex = [int(hex_str[0:2], 16), int(hex_str[2:], 16)]
    elif numbits == 32:
        param_str = f_signedIntToHex(param, 32)
        hex_str = param_str[4:8] + param_str[0:4]
        param_hex = [int(param_str[4:6], 16), int(param_str[6:8], 16), int(param_str[2:4], 16), int(param_str[0:2], 16)]
    else:
        print('error, pleas confirm the bits of input')
        return []
    crc_hex_str = crc16Add(addr + hex_str)  # 计算该指令的crc校验码
    crc_hex = [int(crc_hex_str[0:2], 16), int(crc_hex_str[3:], 16)]
    order = param_hex + crc_hex
    return order


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
        v1_hex = [int(v1_hex_str[0:2], 16), int(v1_hex_str[3:], 16)]                # 将v1转成对应两个字节的10进制数, 注意区别与16进制字符串的区别
        crc1_hex = [int(crc1_hex_str[0:2], 16), int(crc1_hex_str[3:], 16)]          # 将crc校验码转成对应两个字节的10进制数, 同上
        ord1_hex = [0x01, 0x06, 0x00, 0x9A] + v1_hex + crc1_hex                     # 整合这条指令
    # 电机2 指令转换
    if v2 == 0:
        ord2_hex = [0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07]
    else:
        v2_hex_str = "{:04X}".format(v2)[0:2] + ' ' + "{:04X}".format(v2)[2:]       # 将v2转成对应16进制字符串
        crc2_hex_str = crc16Add("02 06 00 9A" + v2_hex_str)
        v2_hex = [int(v2_hex_str[0:2], 16), int(v2_hex_str[3:], 16)]                # 将v2转成对应两个字节的10进制数, 注意区别与16进制字符串的区别
        crc2_hex = [int(crc2_hex_str[0:2], 16), int(crc2_hex_str[3:], 16)]          # 将crc校验码转成对应两个字节的10进制数, 同上
        ord2_hex = [0x02, 0x06, 0x00, 0x9A] + v2_hex + crc2_hex

    return ord1_hex, ord2_hex


# 将所需的方向转换为对应的指令
def dirConvert(dir1, dir2):
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
            local = eval('0x' + readdata[6:10]) + 2**16 * eval('0x' + readdata[10:14])
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


if __name__ == '__main__':
    status()
    motor1 = input("")
    print("the com of first motor:", motor1)
    motor2 = input("")
    print("the com of second motor:", motor2)
    motor1 = init(motor1)
    motor2 = init(motor2)


    dirCur1, dirCur2 = 0, 0             # 用于记录当前方向
    lCur1, lCur2 = 0, 0                 # 用于记录当前位置
    vCur1, vCur2 = 0, 0                 # 用于记录当前实际速度
    # v: 单位mm/s, dir取0为前进, 取1为后退, dirCur表示指令发送前当前时刻电机的方向
    # 关于速度需要有更多的注意事项:
    # 1. 电机最大启动速度不超过 240r/min, 对应2mm导程导轨启动速度不超过8mm/s, 否则可能会出现卡转等现象
    # 2. 电机最大运行速度不超过 3000r/min, 对应2mm导程导轨启动速度不超过100mm/s, 否则可能会出现卡转等现象
    # 3. 电机最大停止速度不超过 600r/min, 对应2mm导程导轨启动速度不超过20mm/s, 否则可能会出现卡转等现象
    # 4. 同理, 在加速和减速过程中, 相应的加速度不宜过大, 尤其是在负载高的情况下, 否则可能出现一场情况
    while 1:
        # 输入两台电机的转速以及操作模式
        # V1: 导轨电机速度, 单位mm/s, 浮点数, 正负决定方向
        # V2: 捻针电机速度, 单位r/s, 浮点数, 正负决定方向
        # Mode: 操作模式, 1表示设置模式, 2表示运行模式, 0表示退出
        V1, V2, Mode = map(float, input('输入V1, V2, Mode用英文逗号隔开:').split(','))              # 输入两台电机的转速以及操作模式

        if Mode == 0:                                                                           # 停止运行
            print("正在结束运行")
            break
        elif Mode == 1:                                                                         # 设置绝对位置
            print("请确认此时处于停止状态, 且该位置为起点")
            ver = input("确认设置请输入'1', 继续运行请输入'0':")
            if ver == '1':
                equipNum = input("请输入需要重置位置的设备, 1为导轨电机, 2为捻针电机: ")
                if equipNum == '1':
                    motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x2A])#0
                    # motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x7F, 0x0E])#65535
                    # motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x03, 0xE8, 0x00, 0x00, 0xFF, 0x5A])#1000
                    # motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x94, 0x00, 0x77, 0x35, 0xB4, 0xFD])#2_000_000_000
                    time.sleep(0.01)
                    motor1.write([0x01, 0x06, 0x00, 0xDC, 0x00, 0x01, 0x89, 0xF0])
                    time.sleep(0.005)
                elif equipNum == '2':
                    motor2.write([0x02, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x70, 0x6E])
                    time.sleep(0.01)
                    motor2.write([0x02, 0x06, 0x00, 0xDC, 0x00, 0x01, 0x89, 0xC3])
                    time.sleep(0.005)
                else:
                    print("非法输入, 重新运行")
                    continue
            elif ver == '0':
                continue
            else:
                print("非法输入, 结束运行")
                break
        elif Mode == 2:                                                                         # 运行模式
            # 指令处理
            Dir1 = dirGet(-V1)                                                                  # 通过速度正负得到方向(1, 0, -1)
            Dir2 = dirGet(V2)
            # print("V1: %s, Dir1: %s, V2: %s, Dir2:%s" % (V1, Dir1, V2, Dir2))                   # 打印当前速度和方向
            ordV1, ordV2 = vConvert(math.fabs(V1), math.fabs(V2))                               # 速度指令转换
            ordDir1, ordDir2 = dirConvert(Dir1, Dir2)                                           # 方向指令转换

            # 启动电机
            # 判断电机1是否需要变向
            if dirCur1 != Dir1:                                                                 # 若需要变向
                motor1.write([0x01, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x34])                  # 先停止
                time.sleep(0.005)                                                               # 写指令间隔5ms, 同下
                motor1.write(ordV1)                                                             # 设置速度
                time.sleep(0.005)
                motor1.write(ordDir1)                                                           # 按指定方向启动
                time.sleep(0.005)
            else:                                                                               # 若不需要变向
                motor1.write(ordV1)
                time.sleep(0.005)
            # 判断电机2是否需要变向
            if dirCur2 != Dir2:
                motor2.write([0x02, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x08, 0x07])                  # 先停止
                time.sleep(0.005)
                motor2.write(ordV2)                                                             # 设置速度
                time.sleep(0.005)
                motor2.write(ordDir2)                                                           # 按指定方向启动
                time.sleep(0.005)
            else:
                motor2.write(ordV2)
                time.sleep(0.005)

            # 读取当前位置和实际速度
            a = readLocation(motor1)                                                            # 将之前接受的信息读出, 清空接收缓冲区
            b = readLocation(motor2)
            motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])                      # 发出读取位置指令
            motor2.write([0x02, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xF9])
            time.sleep(0.005)  # 延时5ms
            lCur1 = readLocation(motor1)  # 读取当前绝对位置, 单位1/100000圈
            lCur2 = readLocation(motor2)
            motor1.write([0x01, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xCD])                      # 发出读取速度指令
            motor2.write([0x02, 0x03, 0x00, 0x19, 0x00, 0x01, 0x55, 0xFE])
            time.sleep(0.005)
            vCur1 = readSpeed(motor1)                                                           # 读取当前实际速度, 单位r/min
            vCur2 = readSpeed(motor2)
            print("lCur1: ", lCur1, "vCur1: ", vCur1)
            print("lCur2: ", lCur2, "vCur2: ", vCur2)

            # 打印指令, 用于程序测试，正常使用可直接注释
            # print("V1: %s, ordV1: %s, Dir1: %s, ordDir1: %s, dirCur1:%s" % (V1, ordV1, Dir1, ordDir1, dirCur1))
            # print("V2: %s, ordV2: %s, Dir2: %s, ordDir2: %s, dirCur2:%s" % (V2, ordV2, Dir2, ordDir2, dirCur2))
            dirCur1 = Dir1  # 更新当前方向状态
            dirCur2 = Dir2  

        elif Mode == 3: # 设置电机1
            # 设置正限位
            motor1.write([0x01, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x5F, 0x90, 0x00, 0x01, 0x27, 0x72])#正限位90000
            time.sleep(0.01)

            # 设置负限位
            motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x74, 0x0B])#负限位0
            time.sleep(0.01)

            # 设置当前位置
            motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0xB2, 0x62])#80000
            time.sleep(0.01)

        elif Mode == 4: # 设置电机2
            # 设置正限位
            motor2.write([0x02, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x4E, 0x20, 0x00, 0x00, 0xED, 0x2D])#正限位20000
            time.sleep(0.01)

            # 设置负限位
            motor2.write([0x02, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x4F])#负限位0
            time.sleep(0.01)

            # 设置当前位置
            motor2.write([0x02, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0x27, 0x10, 0x00, 0x00, 0x7B, 0x1F])#10000
            time.sleep(0.01)        
        
        elif Mode == 5: # 调试使用
            # # 读取分圈??
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # print(a)
            # motor1.write([0x01, 0x06, 0x00, 0x07, 0x0F, 0xA0, 0x3D, 0x83]) 
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # print(a)
            # motor1.write([0x01, 0x03, 0x00, 0x07, 0x00, 0x01, 0x35, 0xCB]) 
            # time.sleep(0.05) 
            # Pulse1 = readPulse(motor1) 
            # print("Pulse1:", Pulse1)


            # 读取正限位
            a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            motor1.write([0x01, 0x03, 0x00, 0x70, 0x00, 0x02, 0xC5, 0xD0]) 
            time.sleep(0.005) 
            com_input = motor1.read_all()
            if com_input:               # 如果读取结果非空, 判断读取结果是否合法
                com_input = binascii.hexlify(com_input)
                readdata = com_input.decode(encoding='utf-8', errors='ignore')
                readdata = readdata.replace(' ', '')        # 去除空格
                if len(readdata) == 18:                     # 比对指令长度, 检查读取指令的合法性
                    local = eval('0x' + readdata[6:10]) + 2**16 * eval('0x' + readdata[10:14])
                    print(local)
                else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
                    print('-2')
            else:                       # 若读取结果为空, 则返回-1, 表示读取失败
                print('-1')

            # # 设置正限位
            # motor1.write([0x01, 0x10, 0x00, 0x70, 0x00, 0x02, 0x04, 0x20, 0x00, 0x00, 0x01, 0x3E, 0x8B])#正限位73728
            # time.sleep(0.01)
            # motor1.write([0x01, 0x06, 0x00, 0xDC, 0x00, 0x01, 0x89, 0xF0])
            # time.sleep(0.005)

            # 读取负限位
            a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            motor1.write([0x01, 0x03, 0x00, 0x6E, 0x00, 0x02, 0xA5, 0xD6]) 
            time.sleep(0.005) 
            com_input = motor1.read_all()
            if com_input:               # 如果读取结果非空, 判断读取结果是否合法
                com_input = binascii.hexlify(com_input)
                readdata = com_input.decode(encoding='utf-8', errors='ignore')
                readdata = readdata.replace(' ', '')        # 去除空格
                if len(readdata) == 18:                     # 比对指令长度, 检查读取指令的合法性
                    local = eval('0x' + readdata[6:10]) + 2**16 * eval('0x' + readdata[10:14])
                    print(local)
                else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
                    print('-2')
            else:                       # 若读取结果为空, 则返回-1, 表示读取失败
                print('-1')

            # 设置负限位
            # motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x74, 0x0B])#负限位0
            # motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x27, 0x10, 0x00, 0x00, 0x7F, 0x7A])#负限位10000
            # motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0x00, 0x00, 0x80, 0x00, 0x15, 0xCB])#取消负限位
            # motor1.write([0x01, 0x10, 0x00, 0x6E, 0x00, 0x02, 0x04, 0xD8, 0xF0, 0xFF, 0xFF, 0x4F, 0x28])#负限位-10000
            # time.sleep(0.01)
            # motor1.write([0x01, 0x06, 0x00, 0xDC, 0x00, 0x01, 0x89, 0xF0])
            # time.sleep(0.005)


            # # 设置当前位置
            # motor1.write([0x01, 0x10, 0x00, 0xD2, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x7F, 0x0E])#65535
            # time.sleep(0.01)
            # motor1.write([0x01, 0x06, 0x00, 0xDC, 0x00, 0x01, 0x89, 0xF0])
            # time.sleep(0.005)

            # # 基于位置进行运动
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])  # 发出读取位置指令
            # time.sleep(0.005)  # 延时5ms
            # lCur1 = readLocation(motor1)  # 读取当前绝对位置
            # print("lCur1 pre:", lCur1)
            # # # 运行到绝对位置为5000脉冲数
            # # motor1.write([0x01, 0x10, 0x00, 0xD0, 0x00, 0x02, 0x04, 0x13, 0x88, 0x00, 0x00, 0x7A, 0x5D])
            # # # 相对当前位置运行5000脉冲数
            # # motor1.write([0x01, 0x10, 0x00, 0xDE, 0x00, 0x02, 0x04, 0x13, 0x88, 0x00, 0x00, 0xFB, 0xD1])
            # # 运行到绝对位置为1000脉冲数
            # motor1.write([0x01, 0x10, 0x00, 0xD0, 0x00, 0x02, 0x04, 0x03, 0xE8, 0x00, 0x00, 0x7E, 0x83])
            # motor1.write([0x01, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0x31, 0x09]) # 绝对位置80000
            # time.sleep(0.1)

            # # 读取额定电流
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # motor1.write([0x01, 0x03, 0x00, 0x0D, 0x00, 0x01, 0x15, 0xC9]) 
            # time.sleep(0.005) 
            # com_input = motor1.read_all()
            # if com_input:               # 如果读取结果非空, 判断读取结果是否合法
            #     com_input = binascii.hexlify(com_input)
            #     readdata = com_input.decode(encoding='utf-8', errors='ignore')
            #     readdata = readdata.replace(' ', '')        # 去除空格
            #     if len(readdata) == 14:                     # 比对指令长度, 检查读取指令的合法性
            #         local = eval('0x' + readdata[6:10]) 
            #         print(local)
            #     else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
            #         print('-2')
            # else:                       # 若读取结果为空, 则返回-1, 表示读取失败
            #     print('-1')

            # # 读取当前实际运行电流
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # motor1.write([0x01, 0x03, 0x00, 0x1A, 0x00, 0x01, 0xA5, 0xCD]) 
            # time.sleep(0.005) 
            # com_input = motor1.read_all()
            # if com_input:               # 如果读取结果非空, 判断读取结果是否合法
            #     com_input = binascii.hexlify(com_input)
            #     readdata = com_input.decode(encoding='utf-8', errors='ignore')
            #     readdata = readdata.replace(' ', '')        # 去除空格
            #     if len(readdata) == 14:                     # 比对指令长度, 检查读取指令的合法性
            #         local = eval('0x' + readdata[6:10]) 
            #         print(local)
            #     else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
            #         print('-2')
            # else:                       # 若读取结果为空, 则返回-1, 表示读取失败
            #     print('-1')

            # # 读取启动速度
            # a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            # motor1.write([0x01, 0x03, 0x00, 0x96, 0x00, 0x01, 0x64, 0x26]) 
            # time.sleep(0.005) 
            # com_input = motor1.read_all()
            # if com_input:               # 如果读取结果非空, 判断读取结果是否合法
            #     com_input = binascii.hexlify(com_input)
            #     readdata = com_input.decode(encoding='utf-8', errors='ignore')
            #     readdata = readdata.replace(' ', '')        # 去除空格
            #     if len(readdata) == 14:                     # 比对指令长度, 检查读取指令的合法性
            #         local = eval('0x' + readdata[6:10]) 
            #         print(local)
            #     else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
            #         print('-2')
            # else:                       # 若读取结果为空, 则返回-1, 表示读取失败
            #     print('-1')

            # 读取电机运行速度
            a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            motor1.write([0x01, 0x03, 0x00, 0x9A, 0x00, 0x01, 0xA4, 0x25]) 
            time.sleep(0.005) 
            com_input = motor1.read_all()
            if com_input:               # 如果读取结果非空, 判断读取结果是否合法
                com_input = binascii.hexlify(com_input)
                readdata = com_input.decode(encoding='utf-8', errors='ignore')
                readdata = readdata.replace(' ', '')        # 去除空格
                print(readdata)
                if len(readdata) == 14:                     # 比对指令长度, 检查读取指令的合法性
                    local = eval('0x' + readdata[6:10]) 
                    print(local)
                else:                                       # 若校验码错误, 则表示读取数据乱码, 返回-2
                    print('-2')
            else:                       # 若读取结果为空, 则返回-1, 表示读取失败
                print('-1')

            # # 恢复出厂设置
            # motor1.write([0x01, 0x06, 0x00, 0xDC, 0x00, 0x00, 0x48, 0x30])
            # time.sleep(1)
        
        elif Mode == 6:
            # 基于位置进行运动
            a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])  # 发出读取位置指令
            time.sleep(0.005)  # 延时5ms
            lCur1 = readLocation(motor1)  # 读取当前绝对位置
            print("lCur1 pre:", lCur1)
            motor1.write([0x01, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0x11, 0x70, 0x00, 0x01, 0x39, 0x66]) # 绝对位置70000
            time.sleep(0.1)
        
        elif Mode == 7:
            # 基于位置进行运动
            a = readLocation(motor1)  # 将之前接受的信息读出, 清空接收缓冲区
            motor1.write([0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x85, 0xCA])  # 发出读取位置指令
            time.sleep(0.005)  # 延时5ms
            lCur1 = readLocation(motor1)  # 读取当前绝对位置
            print("lCur1 pre:", lCur1)
            motor1.write([0x01, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0x38, 0x80, 0x00, 0x01, 0x31, 0x09])  # 绝对位置80000
            time.sleep(0.1)

        elif Mode == 8:
            # 运行目标速度设置
            motor1.write([0x01, 0x06, 0x00, 0x9A, 0x00, 0x0F, 0x19, 0xE1]) 
            time.sleep(0.005)  # 延时5ms


        



    print("已停止运行")
    # CW顺时针 CCW逆时针

#!/usr/bin/env python
# coding:utf-8
# UI界面的节点

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
import time
import numpy as np

# 发布针刺消息
def talkerStick(indexStick, distanceStick, vStick):
    try:
        rospy.init_node('ui_node', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException as e:
        pass

    location1_pre = rospy.wait_for_message("/LocAndFT", Float32MultiArray, timeout=None)
    location1_pre = location1_pre.data[0]
    
    if indexStick == 0:
        indexStick = -1
    location1_desired = location1_pre + indexStick*distanceStick*2000
    print(location1_desired)
        
    v1_initial = vStick
    v1_desired = -np.sign(indexStick)*v1_initial # v>0,plug
    v2_desired = 0

    for num in range(5):
        pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)
        array = [v1_desired, v2_desired]
        msg = Float32MultiArray(data=array)
        rospy.loginfo(msg.data)
        pub.publish(msg)
        time.sleep(0.05)
    
    while(1):
        location1_current = rospy.wait_for_message("/LocAndFT", Float32MultiArray, timeout=None)
        location1_current = location1_current.data[0]
        if abs(location1_current-location1_desired) < 500:
            for num in range(5):
                pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)
                array = [0, 0]
                msg = Float32MultiArray(data=array)
                rospy.loginfo(msg.data)
                pub.publish(msg)
                time.sleep(0.05)
            break

# 发布捻针消息
def talkerRotate(indexRotate, circleRotate, vRotate):
    try:
        rospy.init_node('ui_node', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException as e:
        pass

    location2_pre = rospy.wait_for_message("/LocAndFT", Float32MultiArray, timeout=None)
    location2_pre = location2_pre.data[1]
    
    if indexRotate == 0:
        indexRotate = -1
    location2_desired = location2_pre + indexRotate*circleRotate*4000
    print(location2_desired)
        
    v2_initial = vRotate
    v2_desired = np.sign(indexRotate)*v2_initial # v<0,Positive
    v1_desired = 0

    for num in range(5):
        pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)
        array = [v1_desired, v2_desired]
        msg = Float32MultiArray(data=array)
        rospy.loginfo(msg.data)
        pub.publish(msg)
        time.sleep(0.05)
    
    while(1):
        location2_current = rospy.wait_for_message("/LocAndFT", Float32MultiArray, timeout=None)
        location2_current = location2_current.data[1]
        if abs(location2_current-location2_desired) < 500:
            for num in range(5):
                pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)
                array = [0, 0]
                msg = Float32MultiArray(data=array)
                rospy.loginfo(msg.data)
                pub.publish(msg)
                time.sleep(0.05)
            break

# 发布电机停止运动消息
def talkerMotorStop():
    try:
        rospy.init_node('ui_node', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException as e:
        pass
    for num in range(5):
        pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)
        array = [0, 0]
        msg = Float32MultiArray(data=array)
        rospy.loginfo(msg.data)
        pub.publish(msg)
        time.sleep(0.05)

# 用于发布针刺相关命令
def talkerStabAndStop(flag):
    try:
        rospy.init_node('ui_node', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException as e:
        pass

    # # 针刺全过程
    # if flag == 1:
    #     talkerStick(0, 20, 5)

    #     talkerStick(1, 3, 0.5)
    #     talkerStick(0, 3, 0.5)

    #     talkerRotate(0, 0.5, 0.2)
    #     talkerRotate(1, 0.5, 0.2)

    #     # for i in range(2):
    #     #     talkerStick(1, 3, 0.5)
    #     #     talkerStick(0, 3, 0.5)

    #     # for i in range(2):
    #     #     talkerRotate(0, 0.5, 0.2)
    #     #     talkerRotate(1, 0.5, 0.2)

    #     talkerStick(1, 20, 5)

    # # 停止针刺与捻针
    # elif flag == 0:
    #     talkerMotorStop()

    # 复位
    # elif flag == -1:
    #     for num in range(2):
    #         pub = rospy.Publisher('/MovePos', Float32MultiArray, queue_size=1)
    #         msg = Float32MultiArray(data=[-1])
    #         rospy.loginfo(msg.data)
    #         pub.publish(msg)
    #         time.sleep(0.05)

    if flag == 1:
        pub = rospy.Publisher('/Run', Bool, queue_size=1)
        msg = Bool(data=1)
        rospy.loginfo(msg.data)
        pub.publish(msg)

        time.sleep(0.5)
        
    elif flag == 0:
        pub = rospy.Publisher('/Run', Bool, queue_size=1)
        msg = Bool(data=0)
        rospy.loginfo(msg.data)
        pub.publish(msg)
        time.sleep(0.1)

        talkerMotorStop()
    # 复位
    elif flag == -1:
        for num in range(2):
            pub = rospy.Publisher('/MovePos', Float32MultiArray, queue_size=1)
            msg = Float32MultiArray(data=[-1])
            rospy.loginfo(msg.data)
            pub.publish(msg)
            time.sleep(0.05)





# # 用于发布针刺相关命令
# # 参数：标志参数、目标穴位、针刺手法
# def talkerStabAndStop(flag, indexAcu=-1, indexNe=-1):
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     pub = rospy.Publisher('/StabAndStop', Float32MultiArray, queue_size=1)

#     # flag：1为开始针刺，0为停止针刺，-1为复位
#     # indexAcu：0为天枢，1为大横，2为上巨虚，3为足三里
#     # indexNe：0为直刺，1为斜刺，2为平刺
#     # 发布一定数量的信息
#     for num in range(2):
#         array = [0, 0, flag, indexAcu, indexNe]
#         msg = Float32MultiArray(data=array)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)
#         time.sleep(0.5)


# # 用于收取机械臂关节角
# def listenerRobotAngles():
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass

#     # 只订阅一次目标话题的机械臂关节角信息
#     angles = rospy.wait_for_message("/RobotAngles", Float32MultiArray, timeout=None)
#     angles = angles.data
#     return angles
    
# # 用于发布机械臂关节角
# # 参数：机械臂6个关节角
# def talkerRobotAngles(angles):
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     pub = rospy.Publisher('/UIRobotAngles', Float32MultiArray, queue_size=1)

#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         msg = Float32MultiArray(data=angles)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)
#         rate.sleep()

# # 用于发布停止机械臂运动信息
# def talkerStopRobot():
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     pub = rospy.Publisher('/UIStopRobot', Bool, queue_size=1)

#     rate = rospy.Rate(1)
#     while not rospy.is_shutdown():
#         msg = Bool(data=0)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)
#         rate.sleep()


# # 发布针刺消息
# def talkerStick(indexStick, distanceStick):
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     for num in range(2):
#         pub = rospy.Publisher('/Stick', Float32MultiArray, queue_size=1)
#         array = [indexStick, distanceStick]
#         msg = Float32MultiArray(data=array)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)

#         pub1 = rospy.Publisher('/Run', Bool, queue_size=1)
#         msg1 = Bool(data=1)
#         rospy.loginfo(msg.data)
#         pub1.publish(msg1)

#         time.sleep(0.5)

#     # rate = rospy.Rate(1)
#     # while not rospy.is_shutdown():
#     #     array = [indexStick, distanceStick]
#     #     msg = Float32MultiArray(data=array)
#     #     rospy.loginfo(msg.data)
#     #     pub.publish(msg)
#     #     rate.sleep()

# # 发布捻针消息
# def talkerRotate(indexRotate, circleRotate):
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     for num in range(2):
#         pub = rospy.Publisher('/Rotate', Float32MultiArray, queue_size=1)
#         array = [indexRotate, circleRotate]
#         msg = Float32MultiArray(data=array)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)

#         pub1 = rospy.Publisher('/Run', Bool, queue_size=1)
#         msg1 = Bool(data=1)
#         rospy.loginfo(msg.data)
#         pub1.publish(msg1)

#         time.sleep(2)

# # 发布电机停止运动消息
# def talkerMotorStop():
#     try:
#         rospy.init_node('ui_node', anonymous=True, disable_signals=True)
#     except rospy.exceptions.ROSException as e:
#         pass
#     for num in range(2):
#         pub = rospy.Publisher('/Run', Bool, queue_size=1)
#         msg = Bool(data=0)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)

#         time.sleep(0.5)



if __name__ == '__main__':
    try:
        pass
    except rospy.ROSInterruptException:
        pass
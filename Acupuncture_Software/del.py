#  designer 运行路径
#  /usr/lib/x86_64-linux-gnu/qt5/bin/designer 


binnum = bin(65535)
print(binnum)
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
elif binstr[0] == '1':
    a = int(binstr, base=2)
    binstr = binstr.replace('_', '')
    result = a - 2**len(binstr)
else:
    print('error')
print(result)

# import rospy
# from std_msgs.msg import String, Float32MultiArray


# def talker():
#     rospy.init_node('talker_node', anonymous=True)
#     pub = rospy.Publisher('/MotorV', Float32MultiArray, queue_size=1)

#     rate = rospy.Rate(2)
#     while not rospy.is_shutdown():
#         array = [0.5, 0.5, 1, 1, 1, 1]
#         msg = Float32MultiArray(data=array)
#         rospy.loginfo(msg.data)
#         pub.publish(msg)
#         rate.sleep()

# def callback(msg):
#     rospy.loginfo(msg.data) 
      

# def listener():
#     # 初始化ROS节点
#     rospy.init_node('listener_node', anonymous=True)
#     rospy.Subscriber('/LocAndF', Float32MultiArray, callback, queue_size=1)
#     rospy.spin()


# if __name__ == '__main__':
#     try:
#         talker()
#         # listener()
#     except rospy.ROSInterruptException:
#         pass





    # img = cv2.imread('/home/ros/Acupuncture_Robot_Software/test_img.jpg')

    # #图片路径
    # a =[]
    # b = []
    # def on_EVENT_LBUTTONDOWN(event, x, y,flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         xy = "%d,%d" % (x, y)
    #         a.append(x)
    #         b.append(y)
    #         cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
    #         cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
    #                     1.0, (255, 255, 255), thickness=1)
    #         cv2.imshow("image", img)
    
    # cv2.namedWindow("image")
    # cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    # cv2.imshow("image", img)
    # cv2.waitKey(0)
    # print(a,b)

    # # 在彩色图像的情况下，解码图像将以b g r顺序存储通道。
    # grid_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
 
    # # 从RGB色彩空间转换到HSV色彩空间
    # grid_HSV = cv2.cvtColor(grid_RGB, cv2.COLOR_RGB2HSV)
 
    # # H、S、V范围一：
    # lower1 = np.array([0,43,46])
    # upper1 = np.array([10,255,255])
    # mask1 = cv2.inRange(grid_HSV, lower1, upper1)       # mask1 为二值图像
    # res1 = cv2.bitwise_and(grid_RGB, grid_RGB, mask=mask1)
 
    # # H、S、V范围二：
    # lower2 = np.array([156,43,46])
    # upper2 = np.array([180,255,255])
    # mask2 = cv2.inRange(grid_HSV, lower2, upper2)
    # res2 = cv2.bitwise_and(grid_RGB,grid_RGB, mask=mask2)
 
    # # 将两个二值图像结果 相加
    # mask3 = mask1 + mask2
    
    # # 结果显示
    # # cv2.imshow("mask3", mask3)
    # # cv2.imshow("img",img)
    # # cv2.imshow("Mask1",mask1)
    # # cv2.imshow("res1",res1)
    # # cv2.imshow("Mask2",mask2)
    # # cv2.imshow("res2",res2)
    # # cv2.imshow("grid_RGB", grid_RGB[:,:,::-1])           # imshow()函数传入的变量也要为b g r通道顺序
    # # cv2.waitKey(0)
    # # cv2.destroyAllWindows()

    
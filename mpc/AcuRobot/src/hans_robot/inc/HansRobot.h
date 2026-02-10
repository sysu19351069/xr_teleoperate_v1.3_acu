#ifndef HANSROBOT
#define HANSROBOT

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <string>

using namespace std;

class HansRobot
{
public:
    int HR_Connect(int boxID, const char* hostName, unsigned short nPort);  //连接控制器
    int HR_DisConnect(); //断开控制器连接
    int HR_IsConnected();          //查看控制器是否连接
    int HR_ShutdownRobot();      //关机

    int HR_ReadActPos(double Base[6], double Joint[6], double Tcp[6], double Ucs[6]);      //读取当前位置
    int HR_Quaternion2RPY(int boxID, int rbtID, double w, double x, double y, double z, double rpy[3]);        //四元素转欧拉角
    int HR_RPY2Quaternion(int boxID, int rbtID, double dx, double dy, double dz, double qua[4]);       //欧拉角转四元素

    int HR_MoveJ(int boxID, int rbtID, double Joint[6], double TCP[6], string TcpName, string UcsName, double vel, 
                    double Acc, double r, double IsUseJoint, double Iseek, double IOBit, double IOState, string ID);     //关节运动

};

#endif

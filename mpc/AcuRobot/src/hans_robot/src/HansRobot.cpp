#include <iostream>
// #include "../inc/testSo.h"
#include "HR_Pro.h"
#include "HansRobot.h"
#include <vector>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
string Msg;
int nRet;
#define err(Msg)   printf("%s调用失败! 错误码为：%d, 报错行数: %d\n",Msg,nRet,__LINE__)

int HansRobot::HR_Connect(int boxID, const char* hostName, unsigned short nPort)
{
    int nRet = HRIF_Connect(boxID, hostName, nPort);
    if (nRet != 0)
    {
        err("HRIF_Connect");
        return nRet;
    }
    cout << "HRIF_Connect调用成功!" << endl;
    return nRet;
}

int HansRobot::HR_DisConnect()
{
    int nRet = HRIF_DisConnect(0);
    if (nRet != 0)
    {
        err("HRIF_DisConnect");
        return nRet;
    }
    cout << "HRIF_DisConnect调用成功!" << endl;
    return nRet;
}

int HansRobot::HR_IsConnected()
{
    bool nRet = HRIF_IsConnected(0);
    cout << "HRIF_IsConnected调用成功!" << endl;
    cout <<"连接状态：" << nRet << endl;
    return nRet;
}

int HansRobot::HR_ShutdownRobot()
{
    int nRet = HRIF_ShutdownRobot(0);
    if (nRet != 0)
    {
        err("HRIF_ShutdownRobot");
        return nRet;
    }
    cout << "HRIF_ShutdownRobot调用成功!" << endl;
    return nRet;
}

int HansRobot::HR_ReadActPos(double Base[6], double Joint[6], double Tcp[6], double Ucs[6])
{
    int nRet = HRIF_ReadActPos(0,0, Base[0], Base[1], Base[2], Base[3], Base[4], Base[5], Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], Tcp[0], Tcp[1], Tcp[2], Tcp[3], Tcp[4]
        , Tcp[5], Ucs[0], Ucs[1], Ucs[2], Ucs[3], Ucs[4], Ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActPos");
        return nRet;
    }
    cout << "HRIF_ReadActPos调用成功!" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Base[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Joint[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Tcp[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Ucs[i] << " ";
    }
    cout << endl;
    return nRet;
}

int HansRobot::HR_Quaternion2RPY(int boxID, int rbtID, double w, double x, double y, double z, double rpy[3])
{
    int nRet = HRIF_Quaternion2RPY(boxID,0, w, x, y, z, rpy[0], rpy[1], rpy[2]);
    if (nRet != 0)
    {
        err("HRIF_Quaternion2RPY");
        return nRet;
    }
    cout << "HRIF_Quaternion2RPY调用成功!" << endl;
    cout << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;
    return nRet;
}

int HansRobot::HR_RPY2Quaternion(int boxID, int rbtID, double dx, double dy, double dz, double qua[4])
{
    int nRet = HRIF_RPY2Quaternion(boxID, 0, dx, dy, dz, qua[0], qua[1], qua[2], qua[3]);
    if (nRet != 0)
    {
        err("HRIF_RPY2Quaternion");
        return nRet;
    }
    cout << "HRIF_RPY2Quaternion调用成功!" << endl;
    cout << qua[0] << " " << qua[1] << " " << qua[2] << " " << qua[3] << endl;
    return nRet;
}

int HansRobot::HR_MoveJ(int boxID, int rbtID, double Joint[6], double TCP[6], string TcpName, string UcsName, double Vel, 
                    double Acc, double r, double IsUseJoint, double Iseek, double IOBit, double IOState, string ID)
{
    int nRet = HRIF_MoveJ(0, 0,TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5], Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    if (nRet != 0)
    {
        err("HRIF_MoveJ");
        return nRet;
    }
    cout << "HRIF_MoveJ调用成功!" << endl;
    return nRet;
}

#include <iostream>
// #include "../inc/testSo.h"
#include <vector>

#include "HR_Pro.h"

#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
string Msg;
int nRet;
#define err(Msg)   printf("%s调用失败！错误码为：%d,报错行数：%d\n",Msg,nRet,__LINE__)


/*class InitRobot
{
public:
    int HR_Connect();  //连接控制器
    int HR_DisConnect(); //断开控制器连接
    int HR_IsConnected();          //查看控制器是否连接
    int HR_ShutdownRobot();      //关机
    int HR_Connect2Box();     //连接控制板
    int HR_Electrify();      //机器人上电
    int HR_Blackout();       //机器人断电
    int HR_Connect2Controller();   //连接机器人
    int HR_IsSimulateRobot();            //查看是否为模拟机器人
    int HR_IsControllerStarted();        //控制器是否启动完成
    int HR_ReadVersion();                   //读取版本号
    int HR_ReadRobotModel();                //读取机器人型号    
};*/


//初始化控制命令
int HR_Connect()
{
    string strCPSIP("192.168.15.24");
    unsigned short nCmdPort = 10003;
    int boxID = 0;
    //int nRet = _Z12HRIF_ConnectjPKct(boxID, strCPSIP.c_str(), nCmdPort);
    int nRet = HRIF_Connect(boxID, strCPSIP.c_str(), nCmdPort);
    
    if (nRet != 0)
    {
        err("HRIF_Connect");
        return nRet;
    }
    cout << "HRIF_Connect调用成功！" << endl;
    return nRet;
}

int main()
{
   HR_Connect();
    return 0;
}

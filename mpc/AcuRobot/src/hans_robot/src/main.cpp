#include <iostream>
// #include "../inc/testSo.h"
#include "HR_Pro.h"
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
#define err(Msg)   printf("%s调用失败！错误码为：%d,报错行数：%d\n",Msg,nRet,__LINE__)


class InitRobot
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
};

class AxisControl
{
public:
    int HR_GrpEnable();   //使能机器人
    int HR_GrpDisable();   //去使能机器人
    int HR_GrpReset();    //复位机器人
    int HR_GrpStop();     //停止机器人运动
    int HR_GrpInterrupt();         //暂停运动命令
    int HR_GrpContinue();          //继续运动命令
    int HR_GrpOpenFreeDriver();    //开启零力示教
    int HR_GrpCloseFreeDriver();    //关闭零力示教
};

class ScriptControl
{
public:
    int HR_RunFunc();   //运行指定函数
    int HR_StartScript();   //运行main函数
    int HR_StopScript();     //停止脚本
    int HR_PauseScript();    //暂停脚本
    int HR_ContinueScript();   //继续脚本
};

class BoxControl
{
public:
    int HR_ReadBoxInfo();  //读取电箱信息
    int HR_ReadBoxCI();    //读取CI状态
    int HR_ReadBoxDI();     //读取DI状态
    int HR_ReadBoxCO();      //读取CO状态
    int HR_ReadBoxDO();      //读取DO状态
    int HR_ReadBoxAI();      //读取AI状态
    int HR_ReadBoxAO();      //读取AO状态
    int HR_SetBoxCO();      //设置CO
    int HR_SetBoxDO();     //设置DO
    int HR_SetBoxAOMode();   //设置AO输出模式
    int HR_SetBoxAOVal();      //设置AO输出值
    int HR_SetEndDO();          //设置末端输出
    int HR_ReadEndDI();         //读取末端输入
    int HR_ReadEndDO();         //读取末端输出
    int HR_ReadEndAI();           //读取末端模拟量输入
    int HR_ReadEndBTN();           //读取末端按键状态
};

class ReadAndSet
{
public:
    int HR_SetOverride();         //设置速度比
    int HR_SetTCPMotion();             //设置Tool坐标模式
    int HR_SetPayload();                  //设置当前负载
    int HR_SetJointMaxVel();                  //设置关节最大速度
    int HR_SetJointMaxAcc();                //设置关节最大加速度
    int HR_SetLinearMaxVel();              //设置直线最大速度
    int HR_SetLinearMaxAcc();                     //设置直线最大加速度
    int HR_SetMaxAcsRange();                 //设置关节最大范围
    int HR_SetMaxPcsRange();                //设置空间最大范围
    int HR_ReadJointMaxVel();                    //读取关节最大运动速度
    int HR_ReadJointMaxAcc();                        //读取关节最大运动加速度
    int HR_ReadJointMaxJerk();                         //读取关节最大运动加加速度
    int HR_ReadLinearMaxSpeed();                      //读取直线最大速度参数
    int HR_ReadEmergencyInfo();                     //读取急停信息
    int HR_ReadRobotState();                        //读取机器人当前状态
    int HR_ReadCurWaypointID();                             //读取当前路点运动ID
    int HR_ReadAxisErrorCode();                       //读取轴错误码
    int HR_ReadCurFSM();                           //读取当前状态机
    int HR_ReadCurFSMFromCPS();                       //读取当前状态机
    int HR_ReadRobotFlags();
};

class Positon_Cur_Speed
{
public:
    int HR_ReadActPos();                 //读取当前位置
    int HR_ReadCmdJointPos();                  //读取关节命令位置
    int HR_ReadActJointPos();                //读取关节实际位置
    int HR_ReadCmdTcpPos();                       //读取TCP命令位置
    int HR_ReadActTcpPos();                         //读取TCP实际位置
    int HR_ReadCmdJointVel();                       //读取关节命令速度
    int HR_ReadActJointVel();                     //读取关节实际速度
    int HR_ReadCmdTcpVel();                       //读取TCP命令速度
    int HR_ReadActTcpVel();                           //读取TCP实际速度
    int HR_ReadCmdJointCur();                       //读取关节命令电流
    int HR_ReadActJointCur();                        //读取关节实际电流
    int HR_ReadTcpVelocity();                            //读取TCP末端速度
};


class PoseCal
{
public:
    int HR_Quaternion2RPY();                  //四元素转欧拉角
    int HR_RPY2Quaternion();                     //欧拉角转四元素
    int HR_GetInverseKin();                       //逆解，由指定用户坐标系位置和工具坐标系下的迪卡尔坐标计算对应的关节坐标位置
    int HR_GetForwardKin();                           //正解，由关节坐标位置计算指定用户坐标系和工具坐标系下的迪卡尔坐标位置
    int HR_Base2UcsTcp();                          //由基坐标系下的坐标位置计算指定用户坐标系和工具坐标系下的迪卡尔坐标位置
    int HR_UcsTcp2Base();                            //由指定用户坐标系和工具坐标系下的迪卡尔坐标位置计算基坐标系下的坐标位置
    int HR_PoseAdd();                                //点位加法计算
    int HR_PoseSub();                                   //点位减法计算
    int HR_PoseTrans();                                  //坐标变换
    int HR_PoseInverse();                                     //坐标逆变换
    int HR_PoseDist();                                       //计算点位距离
    int HR_PoseInterpolate();                                    //空间位置直线插补计算
    int HR_PoseDefdFrame();                                         //以轨迹中心旋转计算，p1,p2,p3 为旋转前轨迹的特征点，p4,p5,p6 为旋转后的轨迹的特征点
};

class TcpAndUcs
{
public:
    int HR_SetTCP();                                //设置当前工具坐标，不写入配置文件，重启后失效
    int HR_SetUCS();                                  //设置当前用户坐标，不写入配置文件，重启后失效
    int HR_ReadCurTCP();                                  //读取当前设置的工具坐标值
    int HR_ReadCurUCS();                                         //读取当前设置的用户坐标值
    int HR_SetTCPByName();                                     //通过名称设置工具坐标列表中的值为当前工具坐标，对应名称为示教器配置页面 TCP 示教的工具名称
    int HR_SetUCSByName();                                      //通过名称设置用户坐标列表中的值为当前用户坐标，对应名称为示教器配置页面用户坐标示教的名称
    int HR_ReadTCPByName();                                         //通过名称读取指定 TCP 坐标，对应名称为示教器配置页面 TCP 示教的工具名称
    int HR_ReadUCSByName();                                      //通过名称读取指定 UCS 坐标，对应名称为示教器配置页面用户坐标示教的用户坐标名称
};

class ForceControl
{
public:
    int HR_GetForceParams();
    int HR_SetForceParams();
    int HR_SetForceControlParams();                   //设置主动探寻参数
    int HR_SetForceMode();                            //设置开启或者关闭力控模式
    int HR_SetForceControlState();                //设置力控状态
    int HR_ReadForceControlState();                 //读取当前力控状态
    int HR_SetForceToolCoordinateMotion();                 //设置力控坐标系方向为 tool 坐标方向模式
    int HR_ForceControlInterrupt();                      //暂停力控运动，仅暂停力控功能，不暂停运动和脚本
    int HR_ForceControlContinue();                           //继续力控运动，仅继续力控运动功能，不继续运动和脚本
    int HR_SetForceZero();                                    //力控清零，在原有数据的基础上重新标定力传感器
    int HR_SetMaxSearchVelocities();                         //设置力控探寻的最大速度
    int HR_SetControlFreedom();                               //设置力控探寻自由度
    int HR_SetForceControlStrategy();                           //设置力控控制策略
    int HR_SetFreeDrivePositionAndOrientation();                     //设置力传感器中心相对于法兰盘的安装位置和姿态
    int HR_SetPIDControlParams();                                   //设置力控探寻 PID 参数
    int HR_SetMassParams();                                 //设置惯量控制参数
    int HR_SetDampParams();                                        //设置阻尼(b)控制参数
    int HR_SetStiffParams();                                       //设置刚度(k)控制参数
    int HR_SetForceControlGoal();                                    //设置力控目标力
    int HR_SetControlGoal();                                             //设置力控目标力和目标距离(力控目标距离暂未启用)
    int HR_SetForceDataLimit();                                      //设置力控限制范围-力传感器超过此范围后控制器断电
    int HR_SetForceDistanceLimit();                                  //设置力控形变范围
    int HR_SetForceFreeDriveMode();                             //设置开启或者关闭力控自由驱动模式
    int HR_ReadFTCabData();                                          //读取标定后的力传感器数据
};

class Move
{
public:
    int HR_ShortJogJ();                        //关节短点动 运动距离 2°，最大速度<10°/s
    int HR_ShortJogL();                            //空间坐标短点动 运动距离 2mm，最大速度<10mm/s
    int HR_LongJogJ();                                 //关节长点动，最大运动速度<10°/s
    int HR_LongJogL();                                 //空间长点动
    int HR_LongMoveEvent();                                 //长点动继续指令，当开始长点动之后，要按 200 毫秒或更短时间为时间周期发送一次该指令，否则长点动会停止
    int HR_IsMotionDone();                              //判断机器人是否处于运动状态
    int HR_IsBlendingDone();                             //判断路点是否运动完成
    int HR_WayPointEx_MoveJ();
    int HR_WayPointEx_MoveL();
    int HR_WayPoint_MoveJ();
    int HR_WayPoint_MoveL();
    int HR_WayPoint2();                                   
    int HR_MoveJ();                                              //关节运动
    int HR_MoveL();                                          //直线轨迹运动
    int HR_MoveC();                                               //M圆弧轨迹运动
    int HR_MoveZ();                                             //Z 型轨迹运动
    int HR_MoveE();                                           //椭圆型轨迹运动
    int HR_MoveS();                                               //螺旋轨迹运动
    int HR_MoveRelJ();
    int HR_MoveRelL();
    int HR_WayPointRel();
};

class MovePathControl
{
public:
    int HR_StartPushMovePathJ();                 //初始化关节连续轨迹运动
    int HR_PushMovePathJ();                              //下发运动轨迹点位
    int HR_EndPushMovePathJ();                       //轨迹下发完成并开始计算轨迹
    int HR_MovePathJ();                                  //运动指定的轨迹
    int HR_ReadMovePathJState();                       //读取当前的轨迹状态
    int HR_UpdateMovePathJName();                          //更新指定轨迹的名称
    int HR_DelMovePathJ();                         //删除指定轨迹
    int HR_ReadTrackProcess();                       //读取当前的轨迹运动进度
    int HR_InitMovePathL();                              //初始化空间轨迹运动
    int HR_PushMovePathL();                           //下发运动轨迹点位，
    int HR_PushMovePaths();                            //批量下发轨迹点位，调用一次可下发多个点位数据
    int HR_MovePathL();                                       //执行空间坐标轨迹运动
};

class ServoControl
{
public:
    int HR_StartServo();                          //启动机器人在线控制（servoJ 或 servoP）时，设定位置固定更新的周期和前瞻时间
    int HR_PushServoJ();                              //在线关节位置命令控制，以 StartServo 设定的固定更新时间发送关节位置，机器人将实时的跟踪关节位置指令
    int HR_PushServoP();                 //在线末端 TCP 位置命令控制，以 StartServo 设定的固定更新时间发送 TCP 位置，机器人将实时的跟踪目标 TCP 位置逆运算转换后的关节位置指令
    int HR_InitServoEsJ();                     //初始化在线控制模式，清空缓存点位,ServoEsJ
    int HR_StartServoEsJ();                        //启动在线控制模式，设定位置固定更新的周期和前瞻时间，开始运动
    int HR_PushServoEsJ();                                //批量下发在线控制点位,每个点位下发频率由固定更新的周期确定
    int HR_ReadServoEsJState();                           //读取当前是否可以继续下发点位信息，循环读取间隔 > 20ms
};
    
class MoveTrace
{
public:
    int HR_SetMoveTraceParams();             //设置相对跟踪运动控制参数
    int HR_SetMoveTraceInitParams();                 //设置相对跟踪运动初始化参数
    int HR_SetMoveTraceUcs();                  //设置相对跟踪运动的跟踪探寻方向
    int HR_SetTrackingState();                  //设置传送带跟踪运动状态
};

class Others
{
public:
    int HR_HRAppCmd();                  //执行插件 app 命令
    int HR_WriteEndHoldingRegisters();             //写末端连接的 modbus 从站寄存器，
    int HR_ReadEndHoldingRegisters();                    //读末端连接的 modbus 从站寄存器，
};

int Others::HR_HRAppCmd()
{
    string nState = "RobotiqGripper,GripperControl";
    string v = "30,100,100"; //参数列表
    int nRet = HRIF_HRAppCmd( 0,nState, v);
    if (nRet != 0)
    {
        err("HRIF_HRAppCmd");
        return nRet;
    }
    cout << "HRIF_HRAppCmd调用成功！" << endl;
    return nRet;
}

int Others::HR_WriteEndHoldingRegisters() 
{
    int nSlaveID = 2;  // 从站ID
    int nFunction = 16; // 功能ID
    int nRegAddr = 5004;    //寄存器地址
    int nRegCount = 2;//寄存器数量
    vector<int> vecData={196,34464};
    nRet = HRIF_WriteEndHoldingRegisters(0,0, nSlaveID,  nFunction,  nRegAddr,  nRegCount, vecData);
    if (nRet != 0)
    {
        err("HRIF_WriteEndHoldingRegisters");
        return nRet;
    }
    cout << "HRIF_WriteEndHoldingRegisters调用成功！" << endl;
    return nRet;
} 

int Others::HR_ReadEndHoldingRegisters() {
    int boxID = 0;
    int nSlaveID = 2;  // 从站ID
    int nFunction = 3; // 功能ID
    int nRegAddr = 5004;    //寄存器地址
    int nRegCount = 2;//寄存器数量
    vector<int> vecData={};
    int nRet = HRIF_ReadEndHoldingRegisters(0,0, nSlaveID, nFunction, nRegAddr, nRegCount, vecData);
    if (nRet != 0)
    {
        err("HRIF_ReadEndHoldingRegisters");
        return nRet;
    }
    cout << "HRIF_ReadEndHoldingRegisters调用成功！" << endl;
    auto iter = vecData.begin();
    auto end = vecData.end();
    for (; iter != end; iter++)
    {
        cout << *iter << endl;
    }
    return nRet;
}

int Move::HR_MoveRelJ()
{
    int nAxisID=0;
    int nDirection=0;
    double nDistance=5;
    int nRet=HRIF_MoveRelJ(0,0,nAxisID,nDirection,nDistance);
    if (nRet != 0)
    {
        err("HRIF_MoveRelJ");
        return nRet;
    }
    cout << "HRIF_MoveRelJ调用成功！" << endl;
    return nRet;
}

int Move::HR_MoveRelL()
{
    int nAxisID=0;
    int nDirection=0;
    double nDistance=5;
    int nToolMotion=0;
    int nRet=HRIF_MoveRelL(0,0,nAxisID,nDirection,nDistance,nToolMotion);
    if (nRet != 0)
    {
        err("HRIF_MoveRelJ");
        return nRet;
    }
    cout << "HRIF_MoveRelJ调用成功！" << endl;
    return nRet;
}

int Move::HR_WayPointRel()
{
    // 定义运动类型
int nType = 0;
// 定义是否使用点位列表的点位
int nPointList= 0;
// 定义空间目标位置
double dPos_X = 420; double dPos_Y = 0; double dPos_Z = 445;
double dPos_Rx = 180; double dPos_Ry = 0; double dPos_Rz = 180;
// 定义关节目标位置
double dPos_J1 = 0; double dPos_J2 = 0; double dPos_J3 = 90;
double dPos_J4 = 0; double dPos_J5 = 90; double dPos_J6 = 0;
// 定义相对运动类型
int nrelMoveType= 0;
// 定义各轴\各方向是否运动
int nAxisMask_1 = 1; int nAxisMask_2 = 1; int nAxisMask_3 = 1;
int nAxisMask_4 = 1; int nAxisMask_5 = 1; int nAxisMask_6 = 1;
// 定义运动距离
double dTarget_1= 1; double dTarget_2= 2; double dTarget_3= 3;
double dTarget_4= 4; double dTarget_5= 5; double dTarget_6= 6;
// 定义工具坐标变量
string sTcpName = "TCP";
// 定义用户坐标变量
string sUcsName = "Base";
// 定义运动速度
double dVelocity = 50;
// 定义运动加速度
double dAcc = 50;
// 定义过渡半径
double dRadius = 50;
// 定义是否使用关节角度
int nIsUseJoint = 1;
// 定义是否使用检测 DI 停止
int nIsSeek = 0;
// 定义检测的 DI 索引
int nIOBit = 0;
// 定义检测的 DI 状态
int nIOState = 0;
// 定义路点 ID 
string strCmdID = "0";
int nRet=HRIF_WayPointRel(0,0,nType,nPointList,dPos_X,dPos_Y,dPos_Z,dPos_Rx,dPos_Ry,dPos_Rz,
dPos_J1,dPos_J2,dPos_J3,dPos_J4,dPos_J5,dPos_J6,
nrelMoveType,
nAxisMask_1,nAxisMask_2,nAxisMask_3,nAxisMask_4,nAxisMask_5,nAxisMask_6,
dTarget_1,dTarget_2,dTarget_3,dTarget_4,dTarget_5,dTarget_6,
sTcpName,sUcsName,dVelocity,dAcc,dRadius,nIsUseJoint,nIsSeek,nIOBit,nIOState,strCmdID);
if (nRet != 0)
    {
        err("HRIF_WayPointRel");
        return nRet;
    }
    cout << "HRIF_WayPointRel调用成功！" << endl;
    return nRet;
}




//初始化控制命令
int InitRobot::HR_Connect()
{
    string strCPSIP("192.168.15.24");
    unsigned short nCmdPort = 10003;
    int boxID = 0;
    int nRet = HRIF_Connect(boxID, strCPSIP.c_str(), nCmdPort);
    if (nRet != 0)
    {
        err("HRIF_Connect");
        return nRet;
    }
    cout << "HRIF_Connect调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_IsConnected()
{
    bool nRet = HRIF_IsConnected(0);
    cout << "HRIF_IsConnected调用成功！" << endl;
    cout <<"连接状态：" << nRet << endl;
    return nRet;
}

int InitRobot::HR_DisConnect()
{
    int nRet = HRIF_DisConnect(0);
    if (nRet != 0)
    {
        err("HRIF_DisConnect");
        return nRet;
    }
    cout << "HRIF_DisConnect调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_Connect2Box()
{
    int nRet = HRIF_Connect2Box(0);
    if (nRet != 0)
    {
        err("HRIF_Connect2Box");
        return nRet;
    }
    cout << "HRIF_Connect2Box调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_Electrify()
{
    int nRet = HRIF_Electrify(0);
    if (nRet != 0)
    {
        err("HRIF_Electrify");
        return nRet;
    }
    cout << "HRIF_Electrify调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_Connect2Controller()
{
    int nRet = HRIF_Connect2Controller(0);
    if (nRet != 0)
    {
        err("HRIF_Connect2Controller");
        return nRet;
    }
    cout << "HRIF_Connect2Controller调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_IsSimulateRobot()
{
    int state;
    int nRet = HRIF_IsSimulateRobot(0, state);
    if (nRet != 0)
    {
        err("HRIF_IsSimulateRobot");
        return nRet;
    }
    cout << "HRIF_IsSimulateRobot调用成功！" << endl;
    cout << "模拟状态：" << state << endl;
    return nRet;
}

int InitRobot::HR_IsControllerStarted()
{
    int start;
    int nRet = HRIF_IsControllerStarted(0, start);
    if (nRet != 0)
    {
        err("HRIF_IsControllerStarted");
        return nRet;
    }
    cout << "HRIF_IsControllerStarted调用成功！" << endl;
    cout << "是否启动：" << start << endl;
    return nRet;
}

int InitRobot::HR_ReadVersion()
{

    string strVer = "";
    int nCPSVersion = 0;
    int nCodesysVersion = 0;
    int nBoxVerMajor = 0;
    int nBoxVerMid = 0;
    int nBoxVerMin = 0;
    int nAlgorithmVer = 0;
    int nElfinFirmwareVer = 0;
    int nRet = HRIF_ReadVersion(0, 0,strVer, nCPSVersion, nCodesysVersion, nBoxVerMajor, nBoxVerMid, nBoxVerMin, nAlgorithmVer, nElfinFirmwareVer);
    if (nRet != 0)
    {
        err("HRIF_ReadVersion");
        return nRet;
    }
    cout << "HRIF_ReadVersion调用成功！" << endl;
    cout << strVer << endl;
    cout << nCPSVersion << endl;
    cout << nCodesysVersion << endl;
    cout << nBoxVerMajor << endl;
    cout << nBoxVerMid << endl;
    cout << nBoxVerMin << endl;
    cout << nAlgorithmVer << endl;
    cout << nElfinFirmwareVer << endl;
    return nRet;
}

int InitRobot::HR_ReadRobotModel()
{
    string strModel = "";
    int nRet = HRIF_ReadRobotModel(0,strModel);
    if (nRet != 0)
    {
        err("HRIF_ReadRobotModel");
        return nRet;
    }
    cout << "HRIF_ReadRobotModel调用成功！" << endl;
    cout <<"机器人的型号：" << strModel << endl;
    return nRet;
}

int InitRobot::HR_Blackout()
{
    nRet = HRIF_Blackout(0);
    if (nRet != 0)
    {
        err("HRIF_Blackout");
        return nRet;
    }
    cout << "HRIF_Blackout调用成功！" << endl;
    return nRet;
}

int InitRobot::HR_ShutdownRobot()
{
    int nRet = HRIF_ShutdownRobot(0);
    if (nRet != 0)
    {
        err("HRIF_ShutdownRobot");
        return nRet;
    }
    cout << "HRIF_ShutdownRobot调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetForceControlState()
{
    int state = 1;
    int nRet = HRIF_SetForceControlState(0,0, state);
    if (nRet != 0)
    {
        err("HRIF_SetForceControlState");
        return nRet;
    }
    cout << "HRIF_SetForceControlState调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetForceControlGoal()
{
    double Force[6] = { 0,0,-10,0,0,0 };
    int nRet = HRIF_SetForceControlGoal(0,0, Force[0], Force[1], Force[2], Force[3], Force[4], Force[5]);
    if (nRet != 0)
    {
        err("HRIF_SetForceControlGoal");
        return nRet;
    }
    cout << "HRIF_SetForceControlGoal调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetControlGoal()
{
    double Force[6] = { 0,0,-20,0,0,0 };
    double Distance[6] = { 0,0,1000,0,0,0 };
    int nRet = HRIF_SetControlGoal(0,0, Force[0], Force[1], Force[2], Force[3], Force[4], Force[5], Distance[0], Distance[1], Distance[2], Distance[3], Distance[4], Distance[5]);
    if (nRet != 0)
    {
        err("HRIF_SetControlGoal");
        return nRet;
    }
    cout << "HRIF_SetControlGoal调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetForceDataLimit()
{
    int Max[6] = { 100,100,100,100,100,100 };
    int Min[6] = { -100,-100,-100,-100,-100,-100 };
    int nRet = HRIF_SetForceDataLimit(0,0, Max[0], Max[1], Max[2], Max[3], Max[4], Max[5], Min[0], Min[1], Min[2], Min[3], Min[4], Min[5]);
    if (nRet != 0)
    {
        err("HRIF_SetForceDataLimit");
        return nRet;
    }
    cout << "HRIF_SetForceDataLimit调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetForceDistanceLimit()
{
    double Distance = 120;
    double Level = 8;
    int nRet = HRIF_SetForceDistanceLimit(0, 0,Distance, Level);
    if (nRet != 0)
    {
        err("HRIF_SetForceDistanceLimit");
        return nRet;
    }
    cout << "HRIF_SetForceDistanceLimit调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_ReadForceControlState()
{
    int state=1 ;
    int nRet = HRIF_ReadForceControlState(0,0, state);
    if (nRet != 0)
    {
        err("HRIF_ReadForceControlState");
        return nRet;
    }
    cout << "HRIF_ReadForceControlState调用成功！" << endl;
    cout <<"ForceControlState:" << state << endl;
    return nRet;
}

int ForceControl::HR_SetForceControlStrategy()
{
    int state = 0;
    int nRet = HRIF_SetForceControlStrategy(0,0, state);
    if (nRet != 0)
    {
        err("HRIF_SetForceControlStrategy");
        return nRet;
    }
    cout << "HRIF_SetForceControlStrategy调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_GetForceParams()
{
    double dInertia[6] = {0};
    double dDamping[6] = {0};
    double dStiffness[6] = {0};
   
    int nRet = HRIF_GetForceParams(0,0, dInertia, dDamping, dStiffness);
    if (nRet != 0)
    {
        err("HRIF_GetForceParams");
        return nRet;
    }
    cout << "HRIF_GetForceParams调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << dInertia[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << dDamping[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << dStiffness[i] << " ";
    }
    return nRet;
}

int ForceControl::HR_SetForceParams()
{
    double dInertia[6] = { 40,40,40,10,10,10 };
    double dDamping[6] = {2000,1000,1000,40,40,40};
    double dStiffness[6] = {1000,1000,1000,100,100,100};
    int nRet = HRIF_SetForceParams(0,0, dInertia, dDamping, dStiffness);
    if (nRet != 0)
    {
        err("HRIF_SetForceParams");
        return nRet;
    }
    cout << "HRIF_SetForceParams调用成功！" << endl;
    return 0;
}

int ForceControl::HR_SetForceControlParams()
{
    double ForceLimit[6] = { 10,10,10,10,10,10 };
    double DistLimit[6] = { 50,50,50,50,50,50 };
    int nRet = HRIF_SetForceControlParams(0,0, ForceLimit, DistLimit);
    if (nRet != 0)
    {
        err("HRIF_SetForceControlParams");
        return nRet;
    }
    cout << "HRIF_SetForceControlParams调用成功！" << endl;
    return nRet;
}


int ForceControl::HR_ForceControlInterrupt()
{
    int nRet = HRIF_ForceControlInterrupt(0,0);
    if (nRet != 0)
    {
        err("HRIF_ForceControlInterrupt");
        return nRet;
    }
    cout << "HRIF_ForceControlInterrupt调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_ForceControlContinue()
{
    int nRet = HRIF_ForceControlContinue(0,0);
    if (nRet != 0)
    {
        err("HRIF_ForceControlContinue");
        return nRet;
    }
    cout << "HRIF_ForceControlContinue调用成功！" << endl;
    return nRet;
}



//轴组控制命令
int AxisControl::HR_GrpEnable()
{
    int nRet = HRIF_GrpEnable(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpEnable");
        return nRet;
    }
    cout << "HRIF_GrpEnable调用成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpDisable()
{
    int nRet = HRIF_GrpDisable(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpDisable");
        return nRet;
    }
    cout << "HRIF_GrpDisable调用成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpReset()
{
    int nRet = HRIF_GrpReset(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpReset");
        return nRet;
    }
    cout << "HRIF_GrpReset调用成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpStop()
{
    int nRet = HRIF_GrpStop(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpStop");
        return nRet;
    }
    cout << "HRIF_GrpStop调用成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpInterrupt()
{
    int nRet = HRIF_GrpInterrupt(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpInterrupt");
        return nRet;
    }
    cout << "HRIF_GrpInterrupt调用成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpContinue()
{
    int nRet = HRIF_GrpContinue(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpContinue");
        return nRet;
    }
    cout << "HRIF_GrpContinue调用成功！" << endl;
    return nRet;
}


int AxisControl::HR_GrpOpenFreeDriver()
{
    int nRet = HRIF_GrpOpenFreeDriver(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpOpenFreeDriver");
        return nRet;
    }
    cout << "开启零力示教成功！" << endl;
    return nRet;
}

int AxisControl::HR_GrpCloseFreeDriver()
{
    int nRet = HRIF_GrpCloseFreeDriver(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpCloseFreeDriver");
        return nRet;
    }
    cout << "关闭零力示教成功！" << endl;
    return nRet;
}


//脚本控制命令
int ScriptControl::HR_RunFunc()
{
    string name = "Func_1";
    vector<string>param = {};
    int nRet = HRIF_RunFunc(0, name, param);
    if (nRet != 0)
    {
        err("HRIF_RunFunc");
        return nRet;
    }
    cout << "HRIF_RunFunc调用成功！" << endl;
    return nRet;
}

int ScriptControl::HR_StartScript()
{
    int nRet = HRIF_StartScript(0);
    if (nRet != 0)
    {
        err("HRIF_StartScript");
        return nRet;
    }
    cout << "脚本运行成功！" << endl;
    return nRet;
}

int ScriptControl::HR_StopScript()
{
    int nRet = HRIF_StopScript(0);
    if (nRet != 0)
    {
        err("HRIF_StopScript");
        return nRet;
    }
    cout << "HRIF_StopScript调用成功！" << endl;
    return nRet;
}

int ScriptControl::HR_PauseScript()
{
    int nRet = HRIF_PauseScript(0);
    if (nRet != 0)
    {
        err("HRIF_PauseScript");
        return nRet;
    }
    cout << "HRIF_PauseScript调用成功！" << endl;
    return nRet;
}

int ScriptControl::HR_ContinueScript()
{
    int nRet = HRIF_ContinueScript(0);
    if (nRet != 0)
    {
        err("HRIF_ContinueScript");
        return nRet;
    }
    cout << "HRIF_ContinueScript调用成功！" << endl;
    return nRet;
}



//电箱控制命令
int BoxControl::HR_ReadBoxInfo()
{
    int nConnected;
    int n48V_ON;
    double d48OUT_Voltag;
    double d48OUT_Current;
    int nRemoteBTN;
    int nThreeStageBTN;
    int nRet = HRIF_ReadBoxInfo(0, 0,nConnected, n48V_ON, d48OUT_Voltag, d48OUT_Current, nRemoteBTN, nThreeStageBTN);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxInfo");
        return nRet;
    }
    cout << "HRIF_ReadBoxInfo调用成功！" << endl;
    cout  << nConnected << "," << n48V_ON << "," << d48OUT_Voltag << "," << d48OUT_Current << "," << nRemoteBTN << "," << nThreeStageBTN << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxCI()
{
    int Bit = 0;
    int Val = 1;
    int nRet = HRIF_ReadBoxCI(0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxCI");
        return nRet;
    }
    cout << "HRIF_ReadBoxCI调用成功！" << endl;
    cout << "CI:" << Val << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxDI()
{
    int Bit = 0;
    int Val = 0;
    int nRet = HRIF_ReadBoxDI(0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxDI");
        return nRet;
    }
    cout << "HRIF_ReadBoxDI调用成功！" << endl;
    cout <<"DI:" << Val << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxDO()
{
    int Bit = 0;
    int Val = 0;
    int nRet = HRIF_ReadBoxDO(0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxDO");
        return nRet;
    }
    cout << "HRIF_ReadBoxDO调用成功！" << endl;
    cout <<"D0:" << Val << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxCO()
{
    int Bit = 0;
    int Val = 0;
    int nRet = HRIF_ReadBoxCO(0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxCO");
        return nRet;
    }
    cout << "HRIF_ReadBoxCO调用成功！" << endl;
    cout <<"CO:" << Val << endl;
    return nRet;
}

int BoxControl::HR_SetBoxAOVal()
{
    int Bit = 1;
    double Val = 4.0;
    int mode = 1;
    int nRet = HRIF_SetBoxAOVal(0, Bit, Val, mode);
    if (nRet != 0)
    {
        err("HRIF_SetBoxAOVal");
        return nRet;
    }
    cout << "HRIF_SetBoxAOVal调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_ReadEndDI()
{
    int Bit = 0;
    int Val = 1;
    int nRet = HRIF_ReadEndDI(0,0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadEndDI");
        return nRet;
    }
    cout << "HRIF_ReadEndDI调用成功！" << endl;
    cout <<"EndDI:" << Val << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxAI()
{
    int bit = 0;
    double val = 0;
    int nRet = HRIF_ReadBoxAI(0, bit, val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxAI");
        return nRet;
    }
    cout << "HRIF_ReadBoxAI调用成功！" << endl;
    cout <<"AI:" << val << endl;
    return nRet;
}

int BoxControl::HR_ReadBoxAO()
{
    int bit = 0;
    double val = 10;
    int mode = 0;
    int nRet = HRIF_ReadBoxAO(0, bit, mode, val);
    if (nRet != 0)
    {
        err("HRIF_ReadBoxAO");
        return nRet;
    }
    cout << "HRIF_ReadBoxAO调用成功！" << endl;
    cout <<"AO:" << val << endl;
    return nRet;
}

int BoxControl::HR_SetBoxCO()
{
    int bit = 2;
    int state = 1;
    int nRet = HRIF_SetBoxCO(0, bit, state);
    if (nRet != 0)
    {
        err("HRIF_SetBoxCO");
        return nRet;
    }
    cout << "HRIF_SetBoxCO调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_SetBoxDO()
{
    int bit = 2;
    int state = 1;
    int nRet = HRIF_SetBoxDO(0, bit, state);
    if (nRet != 0)
    {
        err("HRIF_SetBoxDO");
        return nRet;
    }
    cout << "HRIF_SetBoxDO调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_SetEndDO()
{
    int Bit = 2;
    int Val = 1;
    int nRet = HRIF_SetEndDO(0,0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_SetEndDO");
        return nRet;
    }
    cout << "HRIF_SetEndDO调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_SetBoxAOMode()
{
    int bit = 0;
    int state = 2;
    int nRet = HRIF_SetBoxAOMode(0, bit, state);
    if (nRet != 0)
    {
        err("HRIF_SetBoxAOMode");
        return nRet;
    }
    cout << "HRIF_SetBoxAOMode调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_ReadEndBTN()
{
    int a, b, c, d;
    int nRet = HRIF_ReadEndBTN(0,0, a, b, c, d);
    if (nRet != 0)
    {
        err("HRIF_ReadEndBTN");
        return nRet;
    }
    cout << "HRIF_ReadEndBTN调用成功！" << endl;
    cout << a << " " << b << " " << c << " " << d << endl;
    return nRet;
}

//


int HR_Interrupt()
{
    int nRet = HRIF_GrpInterrupt(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpInterrupt");
        return nRet;
    }
    cout << "HRIF_GrpInterrupt调用成功！" << endl;
    return nRet;
}

int HR_Continue()
{
    int nRet = HRIF_GrpContinue(0,0);
    if (nRet != 0)
    {
        err("HRIF_GrpContinue");
        return nRet;
    }
    cout << "HRIF_GrpContinue调用成功！" << endl;
    return nRet;
}


int PoseCal::HR_PoseAdd()
{
    double P1[6] = { 0,0,0,0,0,0 };
    double P2[6] = { 10,10,10,10,10,10 };
    double P3[6] = { 0 };
    int nRet = HRIF_PoseAdd(0, 0,P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);
    if (nRet != 0)
    {
        err("HRIF_PoseAdd");
        return nRet;
    }
    cout << "HRIF_PoseAdd调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << P3[i] << endl;
    }
    return nRet;
}

int PoseCal::HR_PoseSub()
{
    double P1[6] = { 0,0,0,0,0,0 };
    double P2[6] = { 10,10,10,10,10,10 };
    double P3[6] = { 0 };
    int nRet = HRIF_PoseSub(0, 0,P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], 
        P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);
    if (nRet != 0)
    {
        err("HRIF_PoseSub");
        return nRet;
    }
    cout << "HRIF_PoseSub调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << P3[i] << endl;
    }
    return nRet;
}

int TcpAndUcs::HR_SetTCP()
{
    double TCP[6] = { 0,0,10,0,0,0 };
    int nRet = HRIF_SetTCP(0,0, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_SetTCP");
        return nRet;
    }
    cout << "HRIF_SetTCP调用成功！" << endl;
    return nRet; 
}

int TcpAndUcs::HR_SetUCS()
{
    double Ucs[6] = { 0,0,10,0,0,0 };
    int nRet = HRIF_SetUCS(0,0, Ucs[0], Ucs[1], Ucs[2], Ucs[3], Ucs[4], Ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_SetUCS");
        return nRet;
    }
    cout << "HRIF_SetUCS调用成功！" << endl;
    return nRet;
}

int TcpAndUcs::HR_ReadCurTCP()
{
    double TCP[6] = { 0 };
    int nRet = HRIF_ReadCurTCP(0,0, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCurTCP");
        return nRet;
    }
    cout << "HRIF_ReadCurTCP调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << TCP[i] << endl;
    }
    return nRet;
}

int TcpAndUcs::HR_ReadCurUCS()
{
    double Ucs[6] = { 0 };
    int nRet = HRIF_ReadCurUCS(0,0, Ucs[0], Ucs[1], Ucs[2], Ucs[3], Ucs[4], Ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCurUCS");
        return nRet;
    }
    cout << "HRIF_ReadCurUCS调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Ucs[i] << endl;
    }
    return nRet;
}

int TcpAndUcs::HR_SetTCPByName()
{
    string TCP = "TCP_1";
    int nRet = HRIF_SetTCPByName(0, 0,TCP);
    if (nRet != 0)
    {
        err("HRIF_SetTCPByName");
        return nRet;
    }
    cout << "HRIF_SetTCPByName调用成功！" << endl;
    return nRet;
}

int Move::HR_IsBlendingDone()
{
    bool a;
    int nRet = HRIF_IsBlendingDone(0,0, a);
    if (nRet != 0)
    {
        err("HRIF_IsBlendingDone");
        return nRet;
    }
    cout << "HRIF_IsBlendingDone调用成功！" << endl;
    cout << a << endl;
    return nRet;
}

int Move::HR_MoveE() 
{
    int boxID = 0; 
    double dp1[6]={ 420,0,445,180,0,180 };
    double dp2[6]={ 460,0,445,180,0,180 };
    double dp3[6]={ 480,10,445,180,0,180 };
    double dp4[6]={ 460,20,445,180,0,180 };
    double dp5[6]={ 420,20,445,180,0,180 };

    int nOrientMode = 0;//弧运动类型：0：圆弧，1：整圆
    int nMoveType = 0;  // 0 : 不使用固定姿态，1: 使用固定姿态
    double dArcLength = 1000;     //弧长
    double  dVelocity = 1;     // 速度
    double dAcc = 30;          //加速度
    double dRadius = 0;     // 过渡半径
    string sTcpName = "TCP";    // 刀具坐标名称
    string sUcsName = "Base";    //用户坐标名称
    string sCmdID ="0";  // 命令ID
    int nRet = HRIF_MoveE(boxID, 0,
        dp1[0], dp1[1], dp1[2], dp1[3], dp1[4], dp1[5],
        dp2[0], dp2[1], dp2[2], dp2[3], dp2[4], dp2[5],
        dp3[0], dp3[1], dp3[2], dp3[3], dp3[4], dp3[5],
        dp4[0], dp4[1], dp4[2], dp4[3], dp4[4], dp4[5],
        dp5[0], dp5[1], dp5[2], dp5[3], dp5[4], dp5[5],
        nOrientMode, nMoveType, dArcLength,
         dVelocity,  dAcc,  dRadius,  sTcpName, sUcsName, sCmdID);
    if (nRet != 0)
    {
        err("HRIF_MoveE");
        return nRet;
    }
    cout << "HRIF_MoveE调用成功！" << endl;
    return nRet;
}

int Move::HR_MoveS()
{
    double r = 10;
    double End_r = 30;
    double Vel = 1;
    double Acc = 1;
    double dRadius = 1;
    string  Tcp="TCP";
    string Ucs = "Base";
    string ID = "0";
    int nRet = HRIF_MoveS(0,0, r, End_r, Vel, Acc, dRadius, Tcp,Ucs, ID);
    if (nRet != 0)
    {
        err("HRIF_MoveS");
        return nRet;
    }
    cout << "HRIF_MoveS调用成功！" << endl;
    return nRet;

}


int ServoControl::HR_InitServoEsJ()
{
    //初始化在线控制模式，清空缓存点位
    int boxID = 0;
    int nRet = HRIF_InitServoEsJ(boxID,0);
    if (nRet != 0)
    {
        err("HRIF_InitServoEsJ");
        return nRet;
    }
    cout << "HRIF_InitServoEsJ调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_StartServoEsJ()
{
    //启动在线控制模式，开始运动
    int boxID = 0;
    int dServoTime = 4;     //固定更新的周期 s
    int dLookaheadTime = 4; //前瞻时间 s
    int nRet = HRIF_StartServoEsJ(boxID,0,  dServoTime, dLookaheadTime);
    if (nRet != 0)
    {
        err("HRIF_StartServoEsJ");
        return nRet;
    }
    cout << "HRIF_StartServoEsJ调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_PushServoEsJ() 
{

    //批量下发在线控制点位

    int boxID = 0;
    int nPointsSize = 2;     //点位数量，最大允许下发500个点位
    string sPoints = "0,0,0,0,0,0,40,40,40,40,40,40"; //点位信息，例如："0,0,0,0,0,0,1,1,1,1,1,1,1"
    int nRet = HRIF_PushServoEsJ(boxID,0, nPointsSize, sPoints);
    if (nRet != 0)
    {
        err("HRIF_PushServoEsJ");
        return nRet;
    }
    cout << "HRIF_PushServoEsJ调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_ReadServoEsJState() {
    //读取当前是否可以继续下发点位信息，循环读取间隔>20ms
    int boxID = 0;
    int nState = 0;     //nState: 0：允许下发点位    1：不允许下发点位
    int Index = 0;
    int Count = 0;
    int nRet = HRIF_ReadServoEsJState(boxID, 0,nState,Index,Count);
    if (nRet != 0)
    {
        err("HRIF_ReadServoEsJState");
        return nRet;
    }
    cout << "HRIF_ReadServoEsJState调用成功！" << endl;
    cout << nState << endl;
    return nRet;

}


int Move::HR_WayPoint2()
{
    // 定义运动类型
    int nMoveType = 0;
    // 定义空间目标位置
    double dEndPos_X = 420; double dEndPos_Y = 0; double dEndPos_Z = 445;
    double dEndPos_Rx = 180; double dEndPos_Ry = 0; double dEndPos_Rz = 180;
    // 定义空间目标位置
    double dAuxPos_X = 420; double dAuxPos_Y = 100; double dAuxPos_Z = 445;
    double dAuxPos_Rx = 180; double dAuxPos_Ry = 0; double dAuxPos_Rz = 180;
    // 定义关节目标位置
    double dJ1 = 0; double dJ2 = 0; double dJ3 = 90;
    double dJ4 = 0; double dJ5 = 90; double dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 50;
    // 定义过渡半径
    double dRadius = 1;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_WayPoint2(0,0, nMoveType, dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry,
        dEndPos_Rz, dAuxPos_X, dAuxPos_Y, dAuxPos_Z, dAuxPos_Rx, dAuxPos_Ry, dAuxPos_Rz, dJ1, dJ2, dJ3, dJ4, dJ5,
        dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
        nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    if (nRet != 0)
    {
        err("HRIF_WayPoint2");
        return nRet;
    }
    cout << "HRIF_WayPoint2调用成功！" << endl;
    return nRet;
}



int Move::HR_MoveZ()
{
    // 起始点位置
    double dStartPos_X = 420; double dStartPos_Y = 0; double dStartPos_Z = 445;
    double dStartPos_Rx = 180; double dStartPos_Ry = 0; double dStartPos_Rz = 180;
    // 结束点位置
    double dEndPos_X = 440; double dEndPos_Y = 0; double dEndPos_Z = 445;
    double dEndPos_Rx = 180; double dEndPos_Ry = 0; double dEndPos_Rz = 180;
    // 确定轨迹平面点位置
    double dPlanePos_X = 420; double dPlanePos_Y = 50; double dPlanePos_Z = 445;
    double dPlanePos_Rx = 180; double dPlanePos_Ry = 0; double dPlanePos_Rz = 180;
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 2500;
    // 宽度
    double dWidth = 50;
    // 密度
    double dDensity = 10;
    // 使用密度
    int nEnableDensity = 1;
    // 使用平面点
    int nEnablePlane = 1;
    // 是否在转折点等待-不等待
    int nEnableWaiTime = 0;
    // 正向转折点等待时间
    int nPosiTime = 0;
    // 负向转折点等待时间
    int nNegaTime = 0;
    // 定义过渡半径
    double dRadius = 5;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_MoveZ(0,0,
        dStartPos_X, dStartPos_Y, dStartPos_Z, dStartPos_Rx, dStartPos_Ry, dStartPos_Rz, dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry, dEndPos_Rz, dPlanePos_X, dPlanePos_Y,
        dPlanePos_Z, dPlanePos_Rx, dPlanePos_Ry, dPlanePos_Rz, dVelocity, dAcc, dWidth, dDensity, nEnableDensity, nEnablePlane, nEnableWaiTime, nPosiTime, nNegaTime, dRadius, sTcpName, sUcsName, strCmdID);
    if (nRet != 0)
    {
        err("HRIF_MoveZ");
        return nRet;
    }
    cout << "HRIF_MoveZ调用成功！" << endl;
    return nRet;
}

int Move::HR_ShortJogJ()
{
    int Axis = 0;
    int A = 0;
    int nRet = HRIF_ShortJogJ(0, 0,Axis, A);
    if (nRet != 0)
    {
        err("HRIF_ShortJogJ");
        return nRet;
    }
    cout << "HRIF_ShortJogJ调用成功！" << endl;
    return nRet;
}

int Move::HR_ShortJogL()
{
    int Axis = 0;
    int A = 1;
    int nRet = HRIF_ShortJogL(0,0, Axis, A);
    if (nRet != 0)
    {
        err("HRIF_ShortJogL");
        return nRet;
    }
    cout << "HRIF_ShortJogL调用成功！" << endl;
    return nRet;
}

int Move::HR_LongJogJ()
{
    int Axis = 0;
    int A = 0;
    int state = 1;
    int nRet = HRIF_LongJogJ(0, 0,Axis, A, state);
    if (nRet != 0)
    {
        err("HRIF_LongJogJ");
        return nRet;
    }
    cout << "HRIF_LongJogJ调用成功！" << endl;
   
    return nRet;
}

int Move::HR_LongJogL()
{
    int Axis = 0;
    int A = 0; 
    int state = 1;
    int nRet = HRIF_LongJogL(0,0, Axis, A, state);
    if (nRet != 0)
    {
        err("HRIF_LongJogL");
        return nRet;
    }
    cout << "HRIF_LongJogL调用成功！" << endl;
    
    return nRet;
}

int Move::HR_LongMoveEvent()
{
    
      
    return nRet;
}



int ReadAndSet::HR_SetOverride()
{
    double override = 0.3;
    int nRet = HRIF_SetOverride(0,0, override);
    if (nRet != 0)
    {
        err("HRIF_SetOverride");
        return nRet;
    }
    cout << "HRIF_SetOverride调用成功！" << endl;
    return nRet;
}



int ReadAndSet::HR_SetPayload()
{
    double mass = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    int nRet = HRIF_SetPayload(0,0, mass, x, y, z);
    if (nRet != 0)
    {
        err("HRIF_SetPayload");
        return nRet;
    }
    cout << "HRIF_SetPayload调用成功！" << endl;
    return nRet;
}




int ReadAndSet::HR_ReadEmergencyInfo()
{
    int nESTO_Error = 0;
    int nESTO = 0;
    int nSafetyGuard_Error = 0;
    int nSafetyGuard = 0;
    int nRet = HRIF_ReadEmergencyInfo(0, 0,nESTO_Error, nESTO, nSafetyGuard_Error, nSafetyGuard);
    if (nRet != 0)
    {
        err("HRIF_ReadEmergencyInfo");
        return nRet;
    }
    cout << "HRIF_ReadEmergencyInfo调用成功！" << endl;
    cout << nESTO_Error << endl;
    cout << nESTO << endl;
    cout << nSafetyGuard_Error << endl;
    cout << nSafetyGuard << endl;
    return nRet;
}

int ReadAndSet::HR_ReadRobotState()
{

    int nMovingState = 0; int nEnableState = 0; int nErrorState = 0; int nErrorCode = 0;
    int nErrorAxis = 0; int nBreaking = 0; int nPause = 0; int nEmergencyStop = 0;
    int nSaftyGuard = 0; int nElectrify = 0; int nIsConnectToBox = 0; int nBlendingDone = 0; int nInPos = 0;

    int nRet = HRIF_ReadRobotState(0,0, nMovingState, nEnableState,
        nErrorState, nErrorCode, nErrorAxis, nBreaking, nPause, nEmergencyStop,
        nSaftyGuard, nElectrify, nIsConnectToBox, nBlendingDone, nInPos);
    if (nRet != 0)
    {
        err("HRIF_ReadRobotState");
        return nRet;
    }
    cout << "HRIF_ReadRobotState调用成功！" << endl;
    cout << nMovingState << endl;
    cout << nEnableState << endl;
    cout << nErrorState << endl;
    cout << nErrorCode << endl;
    cout << nErrorAxis << endl;
    cout << nBreaking << endl;
    cout << nPause << endl;
    cout << nEmergencyStop << endl;
    cout << nSaftyGuard << endl;
    cout << nElectrify << endl;
    cout << nIsConnectToBox << endl;
    cout << nBlendingDone << endl;
    cout << nInPos << endl;
    return nRet;
}

int ReadAndSet::HR_ReadJointMaxAcc()
{
    double Acc[6] = { 0 };
    int nRet = HRIF_ReadJointMaxAcc(0, 0,Acc[0], Acc[1], Acc[2], Acc[3], Acc[4], Acc[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadJointMaxAcc");
        return nRet;
    }
    cout << "HRIF_ReadJointMaxAcc调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Acc[i] << endl;
    }
    return nRet;
}

int ReadAndSet::HR_ReadJointMaxJerk()
{
    double Jerk[6] = { 0 };
    int nRet = HRIF_ReadJointMaxJerk(0,0, Jerk[0], Jerk[1], Jerk[2], Jerk[3], Jerk[4], Jerk[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadJointMaxJerk");
        return nRet;
    }
    cout << "HRIF_ReadJointMaxJerk调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Jerk[i] << endl;
    }
    return nRet;
}

int ReadAndSet::HR_ReadLinearMaxSpeed()
{
    double Vel = 0;
    double Acc = 0;
    double Jerk = 0;
    int nRet = HRIF_ReadLinearMaxSpeed(0,0, Vel, Acc, Jerk);
    if (nRet != 0)
    {
        err("HRIF_ReadLinearMaxSpeed");
        return nRet;
    }
    cout << "HRIF_ReadLinearMaxSpeed调用成功！" << endl;
    cout << Vel << endl;
    cout << Acc << endl;
    cout << Jerk << endl;
    return nRet;
}


int ReadAndSet::HR_SetJointMaxAcc()
{
    double Acc[6] = { 10,10,10,10,10,10 };
    int nRet = HRIF_SetJointMaxAcc(0,0, Acc[0], Acc[1], Acc[2], Acc[3], Acc[4], Acc[5]);
    if (nRet != 0)
    {
        err("HRIF_SetJointMaxAcc");
        return nRet;
    }
    cout << "HRIF_SetJointMaxAcc调用成功！" << endl;
    return nRet;
}

int ReadAndSet::HR_SetLinearMaxAcc()
{
    double Acc = 10;
    int nRet = HRIF_SetLinearMaxAcc(0, 0,Acc);
    if (nRet != 0)
    {
        err("HRIF_SetLinearMaxAcc");
        return nRet;
    }
    cout << "HRIF_SetLinearMaxAcc调用成功！" << endl;
    return nRet;
}


int Positon_Cur_Speed::HR_ReadActPos()
{
    double Base[6] = { 0 };
    double Joint[6] = { 0 };
    double Tcp[6] = { 0 };
    double Ucs[6] = { 0 };
    int nRet = HRIF_ReadActPos(0,0, Base[0], Base[1], Base[2], Base[3], Base[4], Base[5], Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], Tcp[0], Tcp[1], Tcp[2], Tcp[3], Tcp[4]
        , Tcp[5], Ucs[0], Ucs[1], Ucs[2], Ucs[3], Ucs[4], Ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActPos");
        return nRet;
    }
    cout << "HRIF_ReadActPos调用成功！" << endl;
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

int Positon_Cur_Speed::HR_ReadCmdJointPos()
{
    double Joint[6] = { 0 };
    int nRet = HRIF_ReadCmdJointPos(0,0, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCmdJointPos");
        return nRet;
    }
    cout << "HRIF_ReadCmdJointPos调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Joint[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadActJointPos()
{
    double Joint[6] = { 0 };
    int nRet = HRIF_ReadActJointPos(0,0, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActJointPos");
        return nRet;
    }
    cout << "HRIF_ReadActJointPos调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Joint[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadCmdTcpPos()
{
    double TCP[6] = { 0 };
    int nRet = HRIF_ReadCmdTcpPos(0, 0,TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCmdTcpPos");
        return nRet;
    }
    cout << "HRIF_ReadCmdTcpPos调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << TCP[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadActTcpPos()
{
    double TCP[6] = { 0 };
    int nRet = HRIF_ReadActTcpPos(0,0, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActTcpPos");
        return nRet;
    }
    cout << "HRIF_ReadActTcpPos调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << TCP[i] << endl;
    }
    return nRet;
}

int  Positon_Cur_Speed::HR_ReadCmdTcpVel()
{
    double Vel[6] = { 0 };
    int nRet = HRIF_ReadCmdTcpVel(0,0, Vel[0], Vel[1], Vel[2], Vel[3], Vel[4], Vel[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCmdTcpVel");
        return nRet;
    }
    cout << "HRIF_ReadCmdTcpVel调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Vel[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadActTcpVel()
{
    double Vel[6] = { 0 };
    int nRet = HRIF_ReadActTcpVel(0,0, Vel[0], Vel[1], Vel[2], Vel[3], Vel[4], Vel[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActTcpVel");
        return nRet;
    }
    cout << "HRIF_ReadActTcpVel调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Vel[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadActJointCur()
{
    double Cur[6] = { 0 };
    int nRet = HRIF_ReadActJointCur(0,0, Cur[0], Cur[1], Cur[2], Cur[3], Cur[4], Cur[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActJointCur");
        return nRet;
    }
    cout << "HRIF_ReadActJointCur调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Cur[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadTcpVelocity()
{
    double Cmd = 0;
    double Act = 0;
    int nRet = HRIF_ReadTcpVelocity(0, 0,Cmd, Act);
    if (nRet != 0)
    {
        err("HRIF_ReadTcpVelocity");
        return nRet;
    }
    cout << "HRIF_ReadTcpVelocity调用成功！" << endl;
    cout << Cmd << endl;
    cout << Act << endl;
    return nRet;
}

int PoseCal::HR_PoseDist()
{
    double P1[6] = { 420,0,434,180,0,180 };
    double P2[6] = { 420,100,434,180,0,180 };
    double distance = 0;
    double angle = 0;
    int nRet = HRIF_PoseDist(0, 0,P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], distance, angle);
    if (nRet != 0)
    {
        err("HRIF_PoseDist");
        return nRet;
    }
    cout << "HRIF_PoseDist调用成功！" << endl;
    cout << distance << endl;
    cout << angle << endl;
    return nRet;
}

int PoseCal::HR_PoseInverse()
{
    double Base[6] = { 0,0,1728,0,0,0};
    double Ucs[6] = { 100,100,0,0,0,0 };
    int nRet = HRIF_PoseInverse(0, 0,Base[0], Base[1], Base[2], Base[3], Base[4], Base[5], Ucs[0], Ucs[1], Ucs[2], Ucs[3], Ucs[4], Ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_PoseInverse");
        return nRet;
    }
    cout << "HRIF_PoseInverse调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Ucs[i] << " ";
    }
    cout << endl;
    return nRet;
}
int PoseCal::HR_PoseInterpolate()
{
    double dPose1_X = 420; double dPose1_Y = 0; double dPose1_Z = 445;
    double dPose1_Rx = 180; double dPose1_Ry = 0; double dPose1_Rz = 180;
    // 定义需要计算的空间坐标 2
    double dPose2_X = 420; double dPose2_Y = 100; double dPose2_Z = 445;
    double dPose2_Rx = 180; double dPose2_Ry = 0; double dPose2_Rz = 180;
    // 插补比例
    double dAlpha = 0.6;
    // 计算结果
    double dPose3[6] = { 0 };
    // 计算结果
    int nRet = HRIF_PoseInterpolate(0, 0,dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
        dPose2_X, dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz, dAlpha,
        dPose3[0], dPose3[1], dPose3[2], dPose3[3], dPose3[4], dPose3[5]);
    if (nRet != 0)
    {
        err("HRIF_PoseInterpolate");
        return nRet;
    }
    cout << "HRIF_PoseInterpolate调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << dPose3[i] << endl;
    }
    return nRet;
}

int TcpAndUcs::HR_SetUCSByName()
{
    string Name = "Point_1";
    int nRet = HRIF_SetUCSByName(0,0, Name);
    if (nRet != 0)
    {
        err("HRIF_SetUCSByName");
        return nRet;
    }
    cout << "HRIF_SetUCSByName调用成功！" << endl;
    return nRet;
}

int TcpAndUcs::HR_ReadTCPByName()
{
    string Name = "TCP_1";
    double TCP[6] = { 0 };
    int nRet = HRIF_ReadTCPByName(0,0, Name, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadTCPByName");
        return nRet;
    }
    cout << "HRIF_ReadTCPByName调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << TCP[i] << endl;
    }
    return nRet;
}

int TcpAndUcs::HR_ReadUCSByName()
{
    string Name = "Point_1";
    double TCP[6] = { 0 };
    int nRet = HRIF_ReadUCSByName(0,0, Name, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadUCSByName");
        return nRet;
    }
    cout << "HRIF_ReadUCSByName调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << TCP[i] << endl;
    }
    return nRet;
}

int PoseCal::HR_RPY2Quaternion()
{
    int boxID = 0;
    double dx = 6;
    double dy = -40.556;
    double dz = 16.510; 
    double w;
    double x;
    double y;
    double z;
    int nRet = HRIF_RPY2Quaternion(boxID, 0,dx, dy, dz, w, x, y, z);
    if (nRet != 0)
    {
        err("HRIF_RPY2Quaternion");
        return nRet;
    }
    cout << "HRIF_RPY2Quaternion调用成功！" << endl;
    cout << w << " " << x << " " << y << " " << z << endl;
    return nRet;
}

int PoseCal::HR_Quaternion2RPY()
{
    int boxID = 0;
    double w =0.8;
    double x = 0.5;
    double y = 0.2;
    double z = 0;
    double dx;
    double dy;
    double dz;
    int nRet = HRIF_Quaternion2RPY(boxID,0, w, x, y, z, dx, dy, dz);
    if (nRet != 0)
    {
        err("HRIF_Quaternion2RPY");
        return nRet;
    }
    cout << "HRIF_Quaternion2RPY调用成功！" << endl;
    cout << dx << " " << dy << " " << dz << endl;
    return nRet;
}

int PoseCal::HR_GetInverseKin()
{
    double Base[6] = {420,0,434,180,0,180};
    double tcp[6] = { 0 };
    double ucs[6] = { 0};
    double J[6] = {0,0,90,0,90,0 };
    double T[6] = { 0 };
    int nRet = HRIF_GetInverseKin(0, 0,Base[0], Base[1], Base[2], Base[3], Base[4], Base[5],
        tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5],
        ucs[0], ucs[1], ucs[2], ucs[3], ucs[4], ucs[5],
        J[0], J[1], J[2], J[3], J[4], J[5],
        T[0], T[1], T[2], T[3], T[4], T[5]);
    if (nRet != 0)
    {
        err("HRIF_GetInverseKin");
        return nRet;
    }
    cout << "HRIF_GetInverseKin调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << T[i] << endl;
    }
    return nRet;
}

int MovePathControl::HR_StartPushMovePathJ()
{
    // 轨迹名称
    string sTrackName = "drag_3";
    // 速度比
    double dSpeedRatio = 0.1;
    // 过渡半径
    double dRadius = 1;
    // 初始化关节连续轨迹运动
    int nRet = HRIF_StartPushMovePathJ(0, 0,sTrackName, dSpeedRatio, dRadius);
    if (nRet != 0)
    {
        err("HRIF_StartPushMovePathJ");
        return nRet;
    }
    cout << "HRIF_StartPushMovePathJ调用成功！" << endl;
    return nRet;
}
int MovePathControl::HR_PushMovePathJ()
{
    // 轨迹名称
    string sTrackName = "drag_3";
    // 目标关节位置
    double P1[6] = { 90,90,90,90,90,90 };
    double P2[6] = { -90,-90,-90,-90,-90,-90 };
    double P3[6] = { 90,0,90,0,90,0 };
    double P4[6] = { -90,0,-90,0,-90,0 };
    double P5[6] = { 0,0,90,0,0,0 };
    // 下发关节点位
    int nRet = HRIF_PushMovePathJ(0,0, sTrackName, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5]);
    if (nRet != 0)
    {
        cout << nRet << endl;
        return nRet;
    }

    nRet = HRIF_PushMovePathJ(0, 0,sTrackName, P2[0], P2[1], P2[2], P2[3], P2[4], P2[5]);
    if (nRet != 0)
    {
        cout << nRet << endl;
        return nRet;
    }
    nRet = HRIF_PushMovePathJ(0,0, sTrackName, P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);
    if (nRet != 0)
    {
        cout << nRet << endl;
        return nRet;
    }
    nRet = HRIF_PushMovePathJ(0,0, sTrackName, P4[0], P4[1], P4[2], P4[3], P4[4], P4[5]);
    if (nRet != 0)
    {
        cout << nRet << endl;
        return nRet;
    }
    nRet = HRIF_PushMovePathJ(0,0, sTrackName, P5[0], P5[1], P5[2], P5[3], P5[4], P5[5]);
    if (nRet != 0)
    {
        cout << nRet << endl;
        return nRet;
    }

    return nRet;
}

int MovePathControl::HR_EndPushMovePathJ()
{
    // 轨迹名称
    string sTrackName = "drag_3";
    // 下发完成，开始计算轨迹
    int nRet = HRIF_EndPushMovePathJ(0, 0,sTrackName);
    if (nRet != 0)
    {
        err("HRIF_EndPushMovePathJ");
        return nRet;
    }
    cout << "HRIF_EndPushMovePathJ调用成功！" << endl;
    return nRet;
}

int MovePathControl::HR_MovePathJ()
{
    // 轨迹名称
    string sTrackName = "drag_3";
    // 运动轨迹
    int nRet = HRIF_MovePathJ(0, 0,sTrackName);
    if (nRet != 0)
    {
        err("HRIF_MovePathJ");
        return nRet;
    }
    cout << "HRIF_MovePathJ调用成功！" << endl;
    return nRet;
}

int MovePathControl::HR_ReadMovePathJState()
{
    string Name = "drag_3";
    int state;
    int nRet = HRIF_ReadMovePathJState(0,0, Name, state);
    if (nRet != 0)
    {
        err("HRIF_ReadMovePathJState");
        return nRet;
    }
    cout << "HRIF_ReadMovePathJState调用成功！" << endl;
    cout <<"the  state: " << state << endl;
    return nRet;
}

int MovePathControl::HR_UpdateMovePathJName()
{
    string Old_Name = "drag_3";
    string New_Name = "drag_5";
    int nRet = HRIF_UpdateMovePathJName(0,0, Old_Name, New_Name);
    if (nRet != 0)
    {
        err("HRIF_UpdateMovePathJName");
        return nRet;
    }
    cout << "HRIF_UpdateMovePathJName调用成功！" << endl;
    return nRet;
}

int MovePathControl::HR_DelMovePathJ()
{
    string Name = "drag_5";
    int nRet = HRIF_DelMovePathJ(0,0, Name);
    if (nRet != 0)
    {
        err("HRIF_DelMovePathJ");
        return nRet;
    }
    cout << "HRIF_DelMovePathJ调用成功！" << endl;
    return nRet;
}

int MovePathControl::HR_ReadTrackProcess()
{
    double a;
    int b;
    int nRet = HRIF_ReadTrackProcess(0,0, a, b);
    if (nRet != 0)
    {
        err("HRIF_ReadTrackProcess");
        return nRet;
    }
    cout << "HRIF_ReadTrackProcess调用成功！" << endl;
    cout << a << endl;
    cout << b << endl;
    return nRet;
}

int MovePathControl::HR_InitMovePathL()
{
    string sTrackName = "drag_1";
    double dVelocity = 100;
    double dAcc = 20;
    double dJerk = 100;
    string sTcpName = "TCP";
    string sUcsName = "Base";
    int nRet = HRIF_InitMovePathL(0,0, sTrackName, dVelocity, dAcc, dJerk, sUcsName, sTcpName);
    cout << nRet << endl;
    //double P1[6] = { 700,0,692,-180,0,-180 };
    //double P2[6] = { 700,10,692,-180,0,-180 };
    //double P3[6] = { 700,0,692,-180,0,-180 };
    //double P4[6] = { 700,0,692,-180,0,-180 };
    //double P5[6] = { 710,0,692,-180,0,-180 };
    //double P6[6] = { 700,40,692,-180,0,-180 };
    //double P7[6] = { 720,10,692,-180,0,-180 };
    //double P8[6] = { 700,0,692,-180,0,-180 };
    //double P9[6] = { 700,0,692,-180,0,-180 };
    //double P10[6] = { 700,10,692,-180,0,-180 };
    //double P11[6] = { 700,0,692,-180,0,-180 };
    //nRet = HRIF_PushMovePathL(0,0, "drag", P1[0], P1[1], P1[2], P1[3], P1[4], P1[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P2[0], P2[1], P2[2], P2[3], P2[4], P2[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P4[0], P4[1], P4[2], P4[3], P4[4], P4[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P5[0], P5[1], P5[2], P5[3], P5[4], P5[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P6[0], P6[1], P6[2], P6[3], P6[4], P6[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P7[0], P7[1], P7[2], P7[3], P7[4], P7[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0, 0,"drag", P8[0], P8[1], P8[2], P8[3], P8[4], P8[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P9[0], P9[1], P9[2], P9[3], P9[4], P9[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P10[0], P10[1], P10[2], P10[3], P10[4], P10[5]);
    //cout << nRet << endl;
    //nRet = HRIF_PushMovePathL(0,0, "drag", P11[0], P11[1], P11[2], P11[3], P11[4], P11[5]);
    //cout << nRet << endl;
    /////*int nMoveType = 1;
    ////int nPointsSize = 6;
    ////string sPoints = "720,0,692,180,0,180,700,10,692,180,0,180,750,20,692,180,0,180,700,30,692,180,0,180,700,40,692,180,0,180,700,50,692,180,0,180,700,60,692,180,0,180";
    ////nRet = HRIF_PushMovePaths(0, "drag", nMoveType, nPointsSize, sPoints);
    ////cout << nRet << endl;*/
    ////Sleep(2000);
    //nRet = HRIF_MovePathL(0, 0,"drag");
    cout << nRet << endl;
    return nRet;
}

int MovePathControl::HR_PushMovePathL()
{
    string sTrackName = "drag_1";
    double P1[6] = { 420,10,445,180,0,180 };
    double P2[6] = { 420,30,445,180,0,180 };
    double P3[6] = { 410,50,445,180,0,180 };
    double P4[6] = { 420,-10,445,180,0,180 };
    double P5[6] = { 420,-30,445,180,0,180 };
    double P6[6] = { 410,-50,445,180,0,180 };
    double P7[6] = { 420,-70,445,180,0,180 };
    // 下发关节点位
    int nRet = HRIF_PushMovePathL(0,0, sTrackName, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P2[0], P2[1], P2[2], P2[3], P2[4], P2[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P4[0], P4[1], P4[2], P4[3], P4[4], P4[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P5[0], P5[1], P5[2], P5[3], P5[4], P5[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P6[0], P6[1], P6[2], P6[3], P6[4], P6[5]);

    nRet = HRIF_PushMovePathL(0,0, sTrackName, P7[0], P7[1], P7[2], P7[3], P7[4], P7[5]);
    cout << nRet << endl;
 
    return nRet;
}


int MovePathControl::HR_PushMovePaths()
{
    string sTrackName = "drag_1";
    string P1 = " 420, 0, 434, 180, 0, 180, 420, 50, 434, 180, 0, 180, 410, 0, 434, 180, 0, 180, 430, -50, 434, 180, 0, 180,420,100,434,180,0,180";

    int nRet = HRIF_PushMovePaths(0, 0,sTrackName, 1, 5, P1);
    if (nRet != 0)
    {
        err("HRIF_PushMovePaths");
        return nRet;
    }
    cout << "HRIF_PushMovePaths调用成功！" << endl;
    return nRet;
}

int MovePathControl::HR_MovePathL()
{
    //轨迹名称
    string sTrackName = "drag_1";
    // 开始空间连续轨迹运动
    int nRet = HRIF_MovePathL(0,0, sTrackName);
    if (nRet != 0)
    {
        err("HRIF_MovePathL");
        return nRet;
    }
    cout << "HRIF_MovePathL调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_StartServo()
{
    double dServoTime = 4;
    double dLookaheadTime = 4;
    int nRet = HRIF_StartServo(0,0, dServoTime, dLookaheadTime);
    if (nRet != 0)
    {
        err("HRIF_StartServo");
        return nRet;
    }
    cout << "HRIF_StartServo调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_PushServoJ()
{
    double P1[6] = { 0,0,45,0,45,0 };
    double P0[6] = { 0 };
    double P2[6]= { 0,0,45,0,45,0 };
    sleep(4000);
    int nRet = HRIF_PushServoJ(0,0, P0[0], P0[1], P0[2], P0[3], P0[4], P0[5]);
    sleep(4000);
    nRet = HRIF_PushServoJ(0,0, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5]);
    sleep(4000);
    nRet = HRIF_PushServoJ(0,0, P2[0], P2[1], P2[2], P2[3], P2[4], P2[5]);
    if (nRet != 0)
    {
        err("HRIF_PushServoJ");
        return nRet;
    }
    cout << "HRIF_PushServoJ调用成功！" << endl;
    return nRet;
}

int ServoControl::HR_PushServoP()
{
    vector<double> Base1 = {420,0,445,180,0,180 };
    vector<double> Base2 = { 420,100,445,180,0,180 };
    vector<double> Ucs = { 0,0,0,0,0,0 };
    vector<double> Tcp = { 0,0,0,0,0,0 };
    sleep(4000);
    int nRet = HRIF_PushServoP(0,0, Base1, Ucs, Tcp);
    cout << nRet << endl;
    sleep(4000);
    nRet = HRIF_PushServoP(0,0, Base2, Ucs, Tcp);
    if (nRet != 0)
    {
        err("HRIF_PushServoP");
        return nRet;
    }
    cout << "HRIF_PushServoP调用成功！" << endl;
    return nRet;
}

int MoveTrace::HR_SetMoveTraceParams()
{
    // 设置跟踪状态
    double nState = 1;
    // 相对跟踪运动保持的相对距离
    double dDistance =170;
    // 相对跟踪的运动的探寻速度
    double dAwayVelocity = 50;
    // 相对跟踪的运动的远离速度--设置相对跟踪运动控制参数并开启相对跟踪运动
    double dGobackVelocity = 50;
    //相对跟踪的运动的返回速度
    int nRet = HRIF_SetMoveTraceParams(0,0, nState, dDistance, dAwayVelocity, dGobackVelocity);
    if (nRet != 0)
    {
        err("HRIF_SetMoveTraceParams");
        return nRet;
    }
    cout << "HRIF_SetMoveTraceParams调用成功！" << endl;
    return nRet;
}


int MoveTrace::HR_SetMoveTraceInitParams()
{
    // 传感器计算参数
    double dK = -32;
    // 传感器计算参数
    double dB = 280;
    // 激光传感器检测距离最大值
    double dMaxLimit = 200;
    // 激光传感器检测距离最小值
    double dMinLimit = 65;
    // 设置跟踪状态初始化参数
    int nRet = HRIF_SetMoveTraceInitParams(0,0, -dK, dB, dMaxLimit, dMinLimit);
    if (nRet != 0)
    {
        err("HRIF_SetMoveTraceInitParams");
        return nRet;
    }
    cout << "HRIF_SetMoveTraceInitParams调用成功！" << endl;
    return nRet;
}

int MoveTrace::HR_SetMoveTraceUcs()
{
    // 设置跟踪方向
    double dX = 0; double dY = 0; double dZ = 0;
    double dRx = 180; double dRy = 0; double dRz = 0;
    int nRet = HRIF_SetMoveTraceUcs(0,0, dX, dY, dZ, dRx, dRy, dRz);
    if (nRet != 0)
    {
        err("HRIF_SetMoveTraceUcs");
        return nRet;
    }
    cout << "HRIF_SetMoveTraceUcs调用成功！" << endl;
    return nRet;
}


int MoveTrace::HR_SetTrackingState()
{
    // 设置传送带跟踪开启
    int nState = 1;
    // 开启传送带跟踪
    int nRet = HRIF_SetTrackingState(0,0, nState);
    if (nRet != 0)
    {
        err("HRIF_SetTrackingState");
        return nRet;
    }
    cout << "HRIF_SetTrackingState调用成功！" << endl;
    return nRet;
}



/*int ReadAndSet::HR_SetTCPMotion()
{
    int state = 1;
    int nRet = HRIF_SetTCPMotion(0, 0,state);
    if (nRet != 0)
    {
        err("HRIF_SetTCPMotion");
        return nRet;
    }
    cout << "HRIF_SetTCPMotion调用成功！" << endl;
    return nRet;
}*/




int  PoseCal::HR_PoseDefdFrame()
{
    // 定义需要计算的空间坐标 1
    double dPose1_X = 400; double dPose1_Y = 0; double dPose1_Z =434 ;
    // 定义需要计算的空间坐标 2
    double dPose2_X = 400; double dPose2_Y = 100; double dPose2_Z = 434;
    // 定义需要计算的空间坐标 3
    double dPose3_X = 400; double dPose3_Y = 200; double dPose3_Z = 434;
    // 定义需要计算的空间坐标 4
    double dPose4_X = 430; double dPose4_Y = 0; double dPose4_Z = 434;
    // 定义需要计算的空间坐标 5
    double dPose5_X = 430; double dPose5_Y = -100; double dPose5_Z = 434;
    // 定义需要计算的空间坐标 6
    double dPose6_X = 430; double dPose6_Y = -200; double dPose6_Z = 434;
    // 计算结果
    double dUcs_X = 0; double dUcs_Y = 0; double dUcs_Z = 0;
    double dUcs_Rx = 0; double dUcs_Ry = 0; double dUcs_Rz = 0;
    // 计算结果
    int nRet = HRIF_PoseDefdFrame(0, 0,dPose1_X, dPose1_Y, dPose1_Z, dPose2_X, dPose2_Y, dPose2_Z, dPose3_X,
        dPose3_Y, dPose3_Z, dPose4_X, dPose4_Y, dPose4_Z, dPose5_X, dPose5_Y, dPose5_Z, dPose6_X, dPose6_Y,
        dPose6_Z, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    if (nRet != 0)
    {
        err("HRIF_PoseDefdFrame");
        return nRet;
    }
    cout << "HRIF_PoseDefdFrame调用成功！" << endl;
    cout << dUcs_X << endl;
    cout << dUcs_Y << endl;
    cout << dUcs_Z << endl;
    cout << dUcs_Rx << endl;
    cout << dUcs_Ry << endl;
    cout << dUcs_Rz << endl;
    return nRet;
}

int PoseCal::HR_PoseTrans()
{
    double P1[6] = { 700,0,692,180,0,180 };
    double P2[6] = { 719,118,764,180,11,-170};
    double P3[6] = { 0 };
    int nRet = HRIF_PoseTrans(0, 0,P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5]);
    if (nRet != 0)
    {
        err("HRIF_PoseTrans");
        return nRet;
    }
    cout << "HRIF_PoseTrans调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << P3[i] << endl;
    }
    return nRet;
}

int ForceControl::HR_SetForceFreeDriveMode()
{
    bool type =false ;
    int nRet = HRIF_SetForceFreeDriveMode(0, 0,type);
    if (nRet != 0)
    {
        err("HRIF_SetForceFreeDriveMode");
        return nRet;
    }
    cout << "HRIF_SetForceFreeDriveMode调用成功！" << endl;
    return nRet;
}


int ForceControl::HR_SetStiffParams()
{
    double Stiff[6] = { 1000,1000,1000,100,100,100 };
    int nRet = HRIF_SetStiffParams(0,0, Stiff[0], Stiff[1], Stiff[2], Stiff[3], Stiff[4], Stiff[5]);
    if (nRet != 0)
    {
        err("HRIF_SetStiffParams");
        return nRet;
    }
    cout << "HRIF_SetStiffParams调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetForceMode()
{
    bool state = true;
    int nRet = HRIF_SetForceMode(0,0, state);
    if (nRet != 0)
    {
        err("HRIF_SetForceMode");
        return nRet;
    }
    cout << "HRIF_SetForceMode调用成功！" << endl;
    return nRet;
}


int ForceControl::HR_SetForceToolCoordinateMotion()
{
    int state = 0;
    int nRet = HRIF_SetForceToolCoordinateMotion(0, 0,state);
    if (nRet != 0)
    {
        err("HRIF_SetForceToolCoordinateMotion");
        return nRet;
    }
    cout << "HRIF_SetForceToolCoordinateMotion调用成功！" << endl;
    return nRet;
}



int ForceControl::HR_SetForceZero()
{
    int nRet = HRIF_SetForceZero(0,0);
    if (nRet != 0)
    {
        err("HRIF_SetForceZero");
        return nRet;
    }
    cout << "HRIF_SetForceZero调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetMaxSearchVelocities()
{
    double dMaxLinearVelocity = 20;
    double dMaxAngularVelocity = 50;
    int nRet = HRIF_SetMaxSearchVelocities(0,0, dMaxLinearVelocity, dMaxAngularVelocity);
    if (nRet != 0)
    {
        err("HRIF_SetMaxSearchVelocities");
        return nRet;
    }
    cout << "HRIF_SetMaxSearchVelocities调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetControlFreedom()
{
    int nX =0 ; int nY = 0; int nZ = 1;
    int nRx = 0; int nRy = 0; int nRz = 0;
    int nRet = HRIF_SetControlFreedom(0, 0,nX, nY, nZ, nRx, nRy, nRz);
    if (nRet != 0)
    {
        err("HRIF_SetControlFreedom");
        return nRet;
    }
    cout << "HRIF_SetControlFreedom调用成功！" << endl;
    return nRet;
}


int ForceControl::HR_SetFreeDrivePositionAndOrientation()
{
    double dX =  0; double dY = 0; double dZ = 0;
    double dRx = 0; double dRy = 0; double dRz = 0;
    int nRet = HRIF_SetFreeDrivePositionAndOrientation(0,0, dX, dY, dZ, dRx, dRy, dRz);
    if (nRet != 0)
    {
        err("HRIF_SetFreeDrivePositionAndOrientation");
        return nRet;
    }
    cout << "HRIF_SetFreeDrivePositionAndOrientation调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetPIDControlParams()
{
    double dFp = 1; double dFi = 0.1; double dFd = 0;
    double dTp = 1; double dTi = 0.1; double dTd = 0;
    int nRet = HRIF_SetPIDControlParams(0,0, dFp, dFi, dFd, dTp, dTi, dTd);
    if (nRet != 0)
    {
        err("HRIF_SetPIDControlParams");
        return nRet;
    }
    cout << "HRIF_SetPIDControlParams调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetMassParams()
{
    double dX = 40; double dY = 40; double dZ = 40;
    double dRx = 10; double dRy = 10; double dRz = 10;
    int nRet = HRIF_SetMassParams(0,0, dX, dY, dZ, dRx, dRy, dRz);
    if (nRet != 0)
    {
        err("HRIF_SetMassParams");
        return nRet;
    }
    cout << "HRIF_SetMassParams调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_SetDampParams()
{
    double dX =800; double dY = 800; double dZ = 800;
    double dRx = 40; double dRy = 40; double dRz = 40;
    int nRet = HRIF_SetDampParams(0,0, dX, dY, dZ, dRx, dRy, dRz);
    if (nRet != 0)
    {
        err("HRIF_SetDampParams");
        return nRet;
    }
    cout << "HRIF_SetDampParams调用成功！" << endl;
    return nRet;
}

int ForceControl::HR_ReadFTCabData()
{
    double data[6] = { 0 };
    int nRet = HRIF_ReadFTCabData(0,0, data[0], data[1], data[2], data[3], data[4], data[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadFTCabData");
        return nRet;
    }
    cout << "HRIF_ReadFTCabData调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << data[i] << endl;
    }
    return nRet;
}



int ReadAndSet::HR_SetJointMaxVel()
{
    double Vel[6] = { 180,180,180,180,200,200};
    int nRet = HRIF_SetJointMaxVel(0,0, Vel[0], Vel[1], Vel[2], Vel[3], Vel[4], Vel[5]);
    if (nRet != 0)
    {
        err("HRIF_SetJointMaxVel");
        return nRet;
    }
    cout << "HRIF_SetJointMaxVel调用成功！" << endl;
    return nRet;
}

int ReadAndSet::HR_SetMaxAcsRange()
{
    double MAX[6] = {  360,135,153,360,180,0 };
    double MIN[6] = { -360,-135,-153,-360,-180,0 };
    int nRet = HRIF_SetMaxAcsRange(0,0, MAX[0], MAX[1], MAX[2], MAX[3], MAX[4], MAX[5], MIN[0], MIN[1], MIN[2], MIN[3], MIN[4], MIN[5]);
    if (nRet != 0)
    {
        err("HRIF_SetMaxAcsRange");
        return nRet;
    }
    cout << "HRIF_SetMaxAcsRange调用成功！" << endl;
    return nRet;
}

int ReadAndSet::HR_SetMaxPcsRange()
{
    double MAX[6] = {1500,1500,1500,-1500,-1500,-1500 };
    double ucs[6] = { 0,0,0,0,0,0 };
    int nRet = HRIF_SetMaxPcsRange(0,0, MAX[0], MAX[1], MAX[2], MAX[3], MAX[4], MAX[5], ucs[0], ucs[1], ucs[2], ucs[3], ucs[4], ucs[5]);
    if (nRet != 0)
    {
        err("HRIF_SetMaxPcsRange");
        return nRet;
    }
    cout << "HRIF_SetMaxPcsRange调用成功！" << endl;
    return nRet;
}

int Move::HR_MoveJ()
{
    double Joint[6] = { 0,0,0,0,0,0 };
    double TCP[6] = { 520,0,572,180,0,180 };
    string TcpName = "TCP";
    string UcsName = "Base";
    double Vel = 10;
    double Acc = 360;
    double r = 1;
    int IsUseJoint = 1;
    int Iseek = 0;
    int IOBit = 0;
    int IOState = 0;
    string ID = "0";
    int nRet = HRIF_MoveJ(0, 0,TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5], Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    if (nRet != 0)
    {
        err("HRIF_MoveJ");
        return nRet;
    }
    cout << "HRIF_MoveJ调用成功！" << endl;
    return nRet;
}

int Move::HR_MoveL()
{
    double TCP[6] = {420,0,442,180,0,180 };
    double JOINT[6] = { 0,9.6,79.6,0,90.7,0 };
    string TcpName = "TCP";
    string UcsName = "Base";
    double Vel = 400;
    double Acc = 2500;
    double r = 1;
    int Iseek = 0;
    int IOBit = 0;
    int IOState = 0;
    string ID = "0";
    int nRet = HRIF_MoveL(0,0, TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5], JOINT[0], JOINT[1], JOINT[2], JOINT[3], JOINT[4], JOINT[5], "TCP", "Base", Vel, Acc, r, Iseek, IOBit, IOState, ID);
    if (nRet != 0)
    {
        err("HRIF_MoveL");
        return nRet;
    }
    cout << "HRIF_MoveL调用成功！" << endl;
    return nRet;
}

int Move::HR_WayPoint_MoveJ()
{
    int type = 0;
    double base[6] = { 0,0,1175,0,0,0 };
    double joint[6] = { 0,0,90,0,90,0 };
    string TCP = "TCP";
    string UCS = "Base";
    double Vel = 10;
    double Acc = 360;
    double r = 1;
    int IsJoint = 1;
    int a = 0;
    int b = 0;
    int c = 0;
    string ID = "0";
    int nRet = HRIF_WayPoint(0,0, type, base[0], base[1], base[2], base[3], base[4], base[5], joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], TCP, UCS, Vel, Acc, r, IsJoint, a, b, c, ID);
    if (nRet != 0)
    {
        err("HRIF_WayPoint");
        return nRet;
    }
    cout << "HRIF_WayPoint调用成功！" << endl;
    return nRet;
}

int Move::HR_WayPoint_MoveL()
{
    int type = 1;
    double base[6] = { 420,100,445,180,0,180 };
    double joint[6] = { 0,0,0,0,0,0 };
    string TCP = "TCP";
    string UCS = "Base";
    double Vel = 100;
    double Acc = 2500;
    double r = 1;
    int IsJoint = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    string ID = "0";
    int nRet = HRIF_WayPoint(0,0, type, base[0], base[1], base[2], base[3], base[4], base[5], joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], TCP, UCS, Vel, Acc, r, IsJoint, a, b, c, ID);
    if (nRet != 0)
    {
        err("HRIF_WayPoint");
        return nRet;
    }
    cout << "HRIF_WayPoint调用成功！" << endl;
    return nRet;
}

int Move::HR_WayPointEx_MoveJ()
{
    int type = 0;
    double base[6] = { -155,-380,640,90,0,-90 };
    double joint[6] = { 90,0,90,0,90,0 };
    double tcp[6] = { 0 };
    double ucs[6] = { 0 };
    double Vel = 100;
    double Acc = 360;
    double r = 0;
    int IsJoint = 1;
    int a = 0;
    int b = 0;
    int c = 0;
    string ID = "0";
    int nRet = HRIF_WayPointEx(0,0, type, base[0], base[1], base[2], base[3], base[4], base[5], joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5],
        ucs[0], ucs[1], ucs[2], ucs[3], ucs[4], ucs[5], Vel, Acc, r, IsJoint, a, b, c, ID);
    if (nRet != 0)
    {
        err("HRIF_WayPointEx");
        return nRet;
    }
    cout << "HRIF_WayPointEx调用成功！" << endl;
    return nRet;
}

int Move::HR_WayPointEx_MoveL()
{
    int type = 1;
    double base[6] = { 420,0,445,180,0,180 };
    double joint[6] = { 90,90,90,90,90,90 };
    double tcp[6] = { 0 };
    double ucs[6] = { 0 };
    double Vel = 200;
    double Acc = 2500;
    double r = 0;
    int IsJoint = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    string ID = "0";
    int nRet = HRIF_WayPointEx(0,0, type, base[0], base[1], base[2], base[3], base[4], base[5], joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5],
        ucs[0], ucs[1], ucs[2], ucs[3], ucs[4], ucs[5], Vel, Acc, r, IsJoint, a, b, c, ID);
    if (nRet != 0)
    {
        err("HRIF_WayPointEx");
        return nRet;
    }
    cout << "HRIF_WayPointEx调用成功！" << endl;
    return nRet;
}

int Move::HR_MoveC()
{
    // 圆弧起始点位置
    double dStartPos_X = 420; double dStartPos_Y = 0; double dStartPos_Z = 445;
    double dStartPos_Rx = 180; double dStartPos_Ry = 0; double dStartPos_Rz = 180;
    // 圆弧经过点位置
    double dAuxPos_X = 420; double dAuxPos_Y = 50; double dAuxPos_Z = 445;
    double dAuxPos_Rx = 180; double dAuxPos_Ry = 0; double dAuxPos_Rz = 180;
    // 圆弧结束点位置
    double dEndPos_X = 470; double dEndPos_Y = 0; double dEndPos_Z = 445;
    double dEndPos_Rx = 180; double dEndPos_Ry = 0; double dEndPos_Rz = 180;
    // 是否固定姿态
    int nFixedPosure = 1;
    // 圆弧类型
    int nMoveCType = 0;
    // 整圆圈数
    double dRadLen = 2;
    // 定义运动速度
    double dVelocity = 5;
    // 定义运动加速度
    double dAcc = 50;
    // 定义过渡半径
    double dRadius = 1;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_MoveC(0,0, dStartPos_X, dStartPos_Y, dStartPos_Z, dStartPos_Rx, dStartPos_Ry, dStartPos_Rz,
        dAuxPos_X, dAuxPos_Y, dAuxPos_Z, dAuxPos_Rx, dAuxPos_Ry, dAuxPos_Rz,
        dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry, dEndPos_Rz,
        nFixedPosure, nMoveCType, dRadLen, dVelocity, dAcc, dRadius, sTcpName, sUcsName, strCmdID);
    if (nRet != 0)
    {
        err("HRIF_MoveC");
        return nRet;
    }
    cout << "HRIF_MoveC调用成功！" << endl;
    return nRet;
}

int BoxControl::HR_ReadEndDO()
{
    int Bit = 0;
    int Val = 0;
    int nRet = HRIF_ReadEndDO(0,0, Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadEndDO");
        return nRet;
    }
    cout << "HRIF_ReadEndDO调用成功！" << endl;
    cout << Val << endl;
    return nRet;
}

int BoxControl::HR_ReadEndAI()
{
    int Bit = 0;
    double Val = 0;
    int nRet = HRIF_ReadEndAI(0, 0,Bit, Val);
    if (nRet != 0)
    {
        err("HRIF_ReadEndAI");
        return nRet;
    }
    cout << "HRIF_ReadEndAI调用成功！" << endl;
    cout << Val << endl;
    return nRet;
}

int ReadAndSet::HR_SetLinearMaxVel()
{
    double speed = 100.0;
    int nRet = HRIF_SetLinearMaxVel(0,0, speed);
    if (nRet != 0)
    {
        err("HRIF_SetLinearMaxVel");
        return nRet;
    }
    cout << "HRIF_SetLinearMaxVel调用成功！" << endl;
    return nRet;
}

int ReadAndSet::HR_ReadJointMaxVel()
{
    double joint[6] = { 0 };
    int nRet = HRIF_ReadJointMaxVel(0,0, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadJointMaxVel");
        return nRet;
    }
    cout << "HRIF_ReadJointMaxVel调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << joint[i] << endl;
    }
    return nRet;
}

int ReadAndSet::HR_ReadRobotFlags()
{
    int MovingState;
    int EnableState;
    int ErrorState;
    int ErrorCode;
    int ErrorAxis;
    int Breaking;
    int Pause;
    int IsBlendingDone;
    int nRet = HRIF_ReadRobotFlags(0,0, MovingState, EnableState, ErrorState, ErrorCode, ErrorAxis, Breaking, Pause, IsBlendingDone);
    if (nRet != 0)
    {
        err("HRIF_ReadRobotFlags");
        return nRet;
    }
    cout << "HRIF_ReadRobotFlags调用成功！" << endl;
    cout << MovingState << endl;
    cout << EnableState << endl;
    cout << ErrorState << endl;
    cout << ErrorCode << endl;
    cout << ErrorAxis << endl;
    cout << Breaking << endl;
    cout << Pause << endl;
    cout << IsBlendingDone << endl;
    return nRet;
}

int ReadAndSet::HR_ReadCurWaypointID()
{
    string ID;
    int nRet = HRIF_ReadCurWaypointID(0,0, ID);
    if (nRet != 0)
    {
        err("HRIF_ReadCurWaypointID");
        return nRet;
    }
    cout << "HRIF_ReadCurWaypointID调用成功！" << endl;
    cout << ID << endl;
    return nRet;
}

int ReadAndSet::HR_ReadAxisErrorCode()
{
    int nErrorCode = 0;
    int nJ1 = 0; int nJ2 = 1; int nJ3 = 0; int nJ4 = 0; int nJ5 = 0; int nJ6 = 0;
    int nRet = HRIF_ReadAxisErrorCode(0,0, nErrorCode, nJ1, nJ2, nJ3, nJ4, nJ5, nJ6);
    if (nRet != 0)
    {
        err("HRIF_ReadAxisErrorCode");
        return nRet;
    }
    cout << "HRIF_ReadAxisErrorCode调用成功！" << endl;
    cout << nErrorCode << endl;
    cout << nJ1 << endl;
    cout << nJ2 << endl;
    cout << nJ3 << endl;
    cout << nJ4 << endl;
    cout << nJ5 << endl;
    cout << nJ6 << endl;
    return nRet;
}

int ReadAndSet::HR_ReadCurFSM()
{
    int nCurFSM = 0;
    string strCurFSM = "";
    int nRet = HRIF_ReadCurFSM(0, 0,nCurFSM, strCurFSM);
    if (nRet != 0)
    {
        err("HRIF_ReadCurFSM");
        return nRet;
    }
    cout << "HRIF_ReadCurFSM调用成功！" << endl;
    cout << nCurFSM << endl;
    cout << strCurFSM << endl;
    return nRet;
}

int ReadAndSet::HR_ReadCurFSMFromCPS()
{
    int strCurFSM;
    int nRet = HRIF_ReadCurFSMFromCPS(0,0, strCurFSM);
    if (nRet != 0)
    {
        err("HRIF_ReadCurFSMFromCPS");
        return nRet;
    }
    cout << "HRIF_ReadCurFSMFromCPS调用成功！" << endl;
    cout << strCurFSM << endl;
    return nRet;
}

void time_time(clock_t &time0,clock_t &time1)
{
    clock_t time3 = (double)(time1 - time0) * 1000 / CLOCKS_PER_SEC;
    cout << "Runtime:" << time3 << "ms" << endl;
}


//void HR_RobotStartup()
//{
//    int nCurFSM = 0;                  //状态机的值
//    string strCurFSM = "";            //状态机的状态描述
//    clock_t time0,time3;
//        while (nCurFSM < 34)
//        {
//            nRet =HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);       // 读取当前状态机
//            cout << " 状态机的值为：" << nCurFSM << " 状态机的状态为：" << strCurFSM << endl;
//            switch(nCurFSM)
//            {
//            case 7:                //本体供电已切断
//                cout << "正在上电！" << endl;
//                time0= clock();
//                nRet = HRIF_Electrify(0);              //上电
//                //nRet = HRIF_ReadCurFSM(0, nCurFSM, strCurFSM);
//                while (nCurFSM == 7)
//                    nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM); break;     
//            //case 8:                    //正在准备给本体供电
//            //    cout << "正在准备给本体供电！" << endl;
//            //    nRet = HRIF_ReadCurFSM(0, nCurFSM, strCurFSM);
//            //    while (nCurFSM == 8)
//            //        nRet = HRIF_ReadCurFSM(0, nCurFSM, strCurFSM);break; 
//            case 14:                    //控制器已处于未初始化阶段
//                cout << "控制器已处于未初始化阶段!"<<endl;
//                nRet = HRIF_ReadCurFSM(0,0,nCurFSM,strCurFSM);
//                while (nCurFSM == 14)
//                    nRet = HRIF_ReadCurFSM(0, 0,nCurFSM, strCurFSM);break;
//            case 15:                       //正在初始化控制器
//                cout << "正在初始化控制器!" << endl;
//                nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//                while (nCurFSM == 15)
//                    nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);break;  
//            case 18:                       //控制器初始化后检查状态
//                cout << "控制器初始化后检查状态！" << endl;
//                while (nCurFSM == 18)
//                    nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);break;
//            case 23:
//                cout << "机器人使能中!" << endl;
//                nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//                while (nCurFSM == 23)
//                    nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);break;
//            case 24:                      //机器人去使能
//                time3 = clock();
//                cout << "机器人正在上使能!" << endl;
//                nRet =HRIF_GrpEnable(0,0);    //使能操作
//                nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//                while (nCurFSM == 24)
//                    nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);break;
//            case 33:
//                cout << "机器人已就绪！" << endl;
//                cout << "机器人的总的上电时间为:" << (double)(time3 - time0)/ CLOCKS_PER_SEC << "s" << endl;
//                nCurFSM += 1;  //让状态机退出while循环条件
//                break;
//            }
//        }
//           nRet = HRIF_GrpReset(0,0);                // 初始化完成后复位一次
//        cout << "初始化完成后复位一次！ 上电过程完成！" << endl;
//}

// void test_Electrify()
// {
//     fstream file;
//     int nRet;
//     clock_t time1,time2,time3,time4,time5,time6,time7,time8;
//     int nCurFSM = 0;                  //状态机的值
//     string strCurFSM = "";            //状态机的状态描述
//     file.open("调用_生效时间.csv", ios::out);
//     if (file.is_open() == true)
//     {
//         cout << "文件打开成功！" << endl;
//     }
//     else
//         cout << "文件打开失败！" << endl;
//     const char *str = "状态机从7跳到8的时间,状态机从8跳到14的时间,状态机从14跳到15的时间,状态机从15跳到18的时间,状态机从18跳到24的时间,状态机从24跳到33的时间\n";
//     file.write(str, strlen(str));
    
//     for (int i = 0;i < 50; i++)
//     {
//         Sleep(3000);
//         printf("**************************第%d次********************************************\n", i);
//         time1 = clock();
//         nRet = HRIF_Electrify(0);
//         while (nCurFSM <8)
//         {
//             nRet = HRIF_ReadCurFSM(0, 0,nCurFSM, strCurFSM);
//         }
//         time2 = clock();
//         cout << "状态机从7跳到8的时间为：" << (double)(time2 - time1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         while (nCurFSM !=14)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         time3 = clock();
//         cout<<"状态机从8跳到14的时间为："<< (double)(time3 - time2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         while (nCurFSM != 15)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         time4 = clock();
//         cout << "状态机从14跳到15的时间为：" << (double)(time4 - time3) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         while (nCurFSM != 18)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         time5 = clock();
//         cout << "状态机从15跳到18的时间为：" << (double)(time5- time4) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         while (nCurFSM != 24)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         time6 = clock();
//         cout << "状态机从18跳到24的时间为：" << (double)(time6 - time5) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         Sleep(1000);
//         time7 = clock();
//         nRet = HRIF_GrpEnable(0,0);
//         while (nCurFSM != 33)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         time8 = clock();
//         Sleep(1000);
//         cout << "状态机从24跳到33的时间为：" << (double)(time8 - time7) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
//         file << (double)(time2 - time1) * 1000 / CLOCKS_PER_SEC  <<","<< (double)(time3 - time2) * 1000 / CLOCKS_PER_SEC <<","
//             << (double)(time4 - time3) * 1000 / CLOCKS_PER_SEC <<","<< (double)(time5 - time4) * 1000 / CLOCKS_PER_SEC
//             <<","<< (double)(time6 - time5) * 1000 / CLOCKS_PER_SEC <<","<< (double)(time8 - time7) * 1000 / CLOCKS_PER_SEC << endl;
//         nRet = HRIF_GrpDisable(0,0);
//         while (nCurFSM != 24)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         Sleep(1000);
//         nRet = HRIF_Blackout(0);
//         while (nCurFSM != 7)
//         {
//             nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//         }
//         cout << "总的上电时间：" << (double)(time6-time1)/CLOCKS_PER_SEC <<"s" << endl;
//         Sleep(1000);
//     }
//     file.close();
// }


// void close()
// {
//     clock_t time0, time1;
//     int nCurFSM = 0;                  //状态机的值
//     string strCurFSM = "";            //状态机的状态描述
//     while (nCurFSM != 24)
//     {
//         nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//     }
//     time0 = clock();
//     nRet = HRIF_Blackout(0);
//     Sleep(10000);
//     while (nCurFSM != 7)
//     {   
//         nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//     }
//     while (nCurFSM != 7)
//     {
//         nRet = HRIF_ReadCurFSM(0,0, nCurFSM, strCurFSM);
//     }
//     time1 = clock();
//     cout << "Runtime:" << (double)((time1 - time0) * 1000 / CLOCKS_PER_SEC-5000)<< "ms" << endl;
// }

int Positon_Cur_Speed::HR_ReadCmdJointVel()
{
    double Joint[6] = { 0 };
    int nRet = HRIF_ReadCmdJointVel(0,0, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadCmdJointVel");
        return nRet;
    }
    cout << "HRIF_ReadCmdJointVel调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << Joint[i] << endl;
    }
    return nRet;
}

int Positon_Cur_Speed::HR_ReadActJointVel()
{
    double dJ[6] = { 0 };
    int nRet = HRIF_ReadActJointVel(0,0, dJ[0], dJ[1], dJ[2], dJ[3], dJ[4], dJ[5]);
    if (nRet != 0)
    {
        err("HRIF_ReadActJointVel");
        return nRet;
    }
    cout << "HRIF_ReadActJointVel调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << dJ[i] << endl;
    }
    return nRet;
}



int Positon_Cur_Speed::HR_ReadCmdJointCur()
{
    double x;
    double y;
    double z;
    double dx;
    double dy;
    double dz;
    int nRet = HRIF_ReadCmdJointCur(0,0, x, y, z, dx, dy, dz);
    if (nRet != 0)
    {
        err("HRIF_ReadCmdJointCur");
        return nRet;
    }
    cout << "HRIF_ReadCmdJointCur调用成功！" << endl;
    cout << x << endl;
    cout << y << endl;
    cout << z << endl;
    cout << dx << endl;
    cout << dy << endl;
    cout << dz << endl;
    return nRet;
}

int Move::HR_IsMotionDone()
{
    bool Y;
    int nRet = HRIF_IsMotionDone(0,0, Y);
    if (nRet != 0)
    {
        err("HRIF_IsMotionDone");
        return nRet;
    }
    cout << "HRIF_IsMotionDone调用成功！" << endl;
    cout << Y << endl;
    return nRet;
}

int PoseCal::HR_GetForwardKin()
{
    double Joint[6] = {10,0,90,0,90,0 };
    double TCP[6] = { 0,0,0,0,0,0 };
    double UCS[6] = { 0,0,0,0,0,0 };
    double a[6] = { 0 };
    int nRet = HRIF_GetForwardKin(0,0, Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], TCP[0], TCP[1], TCP[2], TCP[3], TCP[4], TCP[5], UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5], a[0], a[1], a[2], a[3], a[4], a[5]);
    if (nRet != 0)
    {
        err("HRIF_GetForwardKin");
        return nRet;
    }
    cout << "HRIF_GetForwardKin调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << a[i] << endl;
    }
    return nRet;
}

int PoseCal::HR_Base2UcsTcp()
{
    double Joint[6] = { 719.947,118.428,764.469,-180,11.345,-171};
    double TCP[6] = { 0,0,0,0,0,0 };
    double UCS[6] = { 700,0,692,180,0,180};
    double a[6] = { 0 };
    int nRet = HRIF_Base2UcsTcp(0, 0,Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], TCP[0], TCP[1], 
        TCP[2], TCP[3], TCP[4], TCP[5], UCS[0], UCS[1], UCS[2], UCS[3], UCS[4], UCS[5], a[0], a[1], a[2], a[3], a[4], a[5]);
    if (nRet != 0)
    {
        err("HRIF_Base2UcsTcp");
        return nRet;
    }
    cout << "HRIF_Base2UcsTcp调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << a[i] << endl;
    }
    return nRet;
}

int PoseCal::HR_UcsTcp2Base()
{
    double joint[6] = {-19.945,118.430,-72.483,0,-11.346,-9.342};
    double tcp[6] = { 0,0,0,0,0,0 };
    double ucs[6] = { 700,0,692,180,0,180 };
    double a[6] = { 0 };
    int nRet = HRIF_UcsTcp2Base(0, 0,joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], tcp[0], tcp[1], tcp[2], tcp[3],
        tcp[4], tcp[5], ucs[0], ucs[1], ucs[2], ucs[3], ucs[4], ucs[5], a[0], a[1], a[2], a[3], a[4], a[5]);
    if (nRet != 0)
    {
        err("HRIF_UcsTcp2Base");
        return nRet;
    }
    cout << "HRIF_UcsTcp2Base调用成功！" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << a[i] << endl;
    }
    return nRet;

}

int main()
{
   InitRobot Robot;
   AxisControl Axis;
   ScriptControl Script;
   ReadAndSet  Set;
   Positon_Cur_Speed a;
   PoseCal Pose;
   TcpAndUcs Tcp;
   ForceControl Force;
   BoxControl Box;
   Move b;
   MovePathControl MovePath;
   ServoControl Servo;
   MoveTrace c;
   Others d;
  Robot.HR_Connect();



     
    return 0;
}

#include <iostream>
#include "Eigen/Dense"
#include "rough_positioning.h"
#include "accurate_positioning.h"
#include "Admittance.h"

#include "HansRobot.h"
#include "HR_Pro.h"
#include "ros/ros.h"

#include <vector>
#include <string>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_func_node");
    ros::NodeHandle nh;

    // 一些复位操作
    MatrixXd init_joint(6, 1);
    init_joint << 106.498, 16.633, -31.432, -2.864, -133.403, 16.757;

    HansRobot hb;
    
    string strCPSIP("192.168.0.10");
    unsigned short nCmdPort = 10003;
    int boxID = 0;

    if(!hb.HR_IsConnected())
    {
        int res = hb.HR_Connect(boxID, strCPSIP.c_str(), nCmdPort);
        if (res != 0)
            exit(0);
    }

    double Base[6] = { 0 };
    double Joint[6] = { 0 };
    double Tcp[6] = { 0 };
    double Ucs[6] = { 0 };

    hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
    // 当前关节角 
    MatrixXd cur_joint(6, 1);
    cur_joint.data() = Joint;

    double TCP[6] = { 0, 0, 0, 0, 0, 0};
    string TcpName = "TCP";
    string UcsName = "Base";
    double Vel = 50;
    double Acc = 100;
    double r = 1;
    int IsUseJoint = 1;
    int Iseek = 0;
    int IOBit = 0;
    int IOState = 0;
    string ID = "0";

    if ((cur_joint - init_joint).norm() > 0.001)
    {
        hb.HR_MoveJ(0, 0, init_joint, TCP, TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    }

    hb.HR_DisConnect();

    // 粗定位 
    int res = rough_positioning(nh);

    // 精定位
    res = accurate_positioning(nh);

    // 末端二自由度的阻抗控制
    double frequency = 10.0;
    
    double m[4] = {5, 0, 0, 5};
    double d[4] = {9, 0, 0, 15};
    double k[4] = {5, 0, 0, 15};
    double dp[2] = {0.05, 0};

    // Parameters
    std::vector<double> M(m, m+4);
    std::vector<double> D(d, d+4);
    std::vector<double> K(k, k+4);
    std::vector<double> desired_pose(dp, dp+2);
    
    double arm_max_vel = 1.0;
    double arm_max_acc = 3;

    string cportName = "/dev/ttyUSB0";
    string fportName = "/dev/ttyUSB1";
    
    Admittance admittance(frequency,
                          M, D, K,
                          desired_pose,
                          arm_max_vel,
                          arm_max_acc,
                          cportName,
                          fportName);

    admittance.run();

    // 再复位
    if(!hb.HR_IsConnected())
    {
        int res = hb.HR_Connect(boxID, strCPSIP.c_str(), nCmdPort);
        if (res != 0)
            exit(0);
    }

    hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
    // 当前关节角 
    cur_joint.data() = Joint;
    if ((cur_joint - init_joint).norm() > 0.001)
    {
        hb.HR_MoveJ(0, 0, init_joint, TCP, TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    }
}
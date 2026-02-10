#include <iostream>
#include "HansRobot.h"
#include <vector>
#include <sys/time.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <string>
#include "HR_Pro.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "Admittance.h"



using namespace std;

//void jointCallback(const sensor_msgs::JointState& msg){

//    ROS_INFO("Current joint value: %.2f %.2f %.2f %.2f %.2f %.2f", msg.position[0], msg.position[1], msg.position[2],
//                                                                        msg.position[3], msg.position[4], msg.position[5]);  //输出显示
//}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "hans_robot_node");
    ros::NodeHandle nh;
    
    /*ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("elfin_basic_api/joint_goal", 10);
    
    sensor_msgs::JointState js;
    
    js.header.stamp = ros::Time::now(); 
    js.name = {"elfin_joint1", "elfin_joint2", "elfin_joint3",
                 "elfin_joint4", "elfin_joint5", "elfin_joint6"};
    js.position = {0.5540, 0.0208, -0.1006, -0.0503, -2.0132, 0.0070};
    
    ros::Rate loop_rate(10.0);
    
    joint_pub.publish(js);
    
    while (ros::ok())
    {
        joint_pub.publish(js);
        loop_rate.sleep();  //设置休眠;
    }
    
    //ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointCallback);
    
    ros::spin();
    
    return 0;*/

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
    
    double TCP[6] = { 0, 0, 0, 0, 0, 0};
    string TcpName = "TCP";
    string UcsName = "Base";
    double Vel = 80;
    double Acc = 360;
    double r = 1;
    int IsUseJoint = 1;
    int Iseek = 0;
    int IOBit = 0;
    int IOState = 0;
    string ID = "0";
    
    double joint_init[6] = {106.498, 16.633, -31.432, -2.864, -133.403, 16.757};
    double joint_goal[6] = {74.773, 30.809, -14.861, 0.007, -133.762, 16.709};
    
    Vector6d ji;
    ji << joint_init[0], joint_init[1], joint_init[2], joint_init[3], joint_init[4], joint_init[5];
    
    Vector6d jg;
    jg << joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3], joint_goal[4], joint_goal[5];
    
    Vector6d j_now;
    j_now << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5];
    
    if ((ji - j_now).norm() > 0.001)
    {
        hb.HR_MoveJ(0, 0, joint_init, TCP, TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    }

    sleep(1);    
//    double Joint2[6] = {89.56, 16.663, -34.475, 4.673, -127.644, 15};

    hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
    j_now << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5];
    
    while ((ji-j_now).norm() > 0.001)
    {
        hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
        j_now << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5];
        usleep(100000);
    }
    
    hb.HR_MoveJ(0, 0, joint_goal, TCP, TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    
    sleep(1); 
    
    hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
    j_now << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5];
    
    while ((jg-j_now).norm() > 0.001)
    {
        hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
        j_now << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5];
        usleep(100000);
    }
    
    admittance.run();
    
    hb.HR_MoveJ(0, 0, joint_init, TCP, TcpName, UcsName, Vel, Acc, r, IsUseJoint, Iseek, IOBit, IOState, ID);
    
    return 0;

}

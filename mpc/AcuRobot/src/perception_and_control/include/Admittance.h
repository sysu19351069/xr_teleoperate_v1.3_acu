#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include "ros/ros.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "SerialPort.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>

#include "std_msgs/Float32.h"


using namespace Eigen;

#define PI 3.1415926


class Admittance
{
public:
  typedef Matrix<double, 7, 1> Vector7d;
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 6, 6> Matrix6d;

  typedef Matrix<double, 2, 1> Vector2d;
  typedef Matrix<double, 2, 2> Matrix2d;

protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  ros::Publisher pub_vel_;

  // ADMITTANCE PARAMETERS:
  Matrix2d M_, D_, K_;

  // 订阅六维力传感器
  // ros::Subscriber sub_force_state_;

  // 发布实时位置、经阻抗控制后的目标位置
  //ros::Publisher pub_x_;
  //ros::Publisher pub_xdes_;

  // 接收到的六轴力
  Vector6d      force_external_;

  Vector6d      force_init_;
  // 目标速度
  Vector2d      acupuncture_desired_vel_adm_;

  // 目标加速度
  Vector2d      acupuncture_desired_accelaration;

  // 最新的环境力
  Vector6d      last_F_env;

  // 最新目标位置和速度
  Vector2d      X_last_;
  Vector2d      Xd_last_;

  // 原本要跟踪的目标位置
  Vector2d      desired_position_;

  // 位置误差
  Vector2d      error;

public:
  void run();
  serialPort fport;
  Admittance(ros::NodeHandle &n, double frequency,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      double arm_max_vel,
                      double arm_max_acc,
                      string fportName
                      );
  ~Admittance()
  {
    fport.ClosePort();
  }
  
  
private:
  // modify environment position
  // void calculate_Xe();

  // get force
  void perceive_force();

  void perceive_force_init();

  void cal_deltaf();

  // 计算阻抗控制的速度和加速度Control
  void compute_admittance();

  // 将命令发送给机器人
  void send_commands_to_robot();

  double sign(double x){
    if(x > 0) return 1;
    else if(x < 0) return -1;
    else return 0;
  }

// private:
//   std::string   base_link_;
//   std::string   end_link_;
  double arm_max_vel_;
  double arm_max_acc_;

  int flag, flag1, flag1_5, flag2, flag3 = 1;

  double deltaf1 = 0;
  double deltaf2 = 0;
};

#endif // ADMITTANCE_H

#ifndef ROUGH_POSITIONING_H_
#define ROUGH_POSITIONING_H_


#include <iostream>
#include "Eigen/Dense"
#include "mpc2qp.h"
#include "mpc2qp_rough_positioning.h"
#include "cpp/declarations.h"
#include "cpp/transforms.h"
#include "cpp/jacobians.h"
#include "cpp/inverse_kinematics.h"
#include "qpOASES.hpp"
#include "HR_Pro.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <string>


using namespace Eigen;
using namespace std;
using namespace AcuRobot::rcg;
USING_NAMESPACE_QPOASES

int NUM_JONT = 8;
double T = 1;
double dt = 0.1;
int NUM_CTRL = round(T/dt);

// 8-dof 关节角值
MatrixXd qk = MatrixXd::Zero(8, 1);

int rough_positioning(ros::NodeHandle& nh);

#endif
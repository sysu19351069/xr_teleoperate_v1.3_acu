#ifndef RCG_ACUROBOT_INVERSE_KINEMATICS_H_
#define RCG_ACUROBOT_INVERSE_KINEMATICS_H_

#include <iostream>
#include "Eigen/Dense"

#include "transforms.h"
#include "jacobians.h"

using namespace Eigen;
using namespace std;
using namespace AcuRobot::rcg;


int ikine(const MatrixXd& T, const MatrixXd& q0, MatrixXd& qt, double tol, int maxiter);





#endif
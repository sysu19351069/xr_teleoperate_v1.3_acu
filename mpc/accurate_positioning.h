#ifndef ACCURATE_POSITIONING_H_
#define ACCURATE_POSITIONING_H_


#include <iostream>
#include "Eigen/Dense"
#include "mpc2qp.h"
#include "cpp/declarations.h"
#include "cpp/transforms.h"
#include "cpp/jacobians.h"
#include "cpp/inverse_kinematics.h"
#include "qpOASES.hpp"

#include <vector>
#include <string>


using namespace Eigen;
using namespace std;
using namespace AcuRobot::rcg;
// USING_NAMESPACE_QPOASES



int accurate_positioning();

#endif
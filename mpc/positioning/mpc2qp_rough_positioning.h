#ifndef MPC2QP_ROUGH_POSITIONING_H_
#define MPC2QP_ROUGH_POSITIONING_H_

#include <iostream>
#include "Eigen/Dense"

#include "cpp/transforms.h"
#include "cpp/jacobians.h"

using namespace AcuRobot::rcg;

using namespace Eigen;
using namespace std;

int mpc2qp_rough_positioning(const int NUM_CTRL, const int NUM_JONT, const double dt,
            const Ref<const MatrixXd> Q,
            const Ref<const MatrixXd> R,
            const Ref<const MatrixXd> T,
            const Ref<const MatrixXd> A,
            const Ref<const MatrixXd> B,
            const Ref<const MatrixXd> Av,
            const Ref<const MatrixXd> Bv,
            const Ref<const MatrixXd> Jacobk,
            const Ref<const MatrixXd> qk, 
            const Ref<const MatrixXd> qkd,
            const Ref<const MatrixXd> qk_1,
            const Ref<const MatrixXd> uk_1,
            const Ref<const Vector3d> Sd,
            const Ref<const Vector3d> Vv,
            const double dotValue,
            const Ref<const MatrixXd> theta,
            const Ref<const MatrixXd> omega,
            const Ref<const MatrixXd> psi,
            const Ref<const MatrixXd> Theta,
            const Ref<const MatrixXd> Psi,
            const Ref<const MatrixXd> Theta_plus,
            MatrixXd& Epsilon,
            MatrixXd& Delta,
            MatrixXd& H,
            MatrixXd& g,
            MatrixXd& G,
            MatrixXd& lbA,
            MatrixXd& ubA
);


#endif
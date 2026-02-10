#include <iostream>
#include "Eigen/Dense"

#include "mpc2qp.h"
#include "mpc2qp_rough_positioning.h"
#include "cpp/declarations.h"
#include "cpp/transforms.h"
#include "cpp/jacobians.h"
#include "cpp/inverse_kinematics.h"
#include "qpOASES.hpp"
#include "accurate_positioning.h"

using namespace Eigen;
using namespace std;
// using namespace AcuRobot::rcg;
// USING_NAMESPACE_QPOASES

// int NUM_JONT = 8;
// double T = 1;
// double dt = 0.1;
// int NUM_CTRL = round(T/dt);

// int main()
// {
//     int res = accurate_positioning();
// }


// 精定位
// int main()
// {

//     MatrixXd A = MatrixXd::Identity(NUM_JONT, NUM_JONT);
//     MatrixXd B = dt * MatrixXd::Identity(NUM_JONT, NUM_JONT);

//     MatrixXd Av(2*NUM_JONT, 2*NUM_JONT);
//     Av << A, MatrixXd::Zero(NUM_JONT, NUM_JONT), A, MatrixXd::Identity(NUM_JONT, NUM_JONT);
//     MatrixXd Bv(2*NUM_JONT, NUM_JONT);
//     Bv << B, B;

//     MatrixXd uk_1 = MatrixXd::Zero(NUM_JONT, 1);

//     // Theta定义
//     cout << Av;
//     MatrixXd Theta = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         for(int j = 0; j <= i; j++)
//         {
//             Theta.block((2 * NUM_JONT) * i, NUM_JONT * j, 2 * NUM_JONT, NUM_JONT).noalias() = matrixPower(Av, i-j) * Bv;
//         }
//     }

//     // Psi定义
//     MatrixXd Psi = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         MatrixXd tmp = matrixPower(Av, i+1);
//         Psi.block((2 * NUM_JONT) * i, NUM_JONT * NUM_CTRL, 2 * NUM_JONT, NUM_JONT) = tmp.block(0, NUM_JONT, 2 * NUM_JONT, NUM_JONT);
//     }

//     MatrixXd Theta_plus = Theta - Psi;

//     // Delta在函数里更新，函数外只做初始化的定义
//     // MatrixXd Delta = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, 1);
//     MatrixXd Delta(2 * NUM_JONT * NUM_CTRL, 1);
    
//     // 海森矩阵，梯度
//     MatrixXd H = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));
//     MatrixXd g = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), 1);
//     // 测试例子
//     MatrixXd qk(NUM_JONT, 1);
//     // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 0.0180033, 0.2709;
//     // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 18.0033, 0.2709;

//     qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 0.018, 0;
//     // qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 18.00, 0;

//     MatrixXd qk_1 = qk;

//     MatrixXd Epsilon = Delta;

//     MatrixXd Qi = MatrixXd::Identity(2 * NUM_JONT, 2 * NUM_JONT);

//     MatrixXd Ri = 0.1 * MatrixXd::Identity(NUM_JONT, NUM_JONT);

//     MatrixXd Q(2 * NUM_JONT * NUM_CTRL, 2 * NUM_JONT * NUM_CTRL);
//     Q = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, 2 * NUM_JONT * NUM_CTRL);

//     // 终点跟踪权重矩阵 P(经过极点配置之后)
//     MatrixXd P = MatrixXd::Zero(2 * NUM_JONT, 2 * NUM_JONT);
//     P(0, 0) = 19.1677; P(1, 1) = 41.7573; P(2, 2) = 31.6742; P(3, 3) = 35.2202;
//     P(4, 4) = 39.2995; P(5, 5) = 43.9919; P(6, 6) = 49.3902; P(7, 7) = 55.6025;
//     P(8, 8) = 366.2165; P(9, 9) = 685.1713; P(10, 10) = 560.9258; P(11, 11) = 607.9773;
//     P(12, 12) = 659.7306; P(13, 13) = 716.7489; P(14, 14) = 779.6759; P(15, 15) = 849.2492;

//     P(0, 8) = -43.0184; P(1, 9) = -129.7105; P(2, 10) = -93.5113; P(3, 11) = -106.6855;
//     P(8, 0) = -43.0184; P(9, 1) = -129.7105; P(10, 2) = -93.5113; P(11, 3) = -106.6855;

//     P(4, 12) = -121.4779; P(5, 13) = -138.1033; P(6, 14) = -156.8090; P(7, 15) = -177.8805;
//     P(12, 4) = -121.4779; P(13, 5) = -138.1033; P(14, 6) = -156.8090; P(15, 7) = -177.8805;

//     // 辅助变量与qkd的跟踪误差系数矩阵 T
//     MatrixXd T = 10 * P.block(NUM_JONT, NUM_JONT, NUM_JONT, NUM_JONT);

//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         if(i != NUM_CTRL - 1)
//         {
//             Q.block((2 * NUM_JONT) * i, (2 * NUM_JONT) * i, 2 * NUM_JONT, 2 * NUM_JONT) = Qi;
//         }
//         else
//         {
//             Q.block((2 * NUM_JONT) * i, (2 * NUM_JONT) * i, 2 * NUM_JONT, 2 * NUM_JONT) = P;
//         }
//     }

//     MatrixXd R(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));
//     R = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));

//     for(int i = 0; i < NUM_CTRL + 1; i++)
//     {
//         if(i != NUM_CTRL)
//         {
//             R.block(NUM_JONT * i, NUM_JONT * i, NUM_JONT, NUM_JONT) = Ri;
//         }
//         else
//         {
//             R.block(NUM_JONT * i, NUM_JONT * i, NUM_JONT, NUM_JONT) = T;;
//         }
//     }

//     // 关节角与控制量的关系
//     MatrixXd theta = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         for(int j = 0; j <= i; j++)
//         {
//             MatrixXd tmp = MatrixXd::Zero(NUM_JONT, NUM_JONT);
//             for(int k = 0; k <= i - j; k++)
//             {
//                 tmp += matrixPower(A, k) * B;
//             }
//             theta.block(NUM_JONT * i, NUM_JONT * j, NUM_JONT, NUM_JONT) = tmp;
//         }
//     }

//     MatrixXd omega = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT);
//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         MatrixXd tmp = MatrixXd::Zero(NUM_JONT, NUM_JONT);
//         for(int k = 0; k <= i; k++)
//         {
//             tmp += matrixPower(A, k) * B;
//         }
//         omega.block(NUM_JONT * i, 0, NUM_JONT, NUM_JONT) = tmp;
//     }

//     MatrixXd psi = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT);
//     for(int i = 0; i < NUM_CTRL; i++)
//     {
//         psi.block(NUM_JONT * i, 0, NUM_JONT, NUM_JONT) = matrixPower(A, i+1);
//     }

//     // 总约束矩阵 G（标准式中的A）
//     MatrixXd G = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 1 + 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));

//     // upper bound ubA, lower bound lbA
//     MatrixXd lbA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 1 + 2 * NUM_JONT, 1);
//     MatrixXd ubA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 1 + 2 * NUM_JONT, 1);

//     MatrixXd qkd(NUM_JONT, 1);

//     // 规划示例
//     Vector3d v;
//     v << 1, 1, 2;
//     Vector3d z;
//     z << 0, 0, 1;

//     Vector3d vn = z.cross(v);  // 叉积操作（可能有问题）
//     double angle = acos(v.dot(z)/(v.norm() * z.norm()));
//     AngleAxisd vn_rotate(angle, vn.normalized());

//     Matrix4d trans;
//     trans.setIdentity();
//     trans.block(0, 0, 3, 3) = vn_rotate.toRotationMatrix();

//     HomogeneousTransforms acubot;
//     Matrix4d curpos = acubot.fr_base0_X_ee(qk);

//     double r = 0.05;  // 针灸长度
//     // double r = 50;  // 针灸长度
//     Matrix4d tl;
//     tl.setIdentity();
//     tl.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);
//     tl.topRightCorner(3, 1) = -v;

//     Matrix4d f_T = curpos * tl;
//     Vector3d dO2 = r * (f_T.topRightCorner(3, 1) - curpos.topRightCorner(3, 1)).normalized();

//     Matrix4d t2;
//     t2.setIdentity();
//     t2.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);
//     t2.topRightCorner(3, 1) << 0, 0, r;

//     Matrix4d c_T = curpos * t2;
//     Vector3d dO1 = curpos.topRightCorner(3, 1) - c_T.topRightCorner(3, 1);

//     Vector3d d12 = dO2 - dO1;
//     Vector3d FP = c_T.topRightCorner(3, 1) + dO2;

//     MatrixXd obj = curpos * trans;
//     obj.topRightCorner(3, 1) = FP;

//     // 需要写一个求逆运动学的函数
//     // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 0.0179988, -0.0124;
//     // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 17.9988, -0.0124;

//     qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 0.0180050, 0.2975;
//     // qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 18.0050, 0.2975;

//     Jacobians acubot_jacob;

//     MatrixXd qt = MatrixXd::Zero(NUM_JONT, 1);

//     double *qtcur = curpos.data();
//     double *qtobj = obj.data();
//     int rv = ikine(obj, qk, qt, 1e-3, 1000);

//     double *qtd = qt.data();

//     Matrix4d testpose = acubot.fr_base0_X_ee(qt);

//     int nV = NUM_JONT * (NUM_CTRL + 1);
//     int nC = 3 * NUM_JONT * NUM_CTRL + 1 + 2 * NUM_JONT;
//     SQProblem mpc(nV, nC);
//     int nWSR = 5*(nV + nC);

//     int itermax = 50;
//     for(int i = 0; i < itermax; i++)
//     {
//         Vector3d OP = curpos.topRightCorner(3, 1) - c_T.topRightCorner(3, 1);
//         Vector3d OPr = r * OP.normalized();
//         Vector3d rotate_axis = OPr.cross(dO2);
//         double anglesum = acos(OPr.dot(dO2)/(r*r));
//         AngleAxisd h((anglesum/NUM_CTRL) * min(i+1, NUM_CTRL), rotate_axis.normalized());

//         Vector3d OM = h.toRotationMatrix() * OPr;
//         Matrix3d rtm = h.toRotationMatrix();
//         double b1 = OM(0, 0);
//         double b2 = OM(1, 0);
//         double b3 = OM(2, 0);

//         Vector3d Sd = OM - OP;

//         double a1 = Sd(0, 0);
//         double a2 = Sd(1, 0);
//         double a3 = Sd(2, 0);

//         Vector3d Vv = OP + 0.5 * Sd;
//         double dotValue = Vv.dot(Sd);

//         MatrixXd jacobk_t = acubot_jacob.fr_base0_J_ee(qk).block(3, 0, 3, 8);
//         int tr = mpc2qp(NUM_CTRL, NUM_JONT, dt,
//                 Q, R, T, A, B, Av, Bv,
//                 jacobk_t, qk, qkd, qk_1,
//                 uk_1, Sd, Vv, dotValue,
//                 theta, omega, psi, Theta,
//                 Psi, Theta_plus, Delta,
//                 Epsilon, H, g, G, lbA, ubA);

//         if(i == 0)
//         {
//             int returnvalue = mpc.init(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
//         }
//         else
//         {
//             int returnvalue = mpc.hotstart(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
//         }

//         cout << mpc.getObjVal() << endl;

//     }

// }

// 粗定位
int main()
{

    MatrixXd A = MatrixXd::Identity(NUM_JONT, NUM_JONT);
    MatrixXd B = dt * MatrixXd::Identity(NUM_JONT, NUM_JONT);

    MatrixXd Av(2*NUM_JONT, 2*NUM_JONT);
    Av << A, MatrixXd::Zero(NUM_JONT, NUM_JONT), A, MatrixXd::Identity(NUM_JONT, NUM_JONT);
    MatrixXd Bv(2*NUM_JONT, NUM_JONT);
    Bv << B, B;

    MatrixXd uk_1 = MatrixXd::Zero(NUM_JONT, 1);

    // Theta定义
    cout << Av;
    MatrixXd Theta = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            Theta.block((2 * NUM_JONT) * i, NUM_JONT * j, 2 * NUM_JONT, NUM_JONT).noalias() = matrixPower(Av, i-j) * Bv;
        }
    }

    // Psi定义
    MatrixXd Psi = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        MatrixXd tmp = matrixPower(Av, i+1);
        Psi.block((2 * NUM_JONT) * i, NUM_JONT * NUM_CTRL, 2 * NUM_JONT, NUM_JONT) = tmp.block(0, NUM_JONT, 2 * NUM_JONT, NUM_JONT);
    }

    MatrixXd Theta_plus = Theta - Psi;

    // Delta在函数里更新，函数外只做初始化的定义
    // MatrixXd Delta = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, 1);
    MatrixXd Delta(2 * NUM_JONT * NUM_CTRL, 1);
    
    // 海森矩阵，梯度
    MatrixXd H = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));
    MatrixXd g = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), 1);
    // 测试例子
    MatrixXd qk(NUM_JONT, 1);
    // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 0.0180033, 0.2709;
    // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 18.0033, 0.2709;

    qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 0.018, 0;
    // qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 18.00, 0;

    MatrixXd qk_1 = qk;

    MatrixXd Epsilon = Delta;

    MatrixXd Qi = MatrixXd::Identity(2 * NUM_JONT, 2 * NUM_JONT);

    MatrixXd Ri = 0.1 * MatrixXd::Identity(NUM_JONT, NUM_JONT);

    MatrixXd Q(2 * NUM_JONT * NUM_CTRL, 2 * NUM_JONT * NUM_CTRL);
    Q = MatrixXd::Zero(2 * NUM_JONT * NUM_CTRL, 2 * NUM_JONT * NUM_CTRL);

    // 终点跟踪权重矩阵 P(经过极点配置之后)
    MatrixXd P = MatrixXd::Zero(2 * NUM_JONT, 2 * NUM_JONT);
    P(0, 0) = 19.1677; P(1, 1) = 41.7573; P(2, 2) = 31.6742; P(3, 3) = 35.2202;
    P(4, 4) = 39.2995; P(5, 5) = 43.9919; P(6, 6) = 49.3902; P(7, 7) = 55.6025;
    P(8, 8) = 366.2165; P(9, 9) = 685.1713; P(10, 10) = 560.9258; P(11, 11) = 607.9773;
    P(12, 12) = 659.7306; P(13, 13) = 716.7489; P(14, 14) = 779.6759; P(15, 15) = 849.2492;

    P(0, 8) = -43.0184; P(1, 9) = -129.7105; P(2, 10) = -93.5113; P(3, 11) = -106.6855;
    P(8, 0) = -43.0184; P(9, 1) = -129.7105; P(10, 2) = -93.5113; P(11, 3) = -106.6855;

    P(4, 12) = -121.4779; P(5, 13) = -138.1033; P(6, 14) = -156.8090; P(7, 15) = -177.8805;
    P(12, 4) = -121.4779; P(13, 5) = -138.1033; P(14, 6) = -156.8090; P(15, 7) = -177.8805;

    // 辅助变量与qkd的跟踪误差系数矩阵 T
    MatrixXd T = 10 * P.block(NUM_JONT, NUM_JONT, NUM_JONT, NUM_JONT);

    for(int i = 0; i < NUM_CTRL; i++)
    {
        if(i != NUM_CTRL - 1)
        {
            Q.block((2 * NUM_JONT) * i, (2 * NUM_JONT) * i, 2 * NUM_JONT, 2 * NUM_JONT) = Qi;
        }
        else
        {
            Q.block((2 * NUM_JONT) * i, (2 * NUM_JONT) * i, 2 * NUM_JONT, 2 * NUM_JONT) = P;
        }
    }

    MatrixXd R(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));
    R = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));

    for(int i = 0; i < NUM_CTRL + 1; i++)
    {
        if(i != NUM_CTRL)
        {
            R.block(NUM_JONT * i, NUM_JONT * i, NUM_JONT, NUM_JONT) = Ri;
        }
        else
        {
            R.block(NUM_JONT * i, NUM_JONT * i, NUM_JONT, NUM_JONT) = T;;
        }
    }

    // 关节角与控制量的关系
    MatrixXd theta = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            MatrixXd tmp = MatrixXd::Zero(NUM_JONT, NUM_JONT);
            for(int k = 0; k <= i - j; k++)
            {
                tmp += matrixPower(A, k) * B;
            }
            theta.block(NUM_JONT * i, NUM_JONT * j, NUM_JONT, NUM_JONT) = tmp;
        }
    }

    MatrixXd omega = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        MatrixXd tmp = MatrixXd::Zero(NUM_JONT, NUM_JONT);
        for(int k = 0; k <= i; k++)
        {
            tmp += matrixPower(A, k) * B;
        }
        omega.block(NUM_JONT * i, 0, NUM_JONT, NUM_JONT) = tmp;
    }

    MatrixXd psi = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        psi.block(NUM_JONT * i, 0, NUM_JONT, NUM_JONT) = matrixPower(A, i+1);
    }

    // 总约束矩阵 G（标准式中的A）
    MatrixXd G = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 4 + 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));

    // upper bound ubA, lower bound lbA
    MatrixXd lbA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 4 + 2 * NUM_JONT, 1);
    MatrixXd ubA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 4 + 2 * NUM_JONT, 1);

    MatrixXd qkd(NUM_JONT, 1);

    // 规划示例

    HomogeneousTransforms acubot;
    Matrix4d curpos = acubot.fr_base0_X_ee(qk);

    // 需要写一个求逆运动学的函数
    // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 0.0179988, -0.0124;
    // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 17.9988, -0.0124;

    qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 0.0180050, 0.2975;
    // qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 18.0050, 0.2975;

    Jacobians acubot_jacob;

    MatrixXd qt = MatrixXd::Zero(NUM_JONT, 1);

    // 这个坐标需经相机传感器进行获取
    MatrixXd obj_rough = curpos;
    obj_rough.topRightCorner(3, 1) << -0.250, -0.750, 0;
    double *qtcur = curpos.data();
    double *qtobj = obj_rough.data();
    // obj_rough.topRightCorner(3, 1) << -250, 750, 0;

    int rv = ikine(obj_rough, qk, qt, 1e-3, 1500);

    double *qtd = qt.data();
    const double *se3qt = acubot.fr_base0_X_ee(qt).data();

    qkd = qt;

    // 方案一：以直角线型规划到穴位处
    Vector3d P0 = curpos.topRightCorner(3, 1);
    Vector3d O = obj_rough.topRightCorner(3, 1);
    Vector3d T0;
    T0 << O.topLeftCorner(2, 1), P0(2);
    Vector3d Pv = (T0 - P0).normalized();
    Vector3d Tv = (O - T0).normalized();

    int flag = 0;

    int nV = NUM_JONT * (NUM_CTRL + 1);
    int nC = 3 * NUM_JONT * NUM_CTRL + 4 + 2 * NUM_JONT;
    SQProblem mpc(nV, nC);
    int nWSR = 5*(nV + nC);

    int itermax = 50;
    for(int i = 0; i < itermax; i++)
    {
        Vector3d Sd;
        Vector3d Vv;
        Vector3d P = curpos.topRightCorner(3, 1);
        if((T0 - P).norm() < 0.005)
        {
            flag = 1;
        }

        if(!flag)
        {
            Vector3d w = P - P0;
            Vector3d P_proj = P0 + w.dot(Pv) * Pv;
            // Vector3d PM = P_proj + (TO - P_proj) / (2 * NUM_CTRL) * min(i, 2 * NUM_CTRL);
            Vector3d PM = P_proj + (T0 - P0) / (2 * NUM_CTRL);
            Sd = PM - P;
            Vv = Tv;
        }
        else
        {
            Vector3d w = P - T0;
            Vector3d P_proj = T0 + w.dot(Tv) * Tv;
            // Vector3d PM = P_proj + (O - P_proj) / (2 * NUM_CTRL) * min(i, 2 * NUM_CTRL);
            Vector3d PM = P_proj + (O - T0) / (5 * NUM_CTRL);
            Sd = PM - P;
            Vv = Pv;
        }

        double dotValue = Vv.dot(Sd);

        MatrixXd jacobk_t = acubot_jacob.fr_base0_J_ee(qk);
        int tr = mpc2qp_rough_positioning(NUM_CTRL, NUM_JONT, dt,
                Q, R, T, A, B, Av, Bv,
                jacobk_t, qk, qkd, qk_1,
                uk_1, Sd, Vv, dotValue,
                theta, omega, psi, Theta,
                Psi, Theta_plus, Delta,
                Epsilon, H, g, G, lbA, ubA);

        if(i == 0)
        {
            int returnvalue = mpc.init(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
        }
        else
        {
            int returnvalue = mpc.hotstart(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
        }

        cout << mpc.getObjVal() << endl;

    }

}
#include "mpc2qp_rough_positioning.h"
#include "mpc2qp.h"


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
){

    MatrixXd dqk = qk - qk_1;

    MatrixXd dqk_qk(2 * NUM_JONT, 1);
    dqk_qk << dqk, qk;

    // Delta在函数里更新
    for(int i = 0; i < NUM_CTRL; i++)
    {
        MatrixXd tmp = matrixPower(Av, i + 1);
        Delta.block((2 * NUM_JONT) * i, 0, 2 * NUM_JONT, 1).noalias() = tmp * dqk_qk;
    }

    Epsilon = -Delta;

    // 第一步的目标跟踪
    MatrixXd tmp(NUM_JONT, NUM_JONT * (NUM_CTRL + 1));
    tmp << MatrixXd::Identity(NUM_JONT, NUM_JONT), MatrixXd::Zero(NUM_JONT, NUM_JONT * NUM_CTRL);

    MatrixXd thetapp(3, NUM_JONT * (NUM_CTRL + 1));
    
    MatrixXd JacobkT = Jacobk.block(3, 0, 3, 8);
    thetapp.noalias() = dt * JacobkT * tmp;

    MatrixXd guk(NUM_JONT * (NUM_CTRL + 1), 1);

    double w = 0.5;

    guk = w * thetapp.transpose() * (Sd - JacobkT * uk_1);

    MatrixXd gqd(NUM_JONT * (NUM_CTRL + 1), 1);
    gqd = MatrixXd::Zero(NUM_JONT * (NUM_CTRL + 1), 1);

    gqd.block(NUM_JONT * NUM_CTRL, 0, NUM_JONT, 1).noalias() = T*qkd;

    g.noalias() = -2 * (Theta_plus.transpose() * Q * Epsilon + gqd + guk);

    H.noalias() = 2 * (Theta_plus.transpose() * Q * Theta_plus + R + w * thetapp.transpose() * thetapp);

    // 关节角约束
    MatrixXd qmin = -2 * M_PI * MatrixXd::Ones(NUM_JONT, 1);
    qmin(6, 0) = 0;
    MatrixXd Qmin(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        Qmin.block(NUM_JONT * i, 0, NUM_JONT, 1) = qmin;
    }

    MatrixXd qmax = 2 * M_PI * MatrixXd::Ones(NUM_JONT, 1);
    qmax(6, 0) = 30;
    MatrixXd Qmax(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        Qmax.block(NUM_JONT * i, 0, NUM_JONT, 1) = qmax;
    }
    
    lbA.block(0, 0, NUM_JONT * NUM_CTRL, 1) = Qmin - psi * qk - omega * uk_1;
    ubA.block(0, 0, NUM_JONT * NUM_CTRL, 1) = Qmax - psi * qk - omega * uk_1;

    // 关节角速度约束
    MatrixXd Gq_dot = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            Gq_dot.block(NUM_JONT * i, NUM_JONT * j, NUM_JONT, NUM_JONT) = MatrixXd::Identity(NUM_JONT, NUM_JONT);
        }
    }
    G.block(NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1)) = Gq_dot;

    MatrixXd umin = -0.5 * MatrixXd::Ones(NUM_JONT, 1);
    umin(6, 0) = -5;
    MatrixXd Umin(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        Umin.block(NUM_JONT * i, 0, NUM_JONT, 1) = umin;
    }

    MatrixXd umax = 0.5 * MatrixXd::Ones(NUM_JONT, 1);
    umax(6, 0) = 5;
    MatrixXd Umax(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        Umax.block(NUM_JONT * i, 0, NUM_JONT, 1) = umax;
    }

    MatrixXd Uk_1(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        Uk_1.block(NUM_JONT * i, 0, NUM_JONT, 1) = uk_1;
    }
    lbA.block(NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, 1) = Umin - Uk_1;
    ubA.block(NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, 1) = Umax - Uk_1;

    // 关节角加速度约束
    MatrixXd Gq_dot_dot = MatrixXd::Identity(NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    G.block(2 * NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1)) = Gq_dot_dot;

    MatrixXd dumin = -0.5 * MatrixXd::Ones(NUM_JONT, 1);
    MatrixXd dUmin(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        dUmin.block(NUM_JONT * i, 0, NUM_JONT, 1) = dumin;
    }
    MatrixXd dumax = 0.5 * MatrixXd::Ones(NUM_JONT, 1);
    MatrixXd dUmax(NUM_JONT * NUM_CTRL, 1);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        dUmax.block(NUM_JONT * i, 0, NUM_JONT, 1) = dumax;
    }
    lbA.block(2 * NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, 1) = dUmin;
    ubA.block(2 * NUM_JONT * NUM_CTRL, 0, NUM_JONT * NUM_CTRL, 1) = dUmax;

    // 第一步状态转移约束: 保持绕x，y轴的角速度为0
    MatrixXd JacobkR = Jacobk.block(0, 0, 2, 8);
    MatrixXd Gv = MatrixXd::Zero(NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    Gv.block(0, 0, 2, NUM_JONT) = JacobkR;

    // HomogeneousTransforms robot;
    // Jacobians robjacob;

    // link2、link4、link6-7避障
    // MatrixXd jacob_l5e_1(6, 8);
    // jacob_l5e_1 << robjacob.fr_base0_J_link3_end(qk), MatrixXd::Zero(6, 5);
    // jacob_l5e_1 = jacob_l5e_1.block(3, 0, 3, 8);

    // // MatrixXd jacob_l5e_2 = robjacob.fr_base0_J_link4_end(qk);
    // MatrixXd jacob_l5e_2(6, 8);
    // jacob_l5e_2 << robjacob.fr_base0_J_link4_end(qk), MatrixXd::Zero(6, 4);
    // jacob_l5e_2 = jacob_l5e_2.block(3, 0, 3, 8);

    // Vector3d roj;   // 障碍物坐标
    // roj << -0.3662, -0.5899, 0.1;
    // roj << -0.3662, -1, 0.1;
    // Vector3d r1 = robot.fr_base0_X_link3_end(qk).topRightCorner(3, 1);
    // Vector3d r2 = robot.fr_base0_X_link4_end(qk).topRightCorner(3, 1);
    // Vector3d r1 = robot.fr_base0_X_fr_link6(qk).topRightCorner(3, 1);
    // Vector3d r2 = robot.fr_base0_X_ee(qk).topRightCorner(3, 1);

    // Vector3d r1 = robot.fr_base0_X_ee(qk).topRightCorner(3, 1);
    // Vector3d r2 = robot.fr_base0_X_fr_link5(qk).topRightCorner(3, 1);
    // Vector3d d1o = roj - r1;
    // Vector3d d12 = r2 - r1;

    // 找连杆距离障碍物的最小值
    // double dlj = (r1 - roj).norm();
    // double increment = 0.1;
    // while(increment <= 1)
    // {
    //     Vector3d pj = r1 + increment * d12;

    //     if((pj - roj).norm() < dlj)
    //     {
    //         dlj = (pj - roj).norm();
    //     }
    //     increment += 0.1;
    // }
    // Gv.block(2, 0, 1, NUM_JONT) = dt * d1o.transpose() * jacob_l5e_1;
    // Gv.block(3, 0, 1, NUM_JONT) = dt * (d1o.transpose() * jacob_l5e_2 - d12.transpose() * jacob_l5e_1);

    // Gv.block(2, 0, 1, NUM_JONT) = dt * d1o.transpose() * jacob_l5e_2;
    // Gv.block(3, 0, 1, NUM_JONT) = dt * (d1o.transpose() * jacob_l5e_1 - d12.transpose() * jacob_l5e_2);

    // MatrixXd jacob_l5e_1(6, 8);
    // jacob_l5e_1 << robjacob.fr_base0_J_fr_link6(qk), MatrixXd::Zero(6, 2);
    // jacob_l5e_1 = jacob_l5e_1.block(3, 0, 3, 8);
    // Vector3d roj;   // 障碍物坐标
    // roj << -0.3662, -0.5899, 0.1;
    // // roj << -0.3662, -1, 0.1;
    // Vector3d r1 = robot.fr_base0_X_fr_link6(qk).topRightCorner(3, 1);
    // Vector3d d1o = roj - r1;
    // double dlj = (r1 - roj).norm();
    // Gv.block(2, 0, 1, NUM_JONT) = dt * d1o.transpose() * jacob_l5e_1;

    MatrixXd tmp1(NUM_JONT * (NUM_CTRL + 1), NUM_JONT * (NUM_CTRL + 1));
    tmp1 << Gq_dot, MatrixXd::Zero(NUM_JONT, NUM_JONT * (NUM_CTRL + 1));

    MatrixXd GvGq_dot(NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    GvGq_dot = Gv * tmp1;

    G.block(3 * NUM_JONT * NUM_CTRL, 0, 2, NUM_JONT * (NUM_CTRL + 1)) = GvGq_dot.block(0, 0, 2, NUM_JONT * (NUM_CTRL + 1));
    // G.block(3 * NUM_JONT * NUM_CTRL, 0, 2, NUM_JONT * (NUM_CTRL + 1)) = GvGq_dot.block(0, 0, 2, NUM_JONT * (NUM_CTRL + 1));


    MatrixXd nUk_1(NUM_JONT * (NUM_CTRL + 1), 1);
    nUk_1 << Uk_1, MatrixXd::Zero(NUM_JONT, 1);
    MatrixXd tmp2 = Gv.block(0, 0, 2, NUM_JONT * (NUM_CTRL + 1)) * nUk_1;
    // MatrixXd tmp2 = Gv.block(0, 0, 2, NUM_JONT * (NUM_CTRL + 1)) * nUk_1;
    // 保持绕x，y轴的角速度为0
    // MatrixXd tmp3(4, 1);
    // MatrixXd tmp4(4, 1);
    // tmp3 << MatrixXd::Zero(2, 1), -1e8, -1e8;
    // tmp4 << MatrixXd::Zero(2, 1), dlj*dlj, dlj*dlj;
    
    // lbA.block(3 * NUM_JONT * NUM_CTRL, 0, 4, 1) = tmp3 - tmp2;
    // ubA.block(3 * NUM_JONT * NUM_CTRL, 0, 4, 1) = tmp4 - tmp2;

    lbA.block(3 * NUM_JONT * NUM_CTRL, 0, 2, 1) = MatrixXd::Zero(2, 1) - tmp2;
    ubA.block(3 * NUM_JONT * NUM_CTRL, 0, 2, 1) = MatrixXd::Zero(2, 1) - tmp2;
    

    // MOAS 约束
    MatrixXd T1(2 * NUM_JONT, 2 * NUM_JONT);
    T1 << A, B, MatrixXd::Zero(NUM_JONT, NUM_JONT), MatrixXd::Identity(NUM_JONT, NUM_JONT);

    MatrixXd T2(2 * NUM_JONT, 2 * NUM_JONT);
    T2 << A - MatrixXd::Identity(NUM_JONT, NUM_JONT), B, A, B;

    MatrixXd T3 = MatrixXd::Identity(2 * NUM_JONT, 3 * NUM_JONT);
    T3.block(NUM_JONT, 2 * NUM_JONT, NUM_JONT, NUM_JONT) = MatrixXd::Identity(NUM_JONT, NUM_JONT);

    MatrixXd Cv = T1 * T2.inverse() * T3;
    MatrixXd Ce = Cv.block(0, 0, 2 * NUM_JONT, 2 * NUM_JONT);
    MatrixXd Cq = Cv.block(0, 2 * NUM_JONT, 2 * NUM_JONT, NUM_JONT);

    MatrixXd Gmoas(2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));

    MatrixXd T4(2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));
    T4 << MatrixXd::Zero(2 * NUM_JONT, NUM_JONT * NUM_CTRL), Cq;

    Gmoas = Ce * Theta_plus.block(NUM_JONT * (NUM_CTRL - 1), 0, 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1)) + T4;

    G.block(3 * NUM_JONT * NUM_CTRL + 2, 0, 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1)) = Gmoas;

    MatrixXd CE(2 * NUM_JONT * NUM_CTRL, 2 * NUM_JONT * NUM_CTRL);
    for(int i = 0; i < NUM_CTRL; i++)
    {
        CE.block(2 * NUM_JONT * i, 2 * NUM_JONT * i, 2 * NUM_JONT, 2 * NUM_JONT) = Ce;
    }

    MatrixXd cq(2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));
    cq << MatrixXd::Zero(2 * NUM_JONT, NUM_JONT * NUM_CTRL), Cq;

    MatrixXd CQ(2 * NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        CQ.block(2 * NUM_JONT * i, 0, 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1)) = cq;
    }

    MatrixXd qmin_umin(2 * NUM_JONT, 1);
    qmin_umin << qmin, umin;
    MatrixXd qmax_umax(2 * NUM_JONT, 1);
    qmax_umax << qmax, umax;
    lbA.block(3 * NUM_JONT * NUM_CTRL + 2, 0, 2 * NUM_JONT, 1) = qmin_umin + Ce * Epsilon.block(2 * NUM_JONT * (NUM_CTRL - 1), 0, 2 * NUM_JONT, 1);
    ubA.block(3 * NUM_JONT * NUM_CTRL + 2, 0, 2 * NUM_JONT, 1) = qmax_umax + Ce * Epsilon.block(2 * NUM_JONT * (NUM_CTRL - 1), 0, 2 * NUM_JONT, 1);

    return 0;
}
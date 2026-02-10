#include "inverse_kinematics.h"

MatrixXd vex(const MatrixXd& S)
{
    MatrixXd v(3, 1);
    v << 0.5 * (S(2, 1)-S(1, 2)), 0.5 * (S(0, 2)-S(2, 0)), 0.5 * (S(1, 0)-S(0, 1));

    return v;
}


MatrixXd tr2delta(const MatrixXd& T0, const MatrixXd& T1)
{
    MatrixXd TD = T0.inverse() * T1;

    MatrixXd delta = MatrixXd::Zero(6, 1);

    delta.bottomLeftCorner(3, 1) = TD.block(0, 3, 3, 1);
    delta.topLeftCorner(3, 1) = vex(TD.block(0, 0, 3, 3) - MatrixXd::Identity(3, 3));

    return delta;
}

// 无碰撞避险
int ikine(const MatrixXd& T, const MatrixXd& q0, MatrixXd& qt, double tol, int maxiter, double dof = 8)
{
    int NUM_JONT = static_cast<int>(q0.rows());

    HomogeneousTransforms robot;
    Jacobians robjocob;

    double lambda = 10;
    double lambdamin = 0;

    int iter = 0;

    MatrixXd q = q0;
    MatrixXd q_new = q0;
    // 每个自由度的权重系数
    MatrixXd w(6, 1);
    w << 1, 1, 1, 1, 1, 1;
    MatrixXd W = MatrixXd::Identity(6, 6);
    W.diagonal() = w;

    MatrixXd e;
    int rejcount = 0;
    int rlimit = 400;

    while(true)
    {
        iter++;
        if(iter > maxiter)
        {
            break;
        }
        MatrixXd T1 = robot.fr_base0_X_ee(q);
        e = tr2delta(T1, T);

        if((W*e).norm() < tol)
        {
            break;
        }
        double et = (W*e).norm();
        double et1 = e.bottomLeftCorner(3,1).norm();
        double et2 = e.topLeftCorner(3,1).norm();

        // 几何雅可比
        MatrixXd Jg = robjocob.fr_base0_J_ee(q);

        MatrixXd G2T = MatrixXd::Zero(6, 6);
        G2T.topLeftCorner(3, 3) = T1.block(0, 0, 3, 3);
        G2T.bottomRightCorner(3, 3) = T1.block(0, 0, 3, 3);
        // 任务雅可比
        MatrixXd J = G2T * Jg;
        // J.block(3, 0, 3, 8) *= 10;

        // J.block(3, 0, 3, 8) *= 100;
        MatrixXd JtJ = J.transpose() * W * J;

        double *Jd = J.data();

        // do the damped inverse Gauss-Newton with Levenberg-Marquadt

        MatrixXd dq = (JtJ + (lambda + lambdamin) * MatrixXd::Identity(NUM_JONT, NUM_JONT)).inverse() * J.transpose() * W * e;
        // if(dq(6) > 0.01)
        // {
        //     dq(6) = 0.01;
        // }
        q_new = q + dq;

        if(q_new(6) < 0)
        {
            q_new(6) = 0;
        }

        if(dof>=6 && dof<7)
        {
            if(q_new(6) > 0.1)
            {
                q_new(6) = 0.1;
            }

            if(q_new(7) > 0.1)
            {
                q_new(7) = 0.1;
            }
        }
        else if(dof>=7 && dof<8)
        {
            if(q_new(7) > 0.1)
            {
                q_new(7) = 0.1;
            }
        }

        MatrixXd enew = tr2delta(robot.fr_base0_X_ee(q_new), T);
        double ent = (W*enew).norm();

        if((W*enew).norm() < (W*e).norm())
        {
            q = q_new;
            e = enew;
            lambda /= 2;
            rejcount = 0;
        }
        else
        {
            lambda *= 2;
            rejcount += 1;
            if(rejcount > rlimit)
            {
                cout << endl;
                cout << "ikine: rejected-step limit " << rlimit << " exceeded." << endl;
                break;
            }
            continue;
        }
    }

    cout << "ikine: final error:" << (W*e).norm() << endl;
    qt = q;

    return 0;

}

// 无碰撞避险
int ikine_link6_end2(const MatrixXd& T, const MatrixXd& q0, MatrixXd& qt, double tol, int maxiter)
{
    int NUM_JONT = static_cast<int>(q0.rows());

    HomogeneousTransforms robot;
    Jacobians robjocob;

    double lambda = 10;
    double lambdamin = 0;

    int iter = 0;

    MatrixXd q = q0;
    MatrixXd q_new = q0;
    // 每个自由度的权重系数
    MatrixXd w(6, 1);
    w << 1, 1, 1, 1, 1, 1;
    MatrixXd W = MatrixXd::Identity(6, 6);
    W.diagonal() = w;

    MatrixXd e;
    int rejcount = 0;
    int rlimit = 400;

    while(true)
    {
        iter++;
        if(iter > maxiter)
        {
            break;
        }
        MatrixXd T1 = robot.fr_base0_X_link6_end2(q);
        e = tr2delta(T1, T);

        if((W*e).norm() < tol)
        {
            break;
        }
        double et = (W*e).norm();
        double et1 = e.bottomLeftCorner(3,1).norm();
        double et2 = e.topLeftCorner(3,1).norm();

        // 几何雅可比
        MatrixXd Jg = robjocob.fr_base0_J_link6_end2(q);

        MatrixXd G2T = MatrixXd::Zero(6, 6);
        G2T.topLeftCorner(3, 3) = T1.block(0, 0, 3, 3);
        G2T.bottomRightCorner(3, 3) = T1.block(0, 0, 3, 3);
        // 任务雅可比
        MatrixXd J = G2T * Jg;
        // J.block(3, 0, 3, 8) *= 10;

        // J.block(3, 0, 3, 8) *= 100;
        MatrixXd JtJ = J.transpose() * W * J;

        double *Jd = J.data();

        // do the damped inverse Gauss-Newton with Levenberg-Marquadt

        MatrixXd dq = (JtJ + (lambda + lambdamin) * MatrixXd::Identity(6, 6)).inverse() * J.transpose() * W * e;


        q_new << q.topLeftCorner(6, 1) + dq, 0, 0;

        MatrixXd enew = tr2delta(robot.fr_base0_X_link6_end2(q_new), T);
        double ent = (W*enew).norm();

        if((W*enew).norm() < (W*e).norm())
        {
            q = q_new;
            e = enew;
            lambda /= 2;
            rejcount = 0;
        }
        else
        {
            lambda *= 2;
            rejcount += 1;
            if(rejcount > rlimit)
            {
                cout << endl;
                cout << "ikine: rejected-step limit " << rlimit << " exceeded." << endl;
                break;
            }
            continue;
        }
    }

    cout << "ikine: final error:" << (W*e).norm() << endl;
    qt = q;

    return 0;

}

// 无碰撞避险
int ikine_e8(const MatrixXd& T, const MatrixXd& q0, MatrixXd& qt, double tol, int maxiter)
{
    int NUM_JONT = static_cast<int>(q0.rows());

    HomogeneousTransforms robot;
    Jacobians robjocob;

    double lambda = 10;
    double lambdamin = 0;

    int iter = 0;

    MatrixXd q = q0;
    MatrixXd q_new = q0;
    // 每个自由度的权重系数
    MatrixXd w(6, 1);
    w << 1, 1, 1, 1, 1, 1;
    MatrixXd W = MatrixXd::Identity(6, 6);
    W.diagonal() = w;

    MatrixXd e;
    int rejcount = 0;
    int rlimit = 400;

    while(true)
    {
        iter++;
        if(iter > maxiter)
        {
            break;
        }
        MatrixXd T1 = robot.fr_base0_X_e8(q);
        e = tr2delta(T1, T);

        if((W*e).norm() < tol)
        {
            break;
        }
        double et = (W*e).norm();
        double et1 = e.bottomLeftCorner(3,1).norm();
        double et2 = e.topLeftCorner(3,1).norm();

        // 几何雅可比
        MatrixXd Jg = robjocob.fr_base0_J_e8(q);

        MatrixXd G2T = MatrixXd::Zero(6, 6);
        G2T.topLeftCorner(3, 3) = T1.block(0, 0, 3, 3);
        G2T.bottomRightCorner(3, 3) = T1.block(0, 0, 3, 3);
        // 任务雅可比
        MatrixXd J = G2T * Jg;
        // J.block(3, 0, 3, 8) *= 10;

        // J.block(3, 0, 3, 8) *= 100;
        MatrixXd JtJ = J.transpose() * W * J;

        double *Jd = J.data();

        // do the damped inverse Gauss-Newton with Levenberg-Marquadt

        MatrixXd dq = (JtJ + (lambda + lambdamin) * MatrixXd::Identity(NUM_JONT, NUM_JONT)).inverse() * J.transpose() * W * e;

        q_new = q + dq;

        if(q_new(6) > 5)
        {
            q_new(6) = 5;
        }
        if(q_new(6) < 0)
        {
            q_new(6) = 0;
        }

        MatrixXd enew = tr2delta(robot.fr_base0_X_e8(q_new), T);
        double ent = (W*enew).norm();

        if((W*enew).norm() < (W*e).norm())
        {
            q = q_new;
            e = enew;
            lambda /= 2;
            rejcount = 0;
        }
        else
        {
            lambda *= 2;
            rejcount += 1;
            if(rejcount > rlimit)
            {
                cout << endl;
                cout << "ikine: rejected-step limit " << rlimit << " exceeded." << endl;
                break;
            }
            continue;
        }
    }

    cout << "ikine: final error:" << (W*e).norm() << endl;
    qt = q;

    return 0;

}

// // link2、link4 碰撞避险
// int ikine(const MatrixXd& T, const MatrixXd& q0, MatrixXd& qt, double tol, int maxiter)
// {
//     int NUM_JONT = static_cast<int>(q0.rows());

//     HomogeneousTransforms robot;
//     Jacobians robjacob;

//     double lambda = 10;
//     double lambdamin = 0;

//     int iter = 0;

//     MatrixXd q = q0;
//     MatrixXd q_new = q0;
//     // 每个自由度的权重系数
//     MatrixXd w(6, 1);
//     w << 1, 1, 1, 1, 1, 1;
//     MatrixXd W = MatrixXd::Identity(6, 6);
//     W.diagonal() = w;

//     MatrixXd e;
//     int rejcount = 0;
//     int rlimit = 400;

//     while(true)
//     {
//         iter++;
//         if(iter > maxiter)
//         {
//             break;
//         }
//         MatrixXd T1 = robot.fr_base0_X_ee(q);
//         e = tr2delta(T1, T);

//         if((W*e).norm() < tol)
//         {
//             break;
//         }
//         double et = (W*e).norm();
//         double et1 = e.bottomLeftCorner(3,1).norm();
//         double et2 = e.topLeftCorner(3,1).norm();

//         // 几何雅可比
//         MatrixXd Jg = robjacob.fr_base0_J_ee(q);

//         MatrixXd G2T = MatrixXd::Zero(6, 6);
//         G2T.topLeftCorner(3, 3) = T1.block(0, 0, 3, 3);
//         G2T.bottomRightCorner(3, 3) = T1.block(0, 0, 3, 3);
//         // 任务雅可比
//         MatrixXd J = G2T * Jg;
//         // J.block(3, 0, 3, 8) *= 10;

//         // link2避障
//         MatrixXd jacob_l2(6, 8);
//         jacob_l2 << robjacob.fr_base0_J_link2_end(q), MatrixXd::Zero(6, 6);
//         jacob_l2 = G2T * jacob_l2;
//         jacob_l2 = jacob_l2.block(3, 0, 3, 8);
//         MatrixXd T_link2_end = robot.fr_base0_X_link2_end(q);
//         MatrixXd T_link1_end = robot.fr_base0_X_link1_end(q);
//         Vector3d e2 = (T_link1_end.topRightCorner(3, 1) - T_link2_end.topRightCorner(3, 1)).normalized();
//         Vector3d s0;
//         // 测试
//         s0 << -0.1, -0.2, 0.35;
//         Vector3d s2 = T_link2_end.topRightCorner(3, 1);
//         double p2 = e2.dot(s0 - s2);
//         Vector3d sc2 = s2 + p2 * e2;
//         Vector3d u2 = (sc2 - s0).normalized();

//         MatrixXd Jc2 = -u2.transpose() * jacob_l2;

//         // 考虑几个连杆的避障
//         MatrixXd Wc = MatrixXd::Identity(1, 1);

//         // J.block(3, 0, 3, 8) *= 100;
//         MatrixXd JtJ = J.transpose() * W * J;

//         double *Jd = J.data();

//         // do the damped inverse Gauss-Newton with Levenberg-Marquadt

//         MatrixXd dq = (JtJ + Jc2.transpose()*Wc*Jc2  + (lambda + lambdamin) * MatrixXd::Identity(NUM_JONT, NUM_JONT)).inverse() * J.transpose() * W * e;
//         // if(dq(6) > 0.01)
//         // {
//         //     dq(6) = 0.01;
//         // }
//         q_new = q + dq;
//         if(q_new(6) > 0.1)
//         {
//             q_new(6) = 0.1;
//         }
        
//         MatrixXd enew = tr2delta(robot.fr_base0_X_ee(q_new), T);
//         double ent = (W*enew).norm();

//         if((W*enew).norm() < (W*e).norm())
//         {
//             q = q_new;
//             e = enew;
//             lambda /= 2;
//             rejcount = 0;
//         }
//         else
//         {
//             lambda *= 2;
//             rejcount += 1;
//             if(rejcount > rlimit)
//             {
//                 cout << endl;
//                 cout << "ikine: rejected-step limit " << rlimit << " exceeded." << endl;
//                 break;
//             }
//             continue;
//         }
//     }

//     cout << "ikine: final error:" << (W*e).norm() << endl;
//     qt = q;

//     return 0;

// }
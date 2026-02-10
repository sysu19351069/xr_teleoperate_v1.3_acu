#include "accurate_positioning.h"
#include <unistd.h>

int NUM_JONT = 8;
double T = 1;
double dt = 0.1;
int NUM_CTRL = round(T/dt);

// 8-dof 关节角值
MatrixXd qk = MatrixXd::Zero(8, 1);


int accurate_positioning()
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
    // MatrixXd qk(NUM_JONT, 1);
    // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 0.0180033, 0.2709;
    // qk << 1.1562, 0.2386, -1.4676, -0.5988, -1.2485, 0.2709, 18.0033, 0.2709;

    // qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 0.018, 0;
    // qk << 0.7540, 0.7540, -0.5027, 0, -1.8850, 0, 18.00, 0;

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

    // // 总约束矩阵 G（标准式中的A）
    MatrixXd G = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 2 * NUM_JONT, NUM_JONT * (NUM_CTRL + 1));

    // upper bound ubA, lower bound lbA
    MatrixXd lbA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 2 * NUM_JONT, 1);
    MatrixXd ubA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 2 * NUM_JONT, 1);

    // MatrixXd G = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 0, NUM_JONT * (NUM_CTRL + 1));

    // // upper bound ubA, lower bound lbA
    // MatrixXd lbA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 0, 1);
    // MatrixXd ubA = MatrixXd::Zero(3 * NUM_JONT * NUM_CTRL + 0 + 0, 1);

    // 规划示例
    Vector3d v;
    v << 0.1, 0.1, 2;
    Vector3d z;
    z << 0, 0, 1;

    Vector3d vn = z.cross(v);  // 叉积操作（可能有问题）
    double angle = acos(v.dot(z)/(v.norm() * z.norm()));
    AngleAxisd vn_rotate(angle, vn.normalized());

    Matrix4d trans;
    trans.setIdentity();
    trans.block(0, 0, 3, 3) = vn_rotate.toRotationMatrix();

    AcuRobot::rcg::HomogeneousTransforms acubot;
    
    // 读取关节角的工具变量
    double Base[6] = { 0 };
    double Joint[6] = { 0 };
    double Tcp[6] = { 0 };
    double Ucs[6] = { 0 };
    // 读取当前关节角
//     hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
//     qk << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], qk(6, 0), qk(7, 0);
// //    ros::spinOnce();
//     qk = qk * M_PI / 180;

    qk << 1.63482, 0.0199317, -1.00999, 0.0587652, -2.15094, 0.152961, 0, 0;

    Matrix4d curpos = acubot.fr_base0_X_ee(qk);
    cout << "curpos: " << endl << curpos << endl;    
    
    cout << "base0_j5: " << endl << acubot.fr_base0_X_fr_j5(qk) << endl;
    cout << "base0_link5: " << endl << acubot.fr_base0_X_fr_link5(qk) << endl;
    cout << "base0_j6: " << endl << acubot.fr_base0_X_fr_j6(qk) << endl;
    cout << "base0_j7: " << endl << acubot.fr_base0_X_fr_j7(qk) << endl;
    cout << "base0_j8: " << endl << acubot.fr_base0_X_fr_j8(qk) << endl;

//    double r = 0.05;  // 针灸长度
    double r = 50;  // 针灸长度
    Matrix4d tl;
    tl.setIdentity();
    tl.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);
    tl.topRightCorner(3, 1) = -v;

    Matrix4d f_T = curpos * tl;
    Vector3d dO2 = r * (f_T.topRightCorner(3, 1) - curpos.topRightCorner(3, 1)).normalized();

    Matrix4d t2;
    t2.setIdentity();
    t2.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);
    t2.topRightCorner(3, 1) << 0, 0, r;

    Matrix4d c_T = curpos * t2;
    Vector3d dO1 = curpos.topRightCorner(3, 1) - c_T.topRightCorner(3, 1);

    Vector3d d12 = dO2 - dO1;
    Vector3d FP = c_T.topRightCorner(3, 1) + dO2;

    MatrixXd obj = curpos * trans;
    obj.topRightCorner(3, 1) = FP;

    // 需要写一个求逆运动学的函数
    // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 0.0179988, -0.0124;
    // qkd << 0.7287, 0.8884, -0.2642, -0.0024, -1.9974, -0.0124, 17.9988, -0.0124;

    // qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 0.0180050, 0.2975;
    // qkd << 1.2030, 0.2139, -1.5076, -0.6106, -1.2546, 0.2975, 18.0050, 0.2975;

    AcuRobot::rcg::Jacobians acubot_jacob;

    MatrixXd qt = MatrixXd::Zero(NUM_JONT, 1);

    double *qtcur = curpos.data();
    double *qtobj = obj.data();
    int rv = ikine(obj, qk, qt, 1e-3, 1000);
    
    cout << "accurate_positioning: " << endl << qt * 180 / M_PI << endl;
    cout << "accurate_positioning: " << endl << qt << endl;

    double *qtd = qt.data();

    MatrixXd qkd(NUM_JONT, 1);
    qkd = qt;

    Matrix4d testpose = acubot.fr_base0_X_ee(qt);

    int nV = NUM_JONT * (NUM_CTRL + 1);
    int nC = 3 * NUM_JONT * NUM_CTRL + 0 + 2 * NUM_JONT;
    // int nC = 3 * NUM_JONT * NUM_CTRL + 0 + 0;
//    qpOASES::SQProblem mpc(nV, nC);
//    int nWSR = 5*(nV + nC);

    MatrixXd qk_1 = qk;

    MatrixXd Gq_dot = MatrixXd::Zero(NUM_JONT * NUM_CTRL, NUM_JONT * (NUM_CTRL + 1));
    for(int i = 0; i < NUM_CTRL; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            Gq_dot.block(NUM_JONT * i, NUM_JONT * j, NUM_JONT, NUM_JONT) = MatrixXd::Identity(NUM_JONT, NUM_JONT);
        }
    }

    // 控制关节角的工具变量
    double TCP[6] = { 0, 0, 0, 0, 0, 0};
    string TcpName = "TCP";
    string UcsName = "Base";
    double Vel = 30;
    double Acc = 30;
    double dr = 1;
    int IsUseJoint = 1;
    int Iseek = 0;
    int IOBit = 0;
    int IOState = 0;
    string ID = "0";

    int itermax = 3;
    int returnvalue = 0;
    
//    qkd = qkd * 180 / M_PI;
//    hb.HR_MoveJ(0, 0, qkd.data(), TCP, TcpName, UcsName, Vel, Acc, dr, IsUseJoint, Iseek, IOBit, IOState, ID);
//    return 0;
    
    for(int i = 0; i < itermax; i++)
    {
        cout << endl << endl << endl;
//        cout << "qk: " << endl << qk * 180 / M_PI << endl;
        cout << "current pos: "<< endl << curpos << endl;
        Vector3d OP = curpos.topRightCorner(3, 1) - c_T.topRightCorner(3, 1);
        Vector3d OPr = r * OP.normalized();
        Vector3d rotate_axis = OPr.cross(dO2);
        double anglesum = acos(OPr.dot(dO2)/(r*r));
        AngleAxisd h((anglesum/NUM_CTRL) * min(i+1, NUM_CTRL), rotate_axis.normalized());

        
        Vector3d OM = h.toRotationMatrix() * OPr;
        Matrix3d rtm = h.toRotationMatrix();
        double b1 = OM(0, 0);
        double b2 = OM(1, 0);
        double b3 = OM(2, 0);

        Vector3d Sd = OM - OP;

        double a1 = Sd(0, 0);
        double a2 = Sd(1, 0);
        double a3 = Sd(2, 0);

        Vector3d Vv = OP + 0.5 * Sd;
        double dotValue = Vv.dot(Sd);
        
        cout << "Sd == " << endl << Sd << endl;

        MatrixXd jacobk_t = acubot_jacob.fr_base0_J_ee(qk).block(3, 0, 3, 8);
        int tr = mpc2qp(NUM_CTRL, NUM_JONT, dt,
                Q, R, T, A, B, Av, Bv,
                jacobk_t, qk, qkd, qk_1,
                uk_1, Sd, Vv, dotValue,
                theta, omega, psi, Theta,
                Psi, Theta_plus, Epsilon,
                Delta, H, g, G, lbA, ubA);
                
        qpOASES::QProblem mpc(nV, nC);
        int nWSR = 5*(nV + nC);
        
        qpOASES::Options myOptions;
        myOptions.setToMPC( );
        myOptions.printLevel = qpOASES::PL_LOW;
        mpc.setOptions( myOptions );
        
//        double *hdata = H.data();
//        double *gdata = g.data();
//        double *Gdata = G.data();
//        double *lbAdata = lbA.data();
//        double *ubAdata = ubA.data();

        MatrixXd Ht = H.transpose();
        MatrixXd gt = g.transpose();
        MatrixXd Gt = G.transpose();

        returnvalue = mpc.init(Ht.data(), gt.data(), Gt.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
        // returnvalue = mpc.init(H.transpose().data(), g.transpose().data(), G.transpose().data(), 0, 0, lbA.transpose().data(), ubA.transpose().data(), nWSR, 0);
//        returnvalue = mpc.init(hdata, gdata, Gdata, 0, 0, lbAdata, ubAdata, nWSR, 0);

//        if(i == 0)
//        {
//            returnvalue = mpc.init(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
//        }
//        else
//        {
//            returnvalue = mpc.hotstart(H.data(), g.data(), G.data(), 0, 0, lbA.data(), ubA.data(), nWSR, 0);
//        }
        cout << mpc.getObjVal() << endl;
        MatrixXd xOpt(nV, 1);
        mpc.getPrimalSolution(xOpt.data());

//        MatrixXd xOpt(nV, 1);
//        double* data = new double[nV];

//        int result = mpc2qp1(NUM_CTRL, NUM_JONT, dt,
//             qk, 
//             qkd,
//             qk_1,
//             uk_1,
//             Sd,
//             data);
//        for(int l=0; l < nV; l++)
//        {
//            xOpt(l, 0) = data[l];
//        }
        
//        cout << "xOpt: " << endl << xOpt << endl;
        MatrixXd qc = psi * qk + omega * uk_1 + theta * xOpt;
//        cout << "qc: " << endl << qc << endl;
        
        MatrixXd Uk_1 = MatrixXd::Zero(NUM_JONT * NUM_CTRL, 1);
        for(int j = 0; j < NUM_CTRL; j++)
        {
            Uk_1.block(NUM_JONT * j, 0, NUM_JONT, 1) = uk_1;
        }
        MatrixXd U = Uk_1 + Gq_dot * xOpt;

        MatrixXd q_next = qc.topLeftCorner(8, 1);
        MatrixXd u_next = U.topLeftCorner(8, 1);
        
        cout << "u_next: " << u_next << endl;
        
        MatrixXd vttt = dt * jacobk_t * u_next;
        
        cout << "vttt == " << endl << vttt << endl;

        // 发送给机械臂的控制量
        MatrixXd command_arm = q_next.topLeftCorner(6, 1);
        command_arm = command_arm * 180 / M_PI;
        // 发送给电机的控制量
//        std_msgs::Float32MultiArray command_motor;
//        command_motor.data.push_back(u_next(6, 0)); 
//        command_motor.data.push_back(u_next(7, 0));
//        pub.publish(command_motor);

        // 采样频率
        sleep(1);

        // 更新关节角
        qk_1 = qk;
//        hb.HR_ReadActPos(Base, Joint, Tcp, Ucs);
//        qk << Joint[0], Joint[1], Joint[2], Joint[3], Joint[4], Joint[5], qk(6, 0), qk(7, 0);
//        ros::spinOnce();
        
        
        qk = q_next;

        cout << "q_next" << q_next * 180 / M_PI << endl;
        // 更新uk_1
        uk_1 = u_next;

        // 更新位姿
        curpos = acubot.fr_base0_X_ee(qk);
        
//        delete[] data;
    }

    return 0;
}

#include "Admittance.h"
#include <ctime>
#include <random>

Admittance::Admittance(ros::NodeHandle &n, double frequency,
                       std::vector<double> M,
                       std::vector<double> D,
                       std::vector<double> K,
                       std::vector<double> desired_pose,
                       double arm_max_vel,
                       double arm_max_acc,
                       string cportName,
                       string fportName) : nh_(n), loop_rate_(frequency), M_(M.data()),
                                           D_(D.data()), K_(K.data()), desired_position_(desired_pose.data()),
                                           arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc)
{

  pub_vel_ = nh.advertise<std_msgs::Float32MultiArray>("/MotorWrite", 1);

  // initializing the class variables
  force_external_.setZero();
  force_init_.setZero();

  last_F_env.setZero();
  X_last_.setZero();  // t - T
  Xd_last_.setZero(); // t - T * 2

  if (fport.OpenPort(fportName.c_str()))
  {
    cout << "打开成功" << endl;
    int f = fport.setup(1500000, 0, 8, 1, 'E');
    ROS_INFO("%d", f);
  }
  else
  {
    cout << "打开失败" << endl;
    // ros::shutdown();
  }

  // Init integrator
  acupuncture_desired_vel_adm_.setZero();
  acupuncture_desired_accelaration.setZero();

  // unsigned char cmdreset[10] = {0xF6, 0x6F, 0x03, 0x00, 0x00, 0x0B, 0XF7, 0X7D, 0X6F, 0XF6};

  // fport.writeBuffer(cmdreset, 10);

  sleep(1);

  // cal_deltaf();
}

void Admittance::cal_deltaf()
{
  double sum1, sum2 = 0;

  double ot1 = force_init_(2);
  double ot2 = force_init_(5);
  for (int i = 0; i < 10; i++)
  {

    perceive_force_init();
    Vector2d F_env;

    double not1 = force_init_(2);
    double not2 = force_init_(5);

    if (abs(ot1 - not1) > 5)
    {
      i--;
      continue;
    }
    if (abs(ot2 - not2) > 1)
    {
      i--;
      continue;
    }

    sum1 += not1;
    sum2 += not2;

    ot1 = not1;
    ot2 = not2;

    loop_rate_.sleep();
  }

  // get average
  deltaf1 = sum1 / 10;
  deltaf2 = sum2 / 10;

  ROS_INFO("deltaf1: %.2f, deltaf2: %.2f", deltaf1, deltaf2);
}

//!-                    CONTROL LOOP                     -!//

void Admittance::run()
{

  //   ROS_INFO("Running the admittance control loop .................");
  // define stiffness
  // for(int i = 0; i < 3; i++){
  //     Ke_(i,i) = 400;
  // }
  while ((desired_position_ - X_last_).norm() > 0.0005)
  {
    ros::Duration duration = loop_rate_.expectedCycleTime();

    // calculate_Xe();

    compute_admittance();

    send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
    // sleep(1);
  }
}

void Admittance::perceive_force_init()
{

  // unsigned char cmdreset[10] = {0xF6, 0x6F, 0x03, 0x00, 0x00, 0x0B, 0XF7, 0X7D, 0X6F, 0XF6};

  // fport.writeBuffer(cmdreset, 10);

  // 单次测量的数据帧
  unsigned char cmdFrame[10] = {0xF6, 0x6F, 0x03, 0x00, 0x00, 0x04, 0x18, 0x8C, 0x6F, 0xF6};

  // ROS_INFO("i am here %x", cmdFrame);
  int a = fport.writeBuffer(cmdFrame, 10);

  // ROS_INFO("i am here 2 %d", a);

  // 返回的数据帧
  unsigned char fdat[34];
  int b = fport.readBuffer(fdat, 34);

  // while(b != 34)
  // {
  //   b = fport.readBuffer(fdat, 34);
  // }

  // ROS_INFO("i am here3 %d %s %x", b, fdat, fdat);
  unsigned char fx[4], fy[4], fz[4], mx[4], my[4], mz[4];

  // 提取其中的六维数据
  fx[0] = fdat[6];
  fx[1] = fdat[7];
  fx[2] = fdat[8];
  fx[3] = fdat[9];
  fy[0] = fdat[10];
  fy[1] = fdat[11];
  fy[2] = fdat[12];
  fy[3] = fdat[13];
  fz[0] = fdat[14];
  fz[1] = fdat[15];
  fz[2] = fdat[16];
  fz[3] = fdat[17];
  mx[0] = fdat[18];
  mx[1] = fdat[19];
  mx[2] = fdat[20];
  mx[3] = fdat[21];
  my[0] = fdat[22];
  my[1] = fdat[23];
  my[2] = fdat[24];
  my[3] = fdat[25];
  mz[0] = fdat[26];
  mz[1] = fdat[27];
  mz[2] = fdat[28];
  mz[3] = fdat[29];

  int fx_, fy_, fz_, mx_, my_, mz_ = 0;
  memcpy(&fx_, fx, 4);
  memcpy(&fy_, fy, 4);
  memcpy(&fz_, fz, 4);
  memcpy(&mx_, mx, 4);
  memcpy(&my_, my, 4);
  memcpy(&mz_, mz, 4);

  // ROS_INFO("fx: %d, fy: %d, fz: %d, mx: %d, my: %d, mz: %d....",
  // fx_, fy_, fz_, mx_, my_, mz_);

  force_init_ << fx_, fy_, fz_, mx_, my_, mz_;
  force_init_ /= 1000;

  // ROS_INFO("fx: %0.3f, fy: %0.3f, fz: %0.3f, mx: %0.3f, my: %0.3f, mz: %0.3f....",
  //                                                   force_init_(0), force_init_(1), force_init_(2), force_init_(3), force_init_(4), force_init_(5));
}

void Admittance::perceive_force()
{

  // unsigned char cmdreset[10] = {0xF6, 0x6F, 0x03, 0x00, 0x00, 0x0B, 0XF7, 0X7D, 0X6F, 0XF6};

  // fport.writeBuffer(cmdreset, 10);

  // 单次测量的数据帧
  unsigned char cmdFrame[10] = {0xF6, 0x6F, 0x03, 0x00, 0x00, 0x04, 0x18, 0x8C, 0x6F, 0xF6};

  // ROS_INFO("i am here %x", cmdFrame);
  int a = fport.writeBuffer(cmdFrame, 10);

  // ROS_INFO("i am here 2 %d", a);

  // 返回的数据帧
  unsigned char fdat[34];
  int b = fport.readBuffer(fdat, 34);

  // while(b != 34)
  // {
  //   b = fport.readBuffer(fdat, 34);
  // }

  // ROS_INFO("i am here3 %d %s %x", b, fdat, fdat);
  unsigned char fx[4], fy[4], fz[4], mx[4], my[4], mz[4];

  // 提取其中的六维数据
  fx[0] = fdat[6];
  fx[1] = fdat[7];
  fx[2] = fdat[8];
  fx[3] = fdat[9];
  fy[0] = fdat[10];
  fy[1] = fdat[11];
  fy[2] = fdat[12];
  fy[3] = fdat[13];
  fz[0] = fdat[14];
  fz[1] = fdat[15];
  fz[2] = fdat[16];
  fz[3] = fdat[17];
  mx[0] = fdat[18];
  mx[1] = fdat[19];
  mx[2] = fdat[20];
  mx[3] = fdat[21];
  my[0] = fdat[22];
  my[1] = fdat[23];
  my[2] = fdat[24];
  my[3] = fdat[25];
  mz[0] = fdat[26];
  mz[1] = fdat[27];
  mz[2] = fdat[28];
  mz[3] = fdat[29];

  int fx_, fy_, fz_, mx_, my_, mz_ = 0;
  memcpy(&fx_, fx, 4);
  memcpy(&fy_, fy, 4);
  memcpy(&fz_, fz, 4);
  memcpy(&mx_, mx, 4);
  memcpy(&my_, my, 4);
  memcpy(&mz_, mz, 4);

  // ROS_INFO("fx: %d, fy: %d, fz: %d, mx: %d, my: %d, mz: %d....",
  // fx_, fy_, fz_, mx_, my_, mz_);

  force_external_ << fx_, fy_, fz_, mx_, my_, mz_;
  force_external_ /= 1000;

  ROS_INFO("fx: %0.3f, fy: %0.3f, fz: %0.3f, mx: %0.3f, my: %0.3f, mz: %0.3f....",
           force_external_(0), force_external_(1), force_external_(2), force_external_(3), force_external_(4), force_external_(5));
}

//!-                Admittance Dynamics                  -!//
void Admittance::compute_admittance()
{

  // while(true)
  // {
  //   perceive_force();
  //   sleep(2);
  //   // loop_rate_.sleep();
  // }

  double ot1 = force_external_(2);
  double ot2 = force_external_(5);
  // perceive_force();

  Vector2d F_env;

  if (abs(ot1 - force_external_(2)) > 10)
  {
    force_external_(2) = ot1;
  }
  if (abs(ot2 - force_external_(5)) > 10)
  {
    force_external_(5) = ot2;
  }

  double tt = force_external_(2);
  double tt2 = force_external_(5);

  F_env << (((tt - deltaf1) < -10) ? -10 : (tt - deltaf1)), (((tt2 - deltaf2) < -10) ? -10 : (tt2 - deltaf2));

  // ROS_INFO("deltaf1: %.2f, f1: %.2f deltaf2: %.2f, m1:%.2f", deltaf1, F_env(0), deltaf2, F_env(1));

  F_env << 0, 0;
  Vector2d coupling_wrench_acup;
  // 针刺速度，需要建模
  Vector2d desired_velocity_;
  desired_velocity_.setZero(); // 暂时设为0

  // 内环速度远大于外环速度
  coupling_wrench_acup = D_ * (Xd_last_) + K_ * (X_last_ - desired_position_);
  acupuncture_desired_accelaration = M_.inverse() * (-coupling_wrench_acup + F_env);

  ros::Duration duration = loop_rate_.expectedCycleTime();
  acupuncture_desired_vel_adm_ += acupuncture_desired_accelaration * duration.toSec();

  // Xd_last_[0] += acupuncture_desired_vel_adm_ * duration.toSec();
  // Xd_last_[1]  = acupuncture_desired_vel_adm_;

  X_last_ += acupuncture_desired_vel_adm_ * duration.toSec();
  Xd_last_ = acupuncture_desired_vel_adm_;
}

//!-               COMMANDING THE ROBOT                  -!//
void Admittance::send_commands_to_robot()
{
  // 电机1，mm/s
  double x = acupuncture_desired_vel_adm_(0) * 1000;
  // 电机2，°/s
  double y = acupuncture_desired_vel_adm_(1) * 180 / PI;

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(x);
  msg.data.push_back(y);
  pub_vel_.publish(msg);
  ROS_INFO("x: %.3f mm/s, y: %.3f °/s", x, y);

  double ba = X_last_(0);
  double bb = X_last_(1);
  ROS_INFO("s1: %f, s2: %f\n", ba, bb);

  // inverse purely
  if ((abs(abs(ba) - 0.05) < 0.001) && flag)
  {
    ROS_INFO("i am here lift!");
    desired_position_(0) = -0.02;
    desired_position_(1) = 0;
    X_last_.setZero();  // t - T
    Xd_last_.setZero(); // t - T * 2
    flag = 0;
    usleep(100000);
  }

  if ((abs(abs(ba) - 0.02) < 0.001) && flag1_5 && (!flag))
  {
    ROS_INFO("i am here plug!");
    desired_position_(0) = 0.02;
    desired_position_(1) = 0;
    X_last_.setZero();  // t - T
    Xd_last_.setZero(); // t - T * 2
    ba = 0;
    flag1_5 = 0;
    usleep(100000);
  }

  // with special skill
  //   translate a while then positive rotate
  if ((abs(abs(ba) - 0.02) < 0.001) && flag1 && (!flag1_5))
  {
    ROS_INFO("i am here positive rotate!");
    desired_position_(0) = 0;
    desired_position_(1) = 2;
    X_last_.setZero();  // t - T
    Xd_last_.setZero(); // t - T * 2
    bb = 0;
    flag1 = 0;
    usleep(100000);
  }

  // negetive rotate
  if ((abs(abs(bb) - 2) < 0.005) && flag2 && (!flag1))
  {
    ROS_INFO("i am here negetive rotate!");
    desired_position_(0) = 0;
    desired_position_(1) = -2;
    bb = X_last_(1);
    X_last_.setZero();  // t - T
    Xd_last_.setZero(); // t - T * 2
    bb = 0;
    flag2 = 0;
    usleep(100000);
  }

  //   if((abs(abs(bb) - 3.14) < 0.005) && (!flag2))
  //   {
  //     flag3 = 0;
  //   }

  // reverse-translate
  if ((abs(abs(bb) - 2) < 0.005) && flag3 && (!flag2))
  {
    ROS_INFO("i am here over!");
    desired_position_(0) = -0.05;
    desired_position_(1) = 0;
    X_last_.setZero();  // t - T
    Xd_last_.setZero(); // t - T * 2
    flag3 = 0;
    sleep(1);
  }
}

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <unistd.h>

#include "mkIdnty.h"

using namespace std;
using namespace cv;

int picwidth = 1280, picheight = 720;

Mat dots(Mat img)
{
    Mat hsv, mask;

    // 转换为hsv颜色空间
	cvtColor(img, hsv, CV_BGR2HSV);
	inRange(hsv, Scalar(0, 0, 120), Scalar(180, 255, 255), mask);

    // 滤波
	Mat medianblur;
	medianBlur(mask, medianblur, 7); //中值滤波

    // 获取轮廓，圆
	vector<vector<Point> > contours;
	findContours(medianblur, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  //找出所有轮廓

    const char *bgImage = "/home/ros/AcuRobot/src/perception_and_control/background_1280_720.jpg"; // 背景图
	Mat EnclosingCircle = imread(bgImage, 0);
	float radius;
	Point2f center;

    for (size_t i = 0; i<contours.size(); i++)
	{
		// 计算轮廓的最小外接圆
		minEnclosingCircle(contours[i], center, radius);
		// 过滤掉半径太小的外接圆
		if (MIN_CIRCLE_RADIUS > radius || radius > MAX_CIRCLE_RADIUS || center.x < 20 || center.x > 900 || center.y < 120 || center.y > 600)
		{
			continue;
		}
		// 画出最小外接圆,并涂红色实心
		circle(EnclosingCircle, center, radius, Scalar(0, 0, 255), -2);
	}

    return EnclosingCircle;
}

int ellipse(Mat circleImg, double EllipseData[][6])
{
    CvMemStorage* storage = cvCreateMemStorage(0);   //临时存储区
	CvSeq* contour;                                 //用于存储指向第一个外接轮廓
	contour = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);
	IplImage* img_Origin = cvCreateImage(cvSize(picwidth, picheight), 8, 3);  //定义输入图片
	IplImage* img_Smooth = cvCreateImage(cvSize(picwidth, picheight), 8, 1);
	IplImage* img_Canny = cvCreateImage(cvSize(picwidth, picheight), 8, 1);
	IplImage* color_dst = cvCreateImage(cvSize(picwidth, picheight), IPL_DEPTH_8U, 3);
	CvPoint* PointArray;
	CvPoint2D32f* PointArray2D32f;

    *img_Origin = cvIplImage(circleImg);

    cvSmooth(img_Origin, img_Smooth, CV_MEDIAN, 3, 3, 0, 0);  //使用中值滤波
	//cvSaveImage("Smooth1.jpg", img_Smooth);
	cvCanny(img_Smooth, img_Canny, 30, 30, 3);  //cvCanny()函数只接受单通道图像作为输入
	cvCvtColor(img_Canny, color_dst, CV_GRAY2BGR);
	cvFindContours(img_Canny, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

    //绘制所有轮廓，并提取参数
	for (; contour; contour = contour->h_next)
	{
		//定义参数
		int i;
		int count = contour->total;   //轮廓总数
		CvPoint center;               //定义椭圆中心点	
		CvSize size;                  //定义椭圆长轴和短轴的尺寸
		// CvBox2D* box;              //定义包围椭圆轮廓的最小矩形
		if (count < 6)                //个数必须大于6，这是cvFitEllipse的要求
		continue;

		//拟合并绘制当前椭圆轮廓
		CvBox2D box = cvFitEllipse2(contour);
		// RotatedRect box2 = fitEllipse(PointArray);
		cvDrawContours(color_dst, contour, cvScalar(255, 255, 255), cvScalar(255, 255, 255), 0, 1, 8, cvPoint(0, 0));
		
		// 将椭圆数据从浮点转化为整数表示
		center.x = cvRound(box.center.x);
		center.y = cvRound(box.center.y);
		size.width = cvRound(box.size.width*0.5);
		size.height = cvRound(box.size.height*0.5);
		box.angle = -box.angle;

		//设定阈值,若符合则画出椭圆，并保存参数
		if ((size.width > 7) && (size.width < 15) && (size.height > 7) && (size.height < 15))
		{
			cvEllipse(color_dst, center, size, box.angle, 0, 360, cvScalar(255, 0, 0), 1, CV_AA, 0);
			EllipseData[g_num][0] = g_num + 1;
			EllipseData[g_num][1] = center.x;
			EllipseData[g_num][2] = center.y;
			// EllipseData[g_num][3] = size.width;
			// EllipseData[g_num][4] = size.height;
			EllipseData[g_num][3] = box.size.width*0.5;
			EllipseData[g_num][4] = box.size.height*0.5;
			EllipseData[g_num][3] = 9.5;
			EllipseData[g_num][4] = 9.5;
			/*if (num == 1)
			{
				EllipseData[g_num][3] = 8.5;
				EllipseData[g_num][4] = 8.5;
			}*/

			EllipseData[g_num][5] = 0;
			g_num = g_num + 1;
			
		}
	}

    return 0;
}

int process(double Ellipse[][6], double* CamParm, double Position[][4])
{
    double u0 = CamParm[0], v0 = CamParm[1], fu = CamParm[2], fv = CamParm[3];

    int p_num = 0;
    double xc = 0, yc = 0, a = 0, b = 0, theta = 0;      //椭圆的5个参数
	double A = 0, B = 0, C = 0, D = 0, E = 0, F = 0;     //椭圆方程的参数
	double Q[3][3] = { 0 };                              //系数矩阵Q
	double Target_Position[3] = { 0 };                   //目标的位置及法向量
	double PosX, PosY, PosZ;                             //正数状态下左目下的位置姿态变量

    //一个大的for循环，输出所有轮廓的三维位置
	for (int i = 0; i < 24; i++)
	{
		//利用结构体参数来做椭圆参数传输的载体
		
		xc = Ellipse[i][1];
		yc = Ellipse[i][2];
		a = Ellipse[i][3];
		b = Ellipse[i][4];
		theta = Ellipse[i][5];

		//-------------椭圆锥方程的五个系数及系数矩阵Q-----------------
		A = (a*a*sin(theta)*sin(theta) + b*b*cos(theta)*cos(theta))*fu*fu;
		B = (2 * (b*b - a*a)*sin(theta)*cos(theta))*fu*fv;
		C = (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*fv*fv;
		D = (-2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*yc)*fu
			+ 2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*fu*u0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*fu*v0;
		E = (-2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc)*fv
			+ 2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*fv*v0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*fv*u0;
		F = (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc*xc
			+ (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc*yc
			+ (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*u0*u0
			+ (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*v0*v0
			+ (2 * (b*b - a*a)*sin(theta)*cos(theta))*u0*v0
			+ (-2 * (b*b*cos(theta)*cos(theta) + a*a*sin(theta)*sin(theta))*xc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*yc)*u0
			+ (-2 * (a*a*cos(theta)*cos(theta) + b*b*sin(theta)*sin(theta))*yc
			- 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc)*v0
			+ 2 * (b*b - a*a)*sin(theta)*cos(theta)*xc*yc
			- a*a*b*b;
		Q[0][0] = A;        Q[0][1] = B / 2;    Q[0][2] = D / 2;
		Q[1][0] = B / 2;    Q[1][1] = C;        Q[1][2] = E / 2;
		Q[2][0] = D / 2;    Q[2][1] = E / 2;    Q[2][2] = F;

		//--------------------矩阵Q的特征值和特征向量---------------------
		double Lambda1, Lambda2, Lambda3; //特征值
		double L_e2[3][1] = { 0 }, L_e3[3][1] = { 0 };//特征向量e2 e3，数组形式
		Mat P, e1;//转换矩阵及向量e1	
		cv::Mat eValMatrixLQ;//矩阵Q的特征值
		cv::Mat eVecMatrixLQ;//矩阵Q的特征向量
		cv::Mat MatrixLQ = cv::Mat(3, 3, CV_64FC1, Q);//生成矩阵Q	
		cv::eigen(MatrixLQ, eValMatrixLQ, eVecMatrixLQ);//计算矩阵Q的特征值及特征向量
		
		Lambda3 = eValMatrixLQ.at<double>(2, 0); Lambda2 = eValMatrixLQ.at<double>(1, 0); Lambda1 = eValMatrixLQ.at<double>(0, 0);
		L_e3[0][0] = eVecMatrixLQ.at<double>(2, 0); L_e3[1][0] = eVecMatrixLQ.at<double>(2, 1); L_e3[2][0] = eVecMatrixLQ.at<double>(2, 2);
		L_e2[0][0] = eVecMatrixLQ.at<double>(1, 0); L_e2[1][0] = eVecMatrixLQ.at<double>(1, 1); L_e2[2][0] = eVecMatrixLQ.at<double>(1, 2);
	
		cv::Mat e2 = cv::Mat(3, 1, CV_64FC1, L_e2);//创建e2向量
		cv::Mat e3 = cv::Mat(3, 1, CV_64FC1, L_e3);//创建e3向量
		e1 = e2.cross(e3);//向量e2叉乘向量e3得出向量e1
		Lambda1 = abs(Lambda1); Lambda2 = abs(Lambda2); Lambda3 = abs(Lambda3);//将特征值绝对值化
		hconcat(e1, e2, P); hconcat(P, e3, P);//由e1 e2 e3生成转换矩阵P	
	
		//-------------------位置姿态计算-----------------------------------------
		Target_Position[0] = sqrt(Lambda3*(Lambda1 - Lambda2) / Lambda1 / (Lambda1 + Lambda3));//位置计算
		Target_Position[1] = 0;
		Target_Position[2] = sqrt(Lambda1*(Lambda2 + Lambda3) / Lambda3 / (Lambda1 + Lambda3));
		Mat Target_Position_tmp = Mat(3, 1, CV_64FC1, Target_Position);//目标位置	
		Target_Position_tmp = P*Target_Position_tmp;//左乘转换矩阵P生成实际的相机坐标系下的三维坐标	

		double Radius = 2;
		PosX = Radius*Target_Position[0];
		PosY = Radius*Target_Position[1];
		PosZ = Radius*Target_Position[2];
		cout << i+1 << "   " << PosX << "   " << PosY << "   " << PosZ << endl;

		Position[p_num][0] = p_num + 1;
		Position[p_num][1] = PosX;
		Position[p_num][2] = PosY;
		Position[p_num][3] = PosZ;

		if(PosZ < 0)
		{
			Position[p_num][1] = -PosX;
			Position[p_num][2] = -PosY;
		}

		p_num = p_num + 1;
	}

    return 0;
}

int markerIdentify(Mat img, double Position[][4])
{
    double EllipseData[3][6];

    double CamParm[4] = {668.9, 417.0, 1162.4, 1170.8};

    Mat circleImg = dots(img);  // 
    ellipse(circleImg, EllipseData);
    process(EllipseData, CamParm, Position);

    return 0;
}

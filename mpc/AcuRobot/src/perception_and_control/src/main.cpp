#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include "mkIdnty.h"
#include "Admittance.h"
//#include "Admittance.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    const char* fn = "/home/ros/AcuRobot/src/perception_and_control/test_img.jpg";
    Mat img = imread(fn, 0);
    //imwrite(fn, img);

    double position[3][4];
    int res = markerIdentify(img, position);

    if(!res)
    {
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                ROS_INFO("%.2f  ", position[i][j]);
                ROS_INFO("\n");
            }
        }
    }
}

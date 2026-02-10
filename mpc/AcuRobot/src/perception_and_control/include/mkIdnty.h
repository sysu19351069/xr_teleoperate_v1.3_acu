#ifndef __MKIDNTY_H
#define __MKIDNTY_H

using namespace cv;

int markerIdentify(Mat img, double position[][4]);

double MIN_CIRCLE_RADIUS = 5;
double MAX_CIRCLE_RADIUS = 10;

int g_num = 2;
#endif

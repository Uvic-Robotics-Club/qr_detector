#pragma once

#ifndef _core_h
#define _core_h

#include "opencv2\opencv.hpp"

using namespace cv;

struct Qr_point {
	int x = 0;
	int y = 0;
	int xCenterWidth = 0;
	int yCenterWidth = 0;
	double orientation = 0;
};

struct Qr_code {
	Qr_point p1;
	Qr_point p2;
	Qr_point p3;
	double orientation;	//Degrees clockwise from p2 in top left corner
	double distance;
};

void img_test();
Mat find_qr_center(Mat src, bool debug = false);
bool verify_y(Mat binary, int cy, int cx, double lowFactor, double highFactor, Qr_point * centerObj);
Mat binarize_image(Mat colImg);
Mat binarize_image_smrt(Mat colImg);

#endif // !_core_h

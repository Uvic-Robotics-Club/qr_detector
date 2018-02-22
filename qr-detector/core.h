#pragma once

#ifndef _core_h
#define _core_h

#include "opencv2\opencv.hpp"
using namespace cv;


struct Qr_point {
	int x = 0;				// pixel position 
	int y = 0;				// pixel position
	int xCenterWidth = 0;	// pixels
	int yCenterWidth = 0;	// pixels
};

struct Qr_code {
	Qr_point p1;
	Qr_point p2;
	Qr_point p3;
	double orientation;	//Degrees clockwise from p2 in top left corner
	double distance;
	bool calculated = false;	//whether p1, p2, and p3 calculations are "clean" enough to trust
};

void img_test();
Mat find_qr_center(Mat src, bool debug = false);
bool verify_y(Mat binary, int cy, int cx, double lowFactor, double highFactor, Qr_point * centerObj);
void block_finder(Qr_point qp, Mat img);

Qr_code join_points(Mat binary, Qr_point p1, Qr_point p2, Qr_point p3);
double angle_between_points(int x1, int x2, int y1, int y2);
double orient_point(Mat binary, Qr_point pq);

Mat binarize_image(Mat colImg);
Mat binarize_image_smrt(Mat colImg);

#endif // !_core_h

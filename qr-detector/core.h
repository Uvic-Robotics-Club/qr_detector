#pragma once

#ifndef _core_h
#define _core_h

#include "opencv2\opencv.hpp"

using namespace cv;

void img_test();
Mat find_qr_center(Mat src, bool debug = false);
bool verify_y(Mat binary, int cy, int cx, double lowFactor, double highFactor);
Mat binarize_image(Mat colImg);
Mat binarize_image_smrt(Mat colImg);

#endif // !_core_h
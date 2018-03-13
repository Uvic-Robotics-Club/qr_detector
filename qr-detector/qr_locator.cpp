#include "qr_locator.h"
#include <string>
#include <math.h>

int main() {
	// Set up the system default webcam (the '0' in the constructor)
	// Should be cross-platform
	VideoCapture cam(0);

	int counter = 0;
	while (true) {
		// Get the most recent image from the webcam
		Mat frame;
		cam.read(frame);
		while (frame.empty())
			cam.read(frame);

		Mat marked = find_qr_center(frame, true);
		//imwrite("C:/Workspace/" + std::to_string(counter) + std::string(".jpg"), frame);

		counter++;
	}
	return 0;
}


void callback(Mat frame) {
	Mat marked = find_qr_center(frame, false);
	//publish marked
}


// Run a single image from memory.
void img_test() {
	Mat src = imread("C:/Workspace/0.jpg");
	Mat qrd = find_qr_center(src, true);
}


// If debug, displays the matrices. Else returns the src matrix
// Not blocking UNLESS debug = true
// TODO: this method is too long. Extract state machine, and make an independent method
// This also means the verification (running in the y-direction instead of x) can be done using the same
// code with a simple image transpose
Mat find_qr_center(Mat src, bool debug) {
	Mat res = src.clone();
	Mat binary = binarize_image(src);
	Mat states = Mat::zeros(src.rows, src.cols, CV_8UC1);
	std::vector<Qr_point> centers;
	// QR pattern, starting on white: 1/1/1/3/1/1/1
	// As we scan the image, we will use a state machine for where we are in the QR seqeuence
	// Counts stores the number of pixels in each band
	int state;
	int counts[7];
	// determines how much of a size difference between bands is allowed
	double highFactor = 1.25;
	double lowFactor = 0.75;
	// TODO: currently runs every row. Once it is more reliable, could make this every few rows
	for (int y = 0; y < binary.rows; ++y) {
		uchar * row = binary.ptr<uchar>(y);
		state = 0;
		for (int i = 0; i < 7; ++i){
			counts[i] = 0;
		}
		for (int x = 0; x < binary.cols; ++x) {
			switch (state) {
			case 0:
				// outside (white)
				if (row[x] == 255) {
					counts[state]++;
				}
				else {
					if (counts[state] > 2) {		//***This provides a minimum size of detection
						state++;
						counts[state] = 1;
					}
					else {
						counts[state] = 0;
					}
				}
				states.at<uchar>(y, x) = 0;
				break;
				// Each block has same set up:
				// if same colour, incrememnt current counter check if too large
				// if other colour, switch to next state if counter is large enough (reset to 0 if not)
				// Zeroing counters is handled by the block of code that changes the state
				// The second band (counts[1]) is used for size as the first row can include pixels that are not part of the pattern

				// band 2 (black)
			case 1:
				if (row[x] == 0) {
					counts[state]++;
					if (counts[1] > counts[0] * highFactor + 1) {
						state = 0;
						counts[0] = 0;
					}
				}
				else {
					// No check here (don't have a lower bound on size, as we only have an upper bound on band 1)
					state++;
					counts[state] = 1;
				}
				states.at<uchar>(y, x) = 30;
				break;
				//band 1 (white)
			case 2:
				if (row[x] == 255) {
					counts[state]++;
					if (counts[1] * highFactor + 1 < counts[2]) {
						state = 0;
						counts[0] = counts[2];
					}
				}
				else {
					if (counts[1] * lowFactor - 1 < counts[2]) {
						state++;
						counts[state] = 1;
					}
					else {
						state = 0;
						counts[0] = counts[2];
					}
				}
				states.at<uchar>(y, x) = 60;
				break;
				// Center (black)
			case 3:
				if (row[x] == 0) {
					counts[state]++;
					if (counts[1] * 3 * highFactor + 3 < counts[3]) {
						state = 0;
						counts[0] = 0;
					}
				}
				else {
					if (counts[1] * 3 * lowFactor - 3 < counts[3]) {
						state++;
						counts[state] = 1;
					}
					else if (counts[2] * highFactor + 1> counts[3]) {
						state = 2;
						counts[0] = counts[2];
						counts[1] = counts[3];
						counts[2] = 1;

					}
					else {
						state = 0;
						counts[0] = 0;
					}
				}
				states.at<uchar>(y, x) = 90;
				break;
				// band 1 far side (white)
			case 4:
				if (row[x] == 255) {
					counts[state]++;
					if (counts[state] > counts[1] * highFactor + 1) {
						state = 0;
						counts[0] = counts[4];
					}
				}
				else {
					if (counts[1] * lowFactor - 1 < counts[state]) {
						state++;
						counts[state] = 1;
					}
					else {
						state = 0;
						counts[0] = counts[4];
					}
				}
				states.at<uchar>(y, x) = 120;
				break;
				// band 2 far side (black)
			case 5:
				if (row[x] == 0) {
					counts[state]++;
					if (counts[state] > counts[1] * highFactor + 1) {
						state = 0;
						counts[0] = 0;
					}
				}
				else {
					if (counts[1] * lowFactor - 1 < counts[state]) {
						state++;
						counts[state] = 1;
					}
					else {
						state = 0;
						counts[0] = 0;
					}
				}
				states.at<uchar>(y, x) = 150;
				break;
				// Border far side (white)
			case 6:
				if (row[x] == 255) {
					counts[state]++;
					if (counts[state] > counts[1] * lowFactor - 1) {
						state = 0;
						counts[0] = counts[6];

						Qr_point center;
						center.x = x - counts[6] - counts[5] - counts[4] - counts[3] / 2;
						center.xCenterWidth = counts[3];
						if (verify_y(binary, y, x - counts[6] - counts[5] - counts[4] - counts[3] / 2, lowFactor, highFactor, &center)) {
							// Check if the point is a new one or not.
							bool newPoint = true;
							for (int i = 0; i < centers.size() && newPoint; ++i) {
								if (centers[i].x - centers[i].xCenterWidth  < center.x && centers[i].x + centers[i].xCenterWidth > center.x) {
									if (centers[i].y - centers[i].yCenterWidth < center.y && centers[i].y + centers[i].yCenterWidth > center.y)
										newPoint = false;
								}
							}
							if (newPoint)
								centers.push_back(center);
						}
					}
				}
				else {
					state = 0;
					counts[0] = counts[6];
				}
				states.at<uchar>(y, x) = 200;
				break;
			default:
				std::cout << "Error: states in core/find_qr_centers reached unexpected value "<< state << "\n";
				state = 0;
				counts[0] = 0;
			}
		}
	}

	if (debug && centers.size() > 0) {
		std::cout << "Debug: found " << centers.size() << " centers, ";
		std::cout << "first point at angle " << orient_point(binary, centers[0]) << "\n";
		block_finder(centers[0], src);
	}

	// Sets the colour we will mark the points we found as
	int targ0 = 0;
	int targ1 = 255;
	int targ2 = 255;

	//Mark our image
	for (int i = 0; i < centers.size(); ++i) {
		Qr_point qrp = centers[i];
		//go right
		for (int x = qrp.x, changes = 0, y = qrp.y; x < qrp.x + qrp.xCenterWidth * 2; ++x) {
			if (changes == 0 && binary.at<uchar>(y, x) == 255)
				changes++;
			else if (changes == 1 && binary.at<uchar>(y, x) == 0)
				changes++;
			else if (changes == 2 && binary.at<uchar>(y, x) == 255)
				break;
			res.at<Vec3b>(y, x)[0] = targ0;
			res.at<Vec3b>(y, x)[1] = targ1;
			res.at<Vec3b>(y, x)[2] = targ2;
		}
		//go left
		for (int x = qrp.x, changes = 0, y = qrp.y; x > qrp.x - qrp.xCenterWidth * 2; --x) {
			if (changes == 0 && binary.at<uchar>(y, x) == 255)
				changes++;
			else if (changes == 1 && binary.at<uchar>(y, x) == 0)
				changes++;
			else if (changes == 2 && binary.at<uchar>(y, x) == 255)
				break;
			res.at<Vec3b>(y, x)[0] = targ0;
			res.at<Vec3b>(y, x)[1] = targ1;
			res.at<Vec3b>(y, x)[2] = targ2;
		}
		//go down
		for (int x = qrp.x, changes = 0, y = qrp.y; y < qrp.y + qrp.yCenterWidth * 2; ++y) {
			if (changes == 0 && binary.at<uchar>(y, x) == 255)
				changes++;
			else if (changes == 1 && binary.at<uchar>(y, x) == 0)
				changes++;
			else if (changes == 2 && binary.at<uchar>(y, x) == 255)
				break;
			res.at<Vec3b>(y, x)[0] = targ0;
			res.at<Vec3b>(y, x)[1] = targ1;
			res.at<Vec3b>(y, x)[2] = targ2;
		}
		//go up
		for (int x = qrp.x, changes = 0, y = qrp.y; y > qrp.y - qrp.yCenterWidth * 2; --y) {
			if (changes == 0 && binary.at<uchar>(y, x) == 255)
				changes++;
			else if (changes == 1 && binary.at<uchar>(y, x) == 0)
				changes++;
			else if (changes == 2 && binary.at<uchar>(y, x) == 255)
				break;
			res.at<Vec3b>(y, x)[0] = targ0;
			res.at<Vec3b>(y, x)[1] = targ1;
			res.at<Vec3b>(y, x)[2] = targ2;
		}
	}
	if (centers.size() > 0) {
		putText(res, block_finder(centers[centers.size() - 1], res), Point(25, res.rows - 25), FONT_HERSHEY_COMPLEX_SMALL, 0.7, cvScalar(0, 0, 0));
	}
	
	// If debugging, display the images and wait for a keypress
	if (debug) {
		imshow("Source", src);
		imshow("Binary", binary);
		imshow("States", states);
		imshow("Result", res);
		waitKey(1);
	}

	return res;
}


// Takes a point and checks it in the y direction.
// Also fills out the centerObj y information
bool verify_y(Mat binary, int cy, int cx, double lowFactor, double highFactor, Qr_point * centerObj) {
	int counts[7] = { 0 };
	int y = cy;
	int centerCountsAbove = 0;
	int centerCountsBelow = 0;
	// Each logic block corresponds to a band. Note that there are two for the middle, as we start at
	// an indeterminate point in the center of the pattern.
	// Each block has the condition to continue (colour change, or for borders minimum size)
	// as well as conditions which immediately disqualify the pattern (extend off image borders, sizes not consistent)
	while (binary.at<uchar>(y, cx) == 0) {
		counts[3]++;
		centerCountsAbove++;
		--y;
		if (y < 0)
			return false;
	}

	while (binary.at<uchar>(y, cx) == 255) {
		counts[2]++;
		--y;
		if (y < 0)
			return false;
	}

	while (binary.at<uchar>(y, cx) == 0) {
		counts[1]++;
		--y;
		if (y < 0)
			return false;
	}
	if (counts[1] * lowFactor - 1 > counts[2] || counts[1] * highFactor + 1 < counts[2])
		return false;

	while (binary.at<uchar>(y, cx) == 255) {
		counts[0]++;
		--y;
		if (counts[0] > counts[1] * lowFactor - 1)
			break;
		if (y < 0)
			return false;
	}

	y = cy + 1;
	while (binary.at<uchar>(y, cx) == 0) {
		counts[3]++;
		centerCountsBelow++;
		++y;
		if (y >= binary.rows)
			return false;
	}
	if (counts[1] * lowFactor * 3 - 3 > counts[3] || counts[1] * highFactor * 3 + 3 < counts[3])
		return false;

	while (binary.at<uchar>(y, cx) == 255) {
		counts[4]++;
		++y;
		if (y >= binary.rows)
			return false;
	}
	if (counts[1] * lowFactor - 1 > counts[4] || counts[1] * highFactor + 1 < counts[4])
		return false;

	while (binary.at<uchar>(y, cx) == 0) {
		counts[5]++;
		++y;
		if (y >= binary.rows)
			return false;
	}
	if (counts[1] * lowFactor - 1 > counts[5] || counts[1] * highFactor + 1 < counts[5])
		return false;

	while (binary.at<uchar>(y, cx) == 255) {
		counts[6]++;
		++y;
		if (counts[6] > counts[1] * lowFactor - 1) {
			centerObj->y = cy + (centerCountsBelow - centerCountsAbove) / 2;
			centerObj->yCenterWidth = counts[3];
			return true;
		}
		if (y >= binary.rows)
			return false;
	}
	// Last white band is too small
	return false;
}


// Simplified locator method that finds the distance and displacement to the code
// Assumes it is looking for a block sitting flat on the same plane as the camera
std::string block_finder(Qr_point qp, Mat img) {
	double fovX = 40;
	double fovY = 22.5;
	double targetSize = 0.8; // qr center, in cm
	int overshoot = 15;	//Tells the arm to go this many cm past where the target is
	int xDisplace = 0;	//Camera is this many cm to the right the base of the arm, viewed from behind
	int yDisplace = 0;
	int zDisplace = 0;
	double pi = 3.14159265358979323;

	// Distance is a simple calculation using the y component of the center size.
	double dist = targetSize / (double(qp.yCenterWidth) / img.rows * fovY * pi / 180);

	double theta = (fovX / img.cols * qp.x) - (fovX / 2);
	double phi = 0 - ((fovY / img.rows * qp.y) - (fovY / 2));

	double y = dist * cos(theta * pi / 180) * cos(phi * pi / 180) - yDisplace;
	double x = dist * sin(theta * pi / 180) * cos(phi * pi / 180) - xDisplace;
	double z = dist * cos(theta * pi / 180) * sin(phi * pi / 180) - zDisplace;

	std::string data = "Distance: ";
	data.append(std::to_string(dist));
	data.append("  X: ");
	data.append(std::to_string(x));
	data.append("  Y: ");
	data.append(std::to_string(y));
	data.append("  Z: ");
	data.append(std::to_string(z));
	return data;
}


// TODO: can be used to locate the point in 3d space
void locate_point(Mat binary, Qr_point qp) {
	// tracks the points we've already added to the heap (to avoid going in circles)
	// todo: big tracker for a little traversal. Room for optimization here. Entire traversal could probably be optimized away with some effort.
	Mat tracker = Mat::zeros(binary.size(), CV_8UC1);
	// Tracks which adjacent points we still need to visit
	std::vector<Point> movingHeap;
	// Tracks all points we have visited. This will get fed into the bounding rectangle function
	std::vector<Point> histHeap;
	// initialize our first point on the heap
	movingHeap.push_back(Point(qp.x, qp.y));
	while (movingHeap.size() > 0) {
		// Save the point
		Point p = movingHeap[movingHeap.size() - 1];
		movingHeap.pop_back();
		histHeap.push_back(p);
		// Look at adjacent points
		Point points[4];
		points[0] = Point(p.x + 1, p.y);
		points[1] = Point(p.x - 1, p.y);
		points[2] = Point(p.x, p.y + 1);
		points[3] = Point(p.x, p.y - 1);
		for (int i = 0; i < 4; ++i) {
			// Skip if outside of bounds
			if (points[i].x < 0 || points[i].x >= binary.cols || points[i].y < 0 || points[i].y > binary.rows)
				continue;
			// Add adjacent points if valid and not yet looked at
			if (tracker.at<uchar>(points[i]) == 0 && binary.at<uchar>(points[i]) == 0) {
				tracker.at<uchar>(points[i]) = 1;
				movingHeap.push_back(points[i]);
			}
		}
	}
	RotatedRect rectPoints = minAreaRect(histHeap);
	double zrot = 0 - rectPoints.angle;
	double yrot;
	double xrot;
	double dist;
	int displaceX;
	int displaceY;
}

/*
Qr_code join_points(Mat binary, Qr_point p1, Qr_point p2, Qr_point p3) {
	double p1orient = orient_point(binary, p1);
	double angleToP2 = angle_between_points(p1.x, p2.x, p1.y, p2.y);
	double angleToP3 = angle_between_points(p1.x, p3.x, p1.y, p3.y);
	double p2Relative = angleToP2 - p1orient;
	double p2PerpVal = int(p2Relative + 90) % 90;
	double p3Relative = angleToP3 - p1orient;
	double p3PerpVal = int(p3Relative + 90) % 90;
	//TODO
}*/


double angle_between_points(int x1, int x2, int y1, int y2) {
	int deltaY = y1 - y2;	//Inversed because we want an "up" change positive, but are counting down
	int deltaX = x2 - x1;
	return atan2(deltaY, deltaX) * 180 / 3.141592653589793238462643;	//convert to degrees
}


// Gives the rotation clockwise from "no rotation" in degrees (0-90 due to radial symmetry)
// Depreciated, included in locate_point above
double orient_point(Mat binary, Qr_point qp) {
	// tracks the points we've already added to the heap (to avoid going in circles)
	// todo: big tracker for a little traversal. Room for optimization here. Entire traversal could probably be optimized away with some effort.
	Mat tracker = Mat::zeros(binary.size(), CV_8UC1);
	// Tracks which adjacent points we still need to visit
	std::vector<Point> movingHeap;
	// Tracks all points we have visited. This will get fed into the bounding rectangle function
	std::vector<Point> histHeap;
	// initialize our first point on the heap
	movingHeap.push_back(Point(qp.x, qp.y));
	while (movingHeap.size() > 0) {
		// Save the point
		Point p = movingHeap[movingHeap.size() - 1];
		movingHeap.pop_back();
		histHeap.push_back(p);
		// Look at adjacent points
		Point points[4];
		points[0] = Point(p.x + 1, p.y);
		points[1] = Point(p.x - 1, p.y);
		points[2] = Point(p.x, p.y + 1);
		points[3] = Point(p.x, p.y - 1);
		for (int i = 0; i < 4; ++i) {
			// Skip if outside of bounds
			if (points[i].x < 0 || points[i].x >= binary.cols || points[i].y < 0 || points[i].y >= binary.rows)
				continue;
			// Add adjacent points if valid and not yet looked at
			if (tracker.at<uchar>(points[i]) == 0 && binary.at<uchar>(points[i]) == 0) {
				tracker.at<uchar>(points[i]) = 1;
				movingHeap.push_back(points[i]);
			}
		}
	}
	RotatedRect rectPoints = minAreaRect(histHeap);
	return 0 - rectPoints.angle;
}


// Returns a black and white image (type: 1-channel uchar, every pixel is either 0 or 255)
// Separate method to make easier to change across code. Currently uses a very simplified approach
// that doesn't deal well with lighting changes
Mat binarize_image(Mat colImg) {
	Mat grey;
	Mat binary;
	cvtColor(colImg, grey, CV_BGR2GRAY);
	threshold(grey, binary, 125, 255, THRESH_BINARY);
	return binary;
}


// Returns a black and white image (type: 1-channel uchar, every pixel is either 0 or 255)
Mat binarize_image_smrt(Mat colImg) {
	Mat grey;
	cvtColor(colImg, grey, COLOR_BGR2GRAY);
	Mat res;
	adaptiveThreshold(grey, res, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
	return res;
}
#include "core.h"
#include <string>

int main() {
	//img_test();
	// Set up the system default webcam (the '0' in the constructor)
	// Should be cross-platform
	VideoCapture cam(0);

	int counter = 0;
	while (true) {
		// Get the most recent image from the webcam
		Mat frame;
		cam.read(frame);

		Mat marked = find_qr_center(frame, true);
		//imwrite("C:/Workspace/" + std::to_string(counter) + std::string(".jpg"), frame);

		counter++;
	}
	return 0;
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
					if (counts[state] > 0) {
						state++;
						counts[state] = 1;
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
				std::cout << "Error: states in core/find_qr_centers reached unexpected value\n";
			}
		}
	}

	if (debug && centers.size() > 0) {
		std::cout << "Debug: found " << centers.size() << " centers\n";
	}

	// Sets the colour we will mark the points we found as
	int targ0 = 0;
	int targ1 = 255;
	int targ2 = 255;

	// Go to the saved points, and do a traversal of all connected black pixels. Mark them all the target colour above
	// Tracker tracks which pixels we've added to the heap already (don't go in circles)
	Mat tracker = Mat::zeros(binary.rows, binary.cols, CV_8UC1);
	for (int i = 0; i < centers.size(); ++i){
		Qr_point qrp = centers[i];
		int x0 = qrp.x;
		int y0 = qrp.y;
		std::vector<Point> heap;
		// skip if we've already marked the point
		if (tracker.at<uchar>(qrp.y, qrp.x) == 1)
			continue;
		heap.push_back(Point(qrp.x, qrp.y));
		while (heap.size() > 0) {
			// Colour the next point
			Point p = heap[heap.size() - 1];
			heap.pop_back();
			res.at<Vec3b>(p)[0] = targ0;
			res.at<Vec3b>(p)[1] = targ1;
			res.at<Vec3b>(p)[2] = targ2;

			// Don't check for adjacent pixels if too far away from the original marked pixel
			// Not very clean, but prevents this part from traversing a large block of the image
			// (big performance hit) if a mistake is made earlier.
			if (abs(x0 - p.x) > 5 || abs(y0 - p.y) > 5)
				continue;

			// Search for any adjacent points (checking for bounds and repeats)
			for (int y = p.y - 1; y <= p.y + 1; ++y) {
				if (y < 0 || y >= res.rows)
					continue;
				for (int x = p.x - 1; x <= p.x + 1; ++x) {
					if (x < 0 || x >= res.cols)
						continue;
					if (tracker.at<uchar>(y, x) == 0 && binary.at<uchar>(y, x) == 0) {
						heap.push_back(Point(x, y));
						tracker.at<uchar>(y, x) = 1;
					}
				}
			}
		}
	}
	//Display the images if debugging and wait for a keypress
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


// Returns the integer value of the point that is the "middle" point
int join_points(Mat img, Qr_point p1, Qr_point p2, Qr_point p3) {

}


// Returns a black and white image (type: 1-channel uchar, every pixel is either 0 or 255)
// Separate method to make easier to change across code. Currently uses a very simplified approach
// that doesn't deal well with lighting changes
Mat binarize_image(Mat colImg) {
	Mat grey;
	Mat binary;
	cvtColor(colImg, grey, CV_BGR2GRAY);
	threshold(grey, binary, 100, 255, THRESH_BINARY);
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
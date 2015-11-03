#ifndef IMAGEPROC_H
#define IMAGEPROC_H

#include <iostream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

const Size MAZE_SIZE3(500, 500);

class ImageProc
{
public:
	void processing_grid(Mat, Mat);
	void get_point(Mat, Point &, int);
	Mat undistorted_grid(Mat, Mat);

private:
	
};

#endif

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
    void resize_to_max(Mat &, int);
	void preprocess_image(Mat &, Mat &);
	void get_point(Mat, Point2f &, int, int, int, int);
	Mat undistorted_grid(Mat, Mat);
    bool get_triangle(Mat, Vec2f &, Point2f &);

private:
	
};

#endif

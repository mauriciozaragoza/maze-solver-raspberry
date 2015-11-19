// g++ main.cpp -o main -std=c++0x `pkg-config --cflags --libs opencv`

#include <iostream>
#include <queue>
#include <vector>
#include <tuple>
#include <string>

#include "maze.h"
#include "img_processing.h"
#include "control.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    const char* image_name = argc > 1 ? argv[1] : "maze.png";

	ImageProc processing;
	Maze solver;

	Mat thresh_maze = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
	Mat color_maze = imread(image_name, CV_LOAD_IMAGE_COLOR);
	// Mat grid = Mat(maze.size(), CV_8UC1);

	// imshow("original", maze);

	// Preprocess the image
	processing.preprocess_image(thresh_maze, thresh_maze);

	processing.resize_to_max(thresh_maze, 500);
	processing.resize_to_max(color_maze, 500);

	// Perspective correcting requires walls to be white
	bitwise_not(thresh_maze, thresh_maze);

	// Correct perspective
	color_maze = processing.undistorted_grid(thresh_maze, color_maze);
	thresh_maze = processing.undistorted_grid(thresh_maze, thresh_maze);

	// Return wall colors to normal
	bitwise_not(thresh_maze, thresh_maze);

	processing.resize_to_max(thresh_maze, 500);
	processing.resize_to_max(color_maze, 500);

	Point start;
	Point end;

	processing.get_point(color_maze, start, 0, 20);
	processing.get_point(color_maze, end, 120, 20);

	// imshow("to thresh_maze", thresh_maze);

	cout << start << " " << end << endl;

	// imshow("un", thresh_maze);

	cout << "A" << endl;
	solver.depth_first_search(color_maze, thresh_maze, start.x, start.y, end.x, end.y);
	cout << "B" << endl;
	solver.draw_path(color_maze, start.x, start.y);
	cout << "C" << endl;

	imshow("sol", color_maze);

	cvWaitKey();
	return 1;
}

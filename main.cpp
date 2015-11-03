// g++ main.cpp -o main -std=c++0x `pkg-config --cflags --libs opencv`

#include <iostream>
#include <queue>
#include <vector>
#include <tuple>
#include <string>

#include "maze.h"
#include "img_processing.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

Size MAZE_SIZE2(250, 250);

int main()
{
	Mat maze = imread("maze.png", 0);
	Mat grid = Mat(maze.size(), CV_8UC1);

	ImageProc processing;
	Maze solver;


	// imshow("original", maze);

	processing.processing_grid(maze, grid);

	Mat color_maze = imread("maze.png", CV_LOAD_IMAGE_COLOR);
	Mat undistorted = processing.undistorted_grid(grid, color_maze);

	Point start;
	Point end;
	processing.get_points(undistorted, start, end);

	Mat solve = processing.undistorted_grid(grid, grid);
	bitwise_not(solve, solve);

	resize(solve, solve, MAZE_SIZE2);
	resize(undistorted, undistorted, MAZE_SIZE2);

	// imshow("to solve", solve);

	cout << start;
	cout << end;

	solver.depth_first_search(undistorted, solve, start.x, start.y, end.x, end.y);
	
	imshow("un", undistorted);
	// imshow("sol", solve);

	cvWaitKey();
	return 1;
}
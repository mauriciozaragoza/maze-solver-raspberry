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

ImageProc processing;
Maze solver;
Control control;

const double PI = 3.14159265358979;

void preprocess(Mat &thresh_maze, Mat &color_maze, int size) 
{
	// Mat grid = Mat(maze.size(), CV_8UC1);

	// imshow("original", maze);

	// Preprocess the image
	processing.preprocess_image(thresh_maze, thresh_maze);

	processing.resize_to_max(thresh_maze, size);
	processing.resize_to_max(color_maze, size);

	// Perspective correcting requires walls to be white
	bitwise_not(thresh_maze, thresh_maze);

	// Correct perspective
	color_maze = processing.undistorted_grid(thresh_maze, color_maze);
	thresh_maze = processing.undistorted_grid(thresh_maze, thresh_maze);

	// Return wall colors to normal
	bitwise_not(thresh_maze, thresh_maze);

	processing.resize_to_max(thresh_maze, size);
	processing.resize_to_max(color_maze, size);
}

int main(int argc, char** argv)
{
    const char* image_name = argc > 1 ? argv[1] : "maze.png";

    Vec2f vehicle_angle(0.0f, 1.0f);
    const double vehicle_speed = 1.0;
    const double rotation_speed = 0.2;

	Point2f vehicle;
	Point2f end;

	Mat thresh_maze = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
	Mat color_maze = imread(image_name, CV_LOAD_IMAGE_COLOR);

	preprocess(thresh_maze, color_maze, 500);
	// imshow("to thresh_maze", thresh_maze);

	processing.get_point(color_maze, vehicle, 120, 20);
	processing.get_point(color_maze, end, 0, 20);

	cout << vehicle << " " << end << " " << endl;

	cout << "A" << endl;
	solver.depth_first_search(color_maze, thresh_maze, end.x, end.y);

	cout << "B" << endl;
	solver.draw_path(color_maze, vehicle.x, vehicle.y);
	cout << "Starting cycle" << endl;

	for (int step = 0; 
		step < 10000 && 
		!(abs(vehicle.x - end.x) < 10 && abs(vehicle.y - end.y) < 10); step++) 
	{
		pair<int, int> next_destination = solver.next_step(vehicle.x, vehicle.y, 20);

		// Compute the next angle
		// Take current rotation into account
		double angle = 
			atan2(next_destination.second - vehicle.y, next_destination.first - vehicle.x) -
			atan2(vehicle_angle[1], vehicle_angle[0]);

		double new_v_x, new_v_y;

		// cout << vehicle << " " << vehicle_angle << " " << angle << " dest=[ " << next_destination.first << " " << next_destination.second << " ] " <<step << endl;

		color_maze.at<Vec3b>(next_destination.second, next_destination.first) = Vec3b(255, 255, 0);

		if (angle < -PI / 16.0)
		{
			control.left();
			new_v_x = vehicle_angle[0] * cos(-rotation_speed) - vehicle_angle[1] * sin(-rotation_speed);
			new_v_y = vehicle_angle[0] * sin(-rotation_speed) + vehicle_angle[1] * cos(-rotation_speed);
			vehicle_angle[0] = new_v_x;
			vehicle_angle[1] = new_v_y;
			norm(vehicle_angle);
		}
		else if (angle > PI / 16.0) 
		{
			control.right();
			new_v_x = vehicle_angle[0] * cos(rotation_speed) - vehicle_angle[1] * sin(rotation_speed);
			new_v_y = vehicle_angle[0] * sin(rotation_speed) + vehicle_angle[1] * cos(rotation_speed);
			vehicle_angle[0] = new_v_x;
			vehicle_angle[1] = new_v_y;
			norm(vehicle_angle);
		}
		else 
		{
			control.forward();
			vehicle.x += (vehicle_speed * (1.0 + (float)(rand() % 10) / 9.0f)) * vehicle_angle[0];
			vehicle.y += (vehicle_speed * (1.0 + (float)(rand() % 10) / 9.0f)) * vehicle_angle[1];

			vehicle.x = std::min(std::max((float)vehicle.x, 0.0f), color_maze.cols - 1.0f);
			vehicle.y = std::min(std::max((float)vehicle.y, 0.0f), color_maze.rows - 1.0f);
		}

		if (vehicle.y >= 0 && vehicle.y < color_maze.rows &&
			vehicle.x >= 0 && vehicle.x < color_maze.cols) {
				color_maze.at<Vec3b>(vehicle.y, vehicle.x) = Vec3b(255, 0, 255);
		}
	}

	control.stop();

	imshow("sol", color_maze);

	cvWaitKey();
	return 1;
}

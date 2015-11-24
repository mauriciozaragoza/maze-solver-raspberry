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

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace cv;
using namespace std;

ImageProc processing;
Maze solver;
Control control;
VideoCapture cap;

const double PI = 3.14159265358979;

void my_handler(int s)
{
   printf("Caught signal %d\n", s);
   control.stop();
   cap.release();
   exit(1);
}

void preprocess(Mat &thresh_maze, Mat &color_maze, int size) 
{
	// Mat grid = Mat(maze.size(), CV_8UC1);

	// imshow("original", maze);

	// Preprocess the image
	processing.preprocess_image(thresh_maze, thresh_maze);

	processing.resize_to_max(thresh_maze, size);
	processing.resize_to_max(color_maze, size);

	// Perspective correcting requires walls to be white
	// bitwise_not(thresh_maze, thresh_maze);

	// // Correct perspective
	// color_maze = processing.undistorted_grid(thresh_maze, color_maze);
	// thresh_maze = processing.undistorted_grid(thresh_maze, thresh_maze);

	// // Return wall colors to normal
	// bitwise_not(thresh_maze, thresh_maze);

	// processing.resize_to_max(thresh_maze, size);
	// processing.resize_to_max(color_maze, size);
}

int main(int argc, char** argv)
{
    // const char* image_name = argc > 1 ? argv[1] : "t1.png";

	// Mat triangle = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
	// VideoCapture cap;

	// cout << atan2(1, 0) << endl;
	// cout << atan2(1, 0) << endl;

 	if (!cap.open(0)) 
 	{
 		cout << "Camera not available" << endl;
        return 0;
 	}
	
	// Mat triangle;
	// Point2f position;
	// cap >> triangle;

	// Vec2f angle;

	// if (processing.get_triangle(triangle, angle, position)) 
	// {
	// 	cout << angle << " " << position << endl;
	// }
	// else 
	// {
	// 	cout << "No triangle found" << endl;
	// }

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

	control.stop();
 	
	freopen("values.txt", "r", stdin);

	int c1, c2, c3;

	cin >> c1 >> c2 >> c3;

 	if (!cap.open(0)) 
 	{
 		cout << "Camera not available" << endl;
        return 0;
 	}

 	Mat color_maze, position_maze;
	cap >> color_maze;

	Mat thresh_maze(color_maze.rows, color_maze.cols, CV_8UC1);

	cvtColor(color_maze, thresh_maze, COLOR_BGR2GRAY);
	// imshow("webcam", color_maze);

    // const char* image_name = argc > 1 ? argv[1] : "maze.png";

    Vec2f vehicle_angle(0.0f, 1.0f);
    const double vehicle_speed = 1.0;
    const double rotation_speed = 0.2;

	Point vehicle, end;

	// Mat thresh_maze = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
	// Mat color_maze = imread(image_name, CV_LOAD_IMAGE_COLOR);

	preprocess(thresh_maze, color_maze, 500);
	// imshow("to thresh_maze", thresh_maze);

	// processing.get_point(color_maze, vehicle, c1, 20, 100, 100);
	// processing.get_point(color_maze, vehicle_front, c2, 20, 100, 100);
	processing.get_point(color_maze, end, c3, 15, c1, c2);

	// circle(color_maze, vehicle, 10, Scalar(255, 0, 0));
	// circle(color_maze, vehicle_front, 10, Scalar(0, 255, 0));
	// circle(color_maze, end, 10, Scalar(0, 0, 255));

	cout << "Destination: " << end << endl;

	imwrite("webcam.png", color_maze);

	cout << "A" << endl;
	solver.depth_first_search(color_maze, thresh_maze, end.x, end.y);

	cout << "B" << endl;
	// solver.draw_path(color_maze, vehicle.x, vehicle.y);
	cout << "Starting cycle" << endl;

	for (int step = 0; step < 500; step++) 
	{
// cout << "asadas" << endl;
		cap >> position_maze;

		if (!processing.get_triangle(position_maze, vehicle_angle, vehicle))
		{
			cout << "Could not find triangle" << endl;
		}
// cout << "bsadas " << vehicle_angle << " " << vehicle << endl;

		pair<int, int> next_destination = solver.next_step(vehicle.x, vehicle.y, 20);
// cout << "csadas" << endl;
		if (next_destination.first == -1) break;

		// Compute the next angle
		// Take current rotation into account
		// double angle = 
		// 	atan2(next_destination.second - vehicle.y, next_destination.first - vehicle.x) -
		// 	atan2(vehicle_angle[1], vehicle_angle[0]);

		Vec2f next_step_vector(next_destination.second - vehicle.y, next_destination.first - vehicle.x);
		next_step_vector = next_step_vector / norm(next_step_vector);

		// Obtain the angle between the two vectors
		// double angle = acos()

		double angle = 
			atan2(next_step_vector[1], next_step_vector[0]) - 
			atan2(vehicle_angle[1], vehicle_angle[0]);

// cout << "dsadas" << endl;
		cout << vehicle << " " << vehicle_angle << " " << angle << " " << step << endl;

		color_maze.at<Vec3b>(next_destination.second, next_destination.first) = Vec3b(255, 255, 0);

		if (angle < -PI / 24.0)
		{
			control.right();
			cout << "left" << endl;
			// new_v_x = vehicle_angle[0] * cos(-rotation_speed) - vehicle_angle[1] * sin(-rotation_speed);
			// new_v_y = vehicle_angle[0] * sin(-rotation_speed) + vehicle_angle[1] * cos(-rotation_speed);
			// vehicle_angle[0] = new_v_x;
			// vehicle_angle[1] = new_v_y;
			// vehicle_angle = norm(vehicle_angle);
		}
		else if (angle > PI / 24.0) 
		{
			control.left();
			cout << "right" << endl;
			// new_v_x = vehicle_angle[0] * cos(rotation_speed) - vehicle_angle[1] * sin(rotation_speed);
			// new_v_y = vehicle_angle[0] * sin(rotation_speed) + vehicle_angle[1] * cos(rotation_speed);
			// vehicle_angle[0] = new_v_x;
			// vehicle_angle[1] = new_v_y;
			// vehicle_angle = norm(vehicle_angle);
		}
		else 
		{
			control.forward();
			cout << "forward" << endl;
			// vehicle.x += (vehicle_speed * (1.0 + (float)(rand() % 10) / 9.0f)) * vehicle_angle[0];
			// vehicle.y += (vehicle_speed * (1.0 + (float)(rand() % 10) / 9.0f)) * vehicle_angle[1];

			// vehicle.x = std::min(std::max((float)vehicle.x, 0.0f), color_maze.cols - 1.0f);
			// vehicle.y = std::min(std::max((float)vehicle.y, 0.0f), color_maze.rows - 1.0f);
		}

		if (vehicle.y >= 0 && vehicle.y < color_maze.rows &&
			vehicle.x >= 0 && vehicle.x < color_maze.cols) {
				color_maze.at<Vec3b>(vehicle.y, vehicle.x) = Vec3b(255, 0, 255);
		}

		// delay(100);
	}
	
	cout << "Finished!" << endl;
	control.stop();

	// imshow("sol", color_maze);
	imwrite("sol.png", color_maze);

	cvWaitKey();
	return 1;
}

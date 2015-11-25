// g++ main.cpp -o main -std=c++0x `pkg-config --cflags --libs opencv`

#include <iostream>
#include <iomanip>
#include <queue>
#include <vector>
#include <tuple>
#include <string>
#include <sstream>

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

#include <wiringPi.h>

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
}

int main(int argc, char** argv)
{
    const char* image_name = argc > 1 ? argv[1] : "maze6.png";

	// Mat triangle = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
	// VideoCapture cap;

	// cout << atan2(1, 0) << endl;
	// cout << atan2(1, 0) << endl;

 	// if (!cap.open(0)) 
 	// {
 	// 	cout << "Camera not available" << endl;
  //       return 0;
 	// }
	
	// Mat triangle;
	// Point position;
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

	// // line(color_maze, vehicle, vehicle + Point((next_step_vector * 10)[0], (next_step_vector * 10)[1]), 2);
	// line(triangle, position, position + Point((angle * 100)[0], (angle * 100)[1]), 2);

	// imwrite("triangle.png", triangle);

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

	control.stop();
 	
	freopen("values.txt", "r", stdin);

	int c1, c2, c3;

	cin >> c1 >> c2 >> c3;

	Mat color_maze = imread(image_name, CV_LOAD_IMAGE_COLOR),
		position_maze;

 	// if (!cap.open(0)) 
 	// {
 	// 	cout << "Camera not available" << endl;
  //       return 0;
 	// }

 	// Mat color_maze, position_maze;
	// cap >> color_maze;

	Mat thresh_maze(color_maze.rows, color_maze.cols, CV_8UC1);

	cvtColor(color_maze, thresh_maze, COLOR_BGR2GRAY);

    Vec2f vehicle_angle(0.0f, 1.0f);
    const double vehicle_speed = 5.0;
    const double rotation_speed = 0.3;

    double new_v_y, new_v_x;

	Point2f vehicle, end;

	preprocess(thresh_maze, color_maze, 500);

	// processing.get_point(color_maze, end, c3, 15, c1, c2);
	end = Point2f(20, 20);

	// DEMO
	if (!processing.get_triangle(color_maze, vehicle_angle, vehicle))
	{
		cout << "NO TRIANGLE" << endl;
		vehicle = Point2f(300, 300);
		imwrite("webcam.png", color_maze);
	}
	else 
	{
		cout << "position " << vehicle << endl;
	}

	circle(color_maze, end, 10, Scalar(0, 0, 255));

	cout << "Destination: " << end << " start: " << vehicle << endl;

	imwrite("webcam.png", color_maze);

	cout << "Performing search" << endl;
	solver.depth_first_search(color_maze, thresh_maze, end.x, end.y);

	cout << "Drawing path" << endl;
	solver.draw_path(color_maze, vehicle.x, vehicle.y);
	imwrite("path.png", color_maze);
	
	cout << "Starting cycle" << endl;

	for (int step = 0; step < 500; step++) 
	{
		Mat current_frame = color_maze.clone();

		// cap >> position_maze;

		// if (!processing.get_triangle(position_maze, vehicle_angle, vehicle))
		// {
		// 	cout << "Could not find triangle" << endl;
		// }

		Vec2f next_step_vector = solver.next_step(vehicle.x, vehicle.y, 5);

		if (next_step_vector == Vec2f()) 
		{
			cout << "Found exit!" << endl;
			break;
		}

		// Obtain the angle between the two vectors
		// double angle = 
		// 	atan2(next_step_vector[1], next_step_vector[0]) - 
		// 	atan2(vehicle_angle[1], vehicle_angle[0]);

		line(current_frame, vehicle, vehicle + Point2f((next_step_vector * 10)[0], (next_step_vector * 10)[1]), Scalar(0, 0, 255), 2);
		line(current_frame, vehicle, vehicle + Point2f((vehicle_angle * 20)[0], (vehicle_angle * 20)[1]), Scalar(255, 0, 0), 2);
		circle(current_frame, vehicle, 3, Scalar(0, 255, 255));
		// color_maze.at<Vec3b>(next_destination.second, next_destination.first) = Vec3b(255, 255, 0);

		double direction = next_step_vector[0] * vehicle_angle[1] - vehicle_angle[0] * next_step_vector[1];

		cout << vehicle << " " << next_step_vector << " " << vehicle_angle << " " << direction << " " << step << endl;

		if (direction > 0.2)
		{
			control.left();
			cout << "left" << endl;
			new_v_x = vehicle_angle[0] * cos(-rotation_speed) - vehicle_angle[1] * sin(-rotation_speed);
			new_v_y = vehicle_angle[0] * sin(-rotation_speed) + vehicle_angle[1] * cos(-rotation_speed);
			vehicle_angle[0] = new_v_x;
			vehicle_angle[1] = new_v_y;
		}
		else if (direction < -0.2) 
		{
			control.right();
			cout << "right" << endl;
			new_v_x = vehicle_angle[0] * cos(rotation_speed) - vehicle_angle[1] * sin(rotation_speed);
			new_v_y = vehicle_angle[0] * sin(rotation_speed) + vehicle_angle[1] * cos(rotation_speed);
			vehicle_angle[0] = new_v_x;
			vehicle_angle[1] = new_v_y;
		}
		else 
		{
			control.forward();
			cout << "forward " << vehicle << " moving by " << vehicle_speed * vehicle_angle[0] 
				 << " , " << vehicle_speed * vehicle_angle[1] << endl;

			vehicle.x += vehicle_speed * vehicle_angle[0];
			vehicle.y += vehicle_speed * vehicle_angle[1];

			vehicle.x = std::min(std::max(vehicle.x, 0.0f), color_maze.cols - 1.0f);
			vehicle.y = std::min(std::max(vehicle.y, 0.0f), color_maze.rows - 1.0f);
		}

		if (vehicle.y >= 0 && vehicle.y < color_maze.rows &&
			vehicle.x >= 0 && vehicle.x < color_maze.cols) {
				color_maze.at<Vec3b>(vehicle.y, vehicle.x) = Vec3b(255, 0, 255);
		}

		ostringstream ss;
		ss << "frames/" << setfill('0') << setw(3) << step << ".png";

		imwrite(ss.str(), current_frame);

		// delay(50);
	}
	
	cout << "Finished!" << endl;
	control.stop();

	// imshow("sol", color_maze);
	imwrite("sol.png", color_maze);

	cvWaitKey();
	return 1;
}

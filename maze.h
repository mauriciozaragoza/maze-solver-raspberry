#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <queue>
#include <vector>
#include <tuple>
#include <functional>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

typedef tuple<double, int, int, int, int> state;

class Maze
{
public:
	double euclidean(int, int, int, int);
	double heuristic(int, int, int, int, Mat &);
	void depth_first_search(Mat &, Mat &, int, int, int, int);

private:
	
};

#endif
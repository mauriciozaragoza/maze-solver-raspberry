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

typedef tuple<int, int, int, int> state;
typedef tuple<double, int, int, int, int> shortest_state;

class Maze
{
public:
	double euclidean(double, double, double, double);
	double manhattan(double, double, double, double);
	void depth_first_search(Mat &, Mat &, int, int);
	Vec2f next_step(int, int, int);
	void draw_path(Mat &, int, int);

private:
	vector <vector <pair<int, int> > > path;
};

#endif
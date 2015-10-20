// g++ main.cpp -o main -std=c++0x `pkg-config --cflags --libs opencv`

#include <iostream>
#include <queue>
#include <vector>
#include <tuple>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

typedef tuple<int, int, int, int, int> state;

int dest_x = 150;
int dest_y = 0;
int start_x = 0;
int start_y = 45;

int euclidean(int x, int y, int x2, int y2)
{
	return sqrt(pow((x - x2), 2) + pow((y - y2), 2));
}

void depth_first_search(Mat &maze, Mat &solve)
{	
	pair<int, int> path[500][500];
	priority_queue< state, vector<state>, greater<state> > q;
	q.push(make_tuple(0, start_x, start_y, 0, 0));

	while (!q.empty())
	{	
		state current = q.top();
		q.pop();

		int weight = get<0>(current);
		int x = get<1>(current);
		int y = get<2>(current);
		int x_prev = get<3>(current);
		int y_prev = get<4>(current);

		if (x < 0 || x >= maze.cols || y < 0 || y >= maze.rows) continue;
		if (solve.at<unsigned char>(y, x) == 0) continue;
		
		path[y][x] = make_pair(y_prev, x_prev);
		
		if (x == dest_x && y == dest_y) break;

		solve.at<unsigned char>(y, x) = 0;

		q.push(make_tuple(weight + euclidean(x, y, dest_x, dest_y), x + 1, y, x, y));
		q.push(make_tuple(weight + euclidean(x, y, dest_x, dest_y), x - 1, y, x, y));
		q.push(make_tuple(weight + euclidean(x, y, dest_x, dest_y), x, y + 1, x, y));
		q.push(make_tuple(weight + euclidean(x, y, dest_x, dest_y), x, y - 1, x, y));
	}

	pair<int, int> current = path[dest_y][dest_x];
	do
	{
		current = path[current.first][current.second];
		maze.at<Vec3b>(current.first, current.second) = Vec3b(0, 255, 0);
	} while(current != make_pair(start_y, start_x));
}

int main() 
{	
	Mat maze = imread("maze3.png", CV_LOAD_IMAGE_COLOR);
	Mat solve = imread("maze3.png", CV_LOAD_IMAGE_GRAYSCALE);

	if (maze.empty() || solve.empty())
    {
        cerr << "Could not load image" << endl;
        return -1;
    }

    if (!maze.data || !solve.data) 
    {
        cerr << "Empty image?" << solve.data << " " << maze.data << endl;
        return -1;
    }

	depth_first_search(maze, solve);
	imshow("maze", maze);
	waitKey();
	return 0;
}
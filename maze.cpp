#include "maze.h"

double Maze::euclidean(int x, int y, int x2, int y2)
{
	return sqrt(pow((x - x2), 2) + pow((y - y2), 2));
}

double Maze::heuristic(int x, int y, int x2, int y2, Mat &walls)
{
	double wall_density = 0.0;
	const int frame_size = 3;

	for (int i = -frame_size; i < frame_size; i++) 
	{
		for (int j = -frame_size; j < frame_size; j++) 
		{
			if (x < walls.cols && x >= 0 &&
				y < walls.rows && y >= 0)
			{
				// add penalty for having walls nearby
				if (((int)walls.at<unsigned char>(y + i, x + j)) < 128)
				{
					// cout << "penalty: " << 1.0 / (euclidean(x, y, x + j, y + i) / frame_size) + 0.1 << endl;
					wall_density += 1.0 / ((euclidean(x, y, x + j, y + i) / (frame_size * frame_size)) + 0.01);
				}
			}
			else
			{
				wall_density += 3.0;
			}
		}
	}

	// cout << x << " " << y << " " << ((int)walls.at<unsigned char>(x, y)) << " " << wall_density << endl;

	return euclidean(x, y, x2, y2) + wall_density / (double)(frame_size * frame_size);
}

void Maze::depth_first_search(Mat &maze, Mat &solve, int start_x, int start_y, int dest_x, int dest_y)
{
	int finish_x;
	int finish_y;

	Mat walls = solve.clone();

	// Create the backpointer array to find the path
	vector <vector <pair<int, int> > > path;
	path.resize(maze.rows);
	for (int i = 0; i < maze.rows; i++)
	{
		path[i].resize(maze.cols);
	}

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
		if (solve.at<unsigned char>(y, x) < 128) continue;
		
		path[y][x] = make_pair(y_prev, x_prev);
		
		if (abs(x - dest_x) < 3 && abs(y - dest_y) < 3)
		{
			finish_x = x;
			finish_y = y;
			break;
		} 

		solve.at<unsigned char>(y, x) = 0;

		q.push(make_tuple(weight + heuristic(x + 1, y, dest_x, dest_y, walls), x + 1, y, x, y));
		q.push(make_tuple(weight + heuristic(x - 1, y, dest_x, dest_y, walls), x - 1, y, x, y));
		q.push(make_tuple(weight + heuristic(x, y + 1, dest_x, dest_y, walls), x, y + 1, x, y));
		q.push(make_tuple(weight + heuristic(x, y - 1, dest_x, dest_y, walls), x, y - 1, x, y));

		q.push(make_tuple(weight + heuristic(x + 1, y - 1, dest_x, dest_y, walls), x + 1, y - 1, x, y));
		q.push(make_tuple(weight + heuristic(x + 1, y + 1, dest_x, dest_y, walls), x + 1, y + 1, x, y));
		q.push(make_tuple(weight + heuristic(x + 1, y - 1, dest_x, dest_y, walls), x + 1, y - 1, x, y));
		q.push(make_tuple(weight + heuristic(x - 1, y - 1, dest_x, dest_y, walls), x - 1, y - 1, x, y));
	}

	pair<int, int> current = path[finish_y][finish_x];
	do
	{
		current = path[current.first][current.second];
		maze.at<Vec3b>(current.first, current.second) = Vec3b(0, 255, 0);
	} while (current != make_pair(start_y, start_x));
}
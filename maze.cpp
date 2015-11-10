#include "maze.h"

double Maze::euclidean(int x, int y, int x2, int y2)
{
	return sqrt(pow((x - x2), 2) + pow((y - y2), 2));
}

double Maze::heuristic(int x, int y, int x2, int y2, Mat &walls)
{
	// double wall_density = 0.0;
	// const int frame_size = 3;

	// for (int i = -frame_size; i < frame_size; i++) 
	// {
	// 	for (int j = -frame_size; j < frame_size; j++) 
	// 	{
	// 		if (x < walls.cols && x >= 0 &&
	// 			y < walls.rows && y >= 0)
	// 		{
	// 			// add penalty for having walls nearby
	// 			if (((int)walls.at<unsigned char>(y + i, x + j)) < 128)
	// 			{
	// 				// cout << "penalty: " << 1.0 / (euclidean(x, y, x + j, y + i) / frame_size) + 0.1 << endl;
	// 				wall_density += 1.0 / ((euclidean(x, y, x + j, y + i) / (frame_size * frame_size)) + 0.01);
	// 			}
	// 		}
	// 		else
	// 		{
	// 			wall_density += 3.0;
	// 		}
	// 	}
	// }

	// // cout << x << " " << y << " " << ((int)walls.at<unsigned char>(x, y)) << " " << wall_density << endl;

	// return euclidean(x, y, x2, y2) + wall_density / (double)(frame_size * frame_size);
	return euclidean(x, y, x2, y2);
}

void Maze::depth_first_search(Mat &maze, Mat &solve, int start_x, int start_y, int dest_x, int dest_y)
{
	Mat walls = solve.clone();

	// Clear the existing path array
	for (int i = 0; i < path.size(); i++)
	{
		this->path[i].clear();
	}
	this->path.clear();

	// Re-create the path
	this->path.resize(maze.rows);
	for (int i = 0; i < maze.rows; i++)
	{
		this->path[i].resize(maze.cols);
	}

	priority_queue< state, vector<state>, greater<state> > q;

	q.push(make_tuple(0, start_x, start_y, 0, 0));

	this->closest_spot = make_pair(start_y, start_x);
	double closest_distance = 1000000.0;

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
		
		this->path[y][x] = make_pair(y_prev, x_prev);

		double current_dist = euclidean(x, y, dest_x, dest_y);
		
		if (current_dist < closest_distance)
		{
			this->closest_spot.first = y;
			this->closest_spot.second = x;
			closest_distance = current_dist;
		}
		
		if (abs(x - dest_x) < 10 && abs(y - dest_y) < 10)
		{
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
}

pair<int, int> Maze::next_step(int start_x, int start_y)
{
	pair<int, int> current = this->path[this->closest_spot.first][this->closest_spot.second],
				   previous,
				   start_point = make_pair(start_y, start_x);

	double current_dist = 100000000,
		   new_dist;

	pair<int, int> closest_spot;

	do
	{
		previous = current;
		current = this->path[current.first][current.second];

		new_dist = euclidean(start_x, start_y, current.second, current.first);

		if (new_dist <= current_dist) 
		{
			current_dist = new_dist;
			closest_spot = previous;
		}
	} 
	while (current != this->path[current.first][current.second]);

	return closest_spot;
}

void Maze::draw_path(Mat &maze, int start_x, int start_y)
{
	pair<int, int> current = this->path[this->closest_spot.first][this->closest_spot.second],
				   start_point = make_pair(start_y, start_x);

	for (int i = 0; i < 10000; i++) 
	{
		current = this->path[current.first][current.second];
		maze.at<Vec3b>(current.first, current.second) = Vec3b(0, 255, 0);	
	}

	// do
	// {
	// 	current = this->path[current.first][current.second];
	// 	maze.at<Vec3b>(current.first, current.second) = Vec3b(0, 255, 0);
	// } 
	// while (current != this->path[current.first][current.second]);
}
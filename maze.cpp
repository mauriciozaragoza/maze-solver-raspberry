#include "maze.h"

double Maze::euclidean(double x, double y, double x2, double y2)
{
	return sqrt(pow((x - x2), 2) + pow((y - y2), 2));
}

double Maze::manhattan(double x, double y, double x2, double y2)
{
	return abs(x - x2) + abs(y - y2);
}

void Maze::depth_first_search(Mat &maze, Mat &solve, int dest_x, int dest_y)
{
	const int TOLERANCE_SIZE = 10;
	const double WALL_HEURISTIC_ALPHA = 50.0;
	// Mat walls = solve.clone();

	queue<state> q;
	priority_queue<shortest_state, vector<shortest_state>, greater<shortest_state> > q_prime;
	vector< vector<bool> > visited(maze.rows, vector<bool>(maze.cols, false));
	vector< vector<int> > wall_distance(maze.rows, vector<int>(maze.cols, -1));
	this->path = vector< vector< pair<int, int> > >(maze.rows, vector< pair<int, int> >(maze.cols, make_pair(-1, -1)));

	// Compute the wall-distance for each cell
	// Add all cells with wall
	for (int i = 0; i < maze.rows; i++)
	{
		for (int j = 0; j < maze.cols; j++) 
		{
			if (solve.at<unsigned char>(i, j) < 128)
			{
				q.push(make_tuple(j, i, 1, -1));
			}
		}
	}

	// BFS to cover all cells and put their wall-distance value
	while (!q.empty())
	{
		state current = q.front();
		q.pop();

		int x = get<0>(current);
		int y = get<1>(current);
		int current_distance = get<2>(current);

		if (x < 0 || x >= maze.cols || y < 0 || y >= maze.rows) continue;
		if (wall_distance[y][x] != -1) continue;
		
		wall_distance[y][x] = current_distance;

		q.push(make_tuple(x + 1, y, current_distance + 1, -1));
		q.push(make_tuple(x - 1, y, current_distance + 1, -1));
		q.push(make_tuple(x, y + 1, current_distance + 1, -1));
		q.push(make_tuple(x, y - 1, current_distance + 1, -1));
	}

	// ***********************
	// Compute the actual path
	// ***********************

	for (int i = 0; i < TOLERANCE_SIZE; i++) 
	{
		for (int j = 0; j < TOLERANCE_SIZE; j++)
		{
			q_prime.push(make_tuple(0, dest_x + i, dest_y + j, -1, -1));
		}
	}

	while (!q_prime.empty())
	{
		shortest_state current = q_prime.top();
		q_prime.pop();

		double cost = get<0>(current);
		int x = get<1>(current);
		int y = get<2>(current);
		int x_prev = get<3>(current);
		int y_prev = get<4>(current);

		if (x < 0 || x >= maze.cols || y < 0 || y >= maze.rows) continue;
		if (solve.at<unsigned char>(y, x) < 128) continue;
		
		this->path[y][x] = make_pair(x_prev, y_prev);

		solve.at<unsigned char>(y, x) = 0;
		visited[y][x] = true;

		int dist = WALL_HEURISTIC_ALPHA / wall_distance[y][x];

		q_prime.push(make_tuple(cost + dist, x + 1, y, x, y));
		q_prime.push(make_tuple(cost + dist, x - 1, y, x, y));
		q_prime.push(make_tuple(cost + dist, x, y + 1, x, y));
		q_prime.push(make_tuple(cost + dist, x, y - 1, x, y));
	}

	// Second BFS pass through to escape from unreachable places in case it gets stuck
	q = queue<state>();

	// Add already-known places
	for (int i = 0; i < maze.rows; i++)
	{
		for (int j = 0; j < maze.cols; j++) 
		{
			if (visited[i][j])
			{
				q.push(make_tuple(j, i, -1, -1));
			}
		}
	}

	vector< vector<bool> > revisited(maze.rows, vector<bool>(maze.cols, false));

	while (!q.empty())
	{
		state current = q.front();
		q.pop();

		int x = get<0>(current);
		int y = get<1>(current);
		int x_prev = get<2>(current);
		int y_prev = get<3>(current);

		if (x < 0 || x >= maze.cols || y < 0 || y >= maze.rows) continue;
		if (revisited[y][x]) continue;
		revisited[y][x] = true;

		if (!visited[y][x])
		{
			this->path[y][x] = make_pair(x_prev, y_prev);
			visited[y][x] = true;
		}

		q.push(make_tuple(x + 1, y, x, y));
		q.push(make_tuple(x - 1, y, x, y));
		q.push(make_tuple(x, y + 1, x, y));
		q.push(make_tuple(x, y - 1, x, y));
	}
}


pair<int, int> Maze::next_step(int start_x, int start_y, int trail_size)
{
	pair<int, int> current = this->path[start_y][start_x],
				   next;

	for (int i = 0; i < trail_size; i++)
	{
		next = this->path[current.second][current.first];

		if (next.first == -1) break;

		current = next;
	}

	return current;
}

void Maze::draw_path(Mat &maze, int start_x, int start_y)
{
	pair<int, int> current = this->path[start_y][start_x];

	while (current.first != -1)
	{
		current = this->path[current.second][current.first];
		maze.at<Vec3b>(current.second, current.first) = Vec3b(0, 255, 0);
	}
}
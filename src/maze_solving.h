#ifndef MAZE_SOLVING_H
#define MAZE_SOLVING_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <queue>
#include <tuple>
#include "utils.h"
#include "constants.h"


////////////////// PROGRAM //////////////////

std::array<GridSquare, GRID_H* GRID_W * 2> tmp = {};
// WARNING: Clobbers `tmp`
int load_neighbors(GridSquare s) {
	int e = 0;
	for (const GridSquare& o : N_OFFSETS) {
		GridSquare n = s + o;
		bool good = n.isvalid();
		if (good) {
			for (const Edge& w : WALLS) {
				if (((w.a == s) && (w.b == n)) || ((w.b == s) && (w.a == n))) {
					good = false;
					break;
				}
			}
		}

		if (good) {
			tmp[e++] = n;
		}
	}
	return e;
}

class PathFinding {
public:
	static const int N_NODES = GRID_W * GRID_H;
	// TODO: These are symmetric so that could be used as a memory optimization!
	// Could use one half as prev and the other half as dist!
	std::array<std::array<int, N_NODES>, N_NODES> dist = {};
	std::array<std::array<int, N_NODES>, N_NODES> prev = {};
	// Only has space for the goals
	std::array<GridSquare, GOALS.max_size()> waypoints;

	// TODO: Maybe it would be more memory efficient to use multiple BFS-s?
	// IDEA: If there is some way to rank paths based on the amount of turns
	// or some other heuristic that takes into account the actual time the
	// robot will take to traverse the path, that could be added as
	// weights/sorting keys
	void calculate() {
		// Use floyd-warshall for distances and paths

		std::fill_n(&dist[0][0], dist.size() * dist[0].size(), INT32_MAX / 2);
		std::fill_n(&prev[0][0], prev.size() * prev[0].size(), -1);


		// for (int i = 0; i < prev.size(); ++i) {
		// 	for (int j = 0; j < prev[i].size(); ++j)
		// 		prev[i][j] = -1;
		// }

		// for (int i = 0; i < dist.size(); ++i) {
		// 	for (int j = 0; j < dist[i].size(); ++j)
		// 		dist[i][j] = INT32_MAX / 2;
		// }

		for (int i = 0; i < N_NODES; ++i) {
			dist[i][i] = 0;
			prev[i][i] = i;
			int n = load_neighbors(GridSquare(i));

			for (int b = 0; b < n; ++b) {
				dist[i][tmp[b].to_i()] = 1; // TODO: Since this is always 1, there
				// might be some way to optimize this?
				prev[i][tmp[b].to_i()] = i;
			}
		}

		// TODO: Optimize for no weights!
		for (int k = 0; k < N_NODES; ++k) {
			for (int j = 0; j < N_NODES; ++j) {
				for (int i = 0; i < N_NODES; ++i) {
					int via_k = dist[i][k] + dist[k][j];
					if (dist[i][j] > via_k) {
						dist[i][j] = via_k;
						prev[i][j] = prev[k][j];
					}
				}
			}
		}

		std::cout << "calculate() completed" << std::endl;
	}

	// void print_dists() {
	// 	for (int i = 0; i < N_NODES; ++i) {
	// 		for (int j = 0; j < N_NODES; ++j) {
	// 			std::cout << GridSquare(i).to_string() << "->"
	// 				<< GridSquare(j).to_string() << " = " << dist[i][j]
	// 				<< std::endl;
	// 		}
	// 	}
	// }

	// N is very   small so using the O(n!) algorithm is good enough, especially
	// since it finds the ideal solution rather than a good-enough one! N <= 3
	void optimize_routes() {
		std::array<GridSquare, GOALS.max_size()> wpts_tmp;
		int min_dist = INT32_MAX;

		// Go through all permutations of the goals
		std::copy(GOALS.begin(), GOALS.end(), wpts_tmp.begin());
		std::sort(wpts_tmp.begin(), wpts_tmp.end());

		// int nperm = 0;
		do {
			// nperm += 1;
			// Calculate distance
			// (START -> ... -> TARGET)
			int d = 0;
			int pvi = START.to_i();
			for (const GridSquare& wpt : wpts_tmp) {
				d += dist[pvi][wpt.to_i()];
				pvi = wpt.to_i();
			}
			d += dist[pvi][TARGET.to_i()];

			// Keep track of min
			if (d < min_dist) {
				min_dist = d;
				waypoints = wpts_tmp;
			}
		} while (std::next_permutation(wpts_tmp.begin(), wpts_tmp.end()));

		std::cout << "optimize_routes() completed" << std::endl;
	}

	// WARNING: Clobbers `tmp`
	int load_path(GridSquare from, GridSquare to, int o = 0) {
		// TODO: Could be optimized by starting at an index determined by
		// distance... eh? then decrement...
		int n = o;

		int u = from.to_i(), v = to.to_i();

		if (prev[u][v] == -1) {
			return 0;
		}
		tmp[n++] = GridSquare(v);

		while (u != v) {
			v = prev[u][v];
			tmp[n++] = GridSquare(v);
			if (n >= tmp.size())
				return -1; // ERROR!
		}

		// Reverse-in-place
		std::reverse(tmp.begin() + o, tmp.begin() + n);
		std::cout << "load_path() completed" << std::endl;

		// tmp[n] = to;
		return n;
	}

	// WARNING: Clobbers `tmp`
	int get_full_path() {
		int n = 0;
		GridSquare prev = START;
		for (GridSquare& sq : waypoints) {
			// Add the path that needs to be travelled to get from the prev. square to
			// the waypoint
			// Sub 1 from n so that there aren't any duplicate positions.
			n = load_path(prev, sq, n) - 1;
			prev = sq;
			if (n < 0)
				return n;
		}
		// Finish up by going to the end
		n = load_path(prev, TARGET, n);
		std::cout << "get_full_path() completed" << std::endl;

		return n;
	}
};

PathFinding p;

// // TODO: Proper array bounds checking!
// int main() {
// 	p.calculate();
// 	p.optimize_routes();

// 	// printf("Dist: %d\n", p.dist[GridSquare(1, 0).to_i()][TARGET.to_i()]);
// 	// std::cout << "Waypoints (ordered):" << std::endl;
// 	// for (GridSquare &sq : p.waypoints) {
// 	//   std::cout << sq.to_string() << std::endl;
// 	// }

// 	int n = p.get_full_path(); // p.load_path(START, p.waypoints[0]);
// 	printf("Took %d steps\n", n);
// 	for (int i = 0; i < n; ++i) {
// 		std::cout << tmp[i].to_string() << std::endl;
// 	}

// 	ASSERT_PANIC(false)

// 		return 0;
// }

#endif
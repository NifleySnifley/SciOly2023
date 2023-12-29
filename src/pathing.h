#ifndef PATHING_H
#define PATHING_H

#include <cmath>
#include <algorithm>
#include "utils.h"
#include <vector>
#define SPLINE_SEGMENT_DIVISIONS 16

class BSplinePath {
public:
	std::vector<Vec2> waypoints;
	std::vector<Vec2> pathpoints;
	std::vector<int> cusp_indices; // Points where the curve has a cusp (robot turns 180)
	std::vector<float> dist_presum;

	BSplinePath(std::vector<Vec2> wpts) {
		ASSERT_PANIC(wpts.size() >= 4);

		waypoints.push_back(wpts[0] - (wpts[1] - wpts[0]));
		for (Vec2 v : wpts) {
			waypoints.push_back(v);
		}
		waypoints.push_back(wpts[wpts.size() - 1] + (wpts[wpts.size() - 1] - wpts[wpts.size() - 2]));
	}

	float total_distance() {
		return dist_presum.size() ? dist_presum[dist_presum.size() - 1] : 0.0f;
	};

	// TODO: Find a way to actually optimize using last guess instead of just cacheing...
	size_t find_prev_index(float distance_mm) {
		static size_t last_result = 0;

		if (distance_mm < 0) {
			return -1;
		} else if (distance_mm > total_distance()) {
			return -1;
		}

		size_t li = clamp((size_t)0, pathpoints.size() - 1, last_result);
		size_t ri = clamp((size_t)0, pathpoints.size() - 1, last_result + 1);
		if (distance_mm >= dist_presum[li] && distance_mm <= dist_presum[ri]) {
			return last_result;
		}

		size_t left = 0;
		size_t right = pathpoints.size() - 1;
		size_t middle = -1;

		while (left <= right) {
			middle = (left + right) / 2; // HACK: opt to bitshift
			if (dist_presum[middle] < distance_mm) {
				left = middle + 1;
			} else if (dist_presum[middle] > distance_mm) {
				right = middle - 1;
			} else {
				// `middle` is success
				break;
			}
		}

		if (dist_presum[middle] > distance_mm && middle >= 1) --middle;

		last_result = middle;

		return middle;
	}

	Vec2 point_at_distance(float distance_mm) {
		if (distance_mm >= total_distance()) {
			return pathpoints[pathpoints.size() - 1];
		} else if (distance_mm <= 0) {
			return pathpoints[0];
		}

		size_t pi = find_prev_index(distance_mm);

		// std::cout << middle << std::endl;
		// std::cout << "Target: " << distance_mm << ", Real: " << dist_presum[middle] << std::endl;

		float seglen = dist_presum[pi + 1] - dist_presum[pi];
		return Vec2::lerp(pathpoints[pi], pathpoints[pi + 1], (distance_mm - dist_presum[pi]) / seglen);
	}

	/// WARNING: Does NOT normalize!
	Vec2 dxy_at_distance(float distance_mm) {
		if (distance_mm >= total_distance()) {
			return pathpoints[pathpoints.size() - 1] - pathpoints[pathpoints.size() - 2];
		} else if (distance_mm <= 0) {
			return pathpoints[1] - pathpoints[0];
		}

		size_t pi = find_prev_index(distance_mm);

		// std::cout << middle << std::endl;
		// std::cout << "Target: " << distance_mm << ", Real: " << dist_presum[middle] << std::endl;
		return pathpoints[pi + 1] - pathpoints[pi];
	}


	void polyline_ize_spline() {
		float a[4] = { 0 };
		float b[4] = { 0 };
		float dy = 0.0f;
		float dx = 0.0f;
		float size = 0.0f;

		Vec2 currentPoint(0.0f);
		Vec2 nextPoint(0.0f);

		for (int i = 0; i < (waypoints.size() - 3); i++) {
			float t = 0.0f;
			Vec2 p1 = waypoints[i], p2 = waypoints[i + 1], p3 = waypoints[i + 2], p4 = waypoints[i + 3];

			a[0] = (-p1.x + 3.0f * p2.x - 3.0f * p3.x + p4.x) / 6.0f;
			a[1] = (3.0f * p1.x - 6.0f * p2.x + 3.0f * p3.x) / 6.0f;
			a[2] = (-3.0f * p1.x + 3.0f * p3.x) / 6.0f;
			a[3] = (p1.x + 4.0f * p2.x + p3.x) / 6.0f;

			b[0] = (-p1.y + 3.0f * p2.y - 3.0f * p3.y + p4.y) / 6.0f;
			b[1] = (3.0f * p1.y - 6.0f * p2.y + 3.0f * p3.y) / 6.0f;
			b[2] = (-3.0f * p1.y + 3.0f * p3.y) / 6.0f;
			b[3] = (p1.y + 4.0f * p2.y + p3.y) / 6.0f;

			currentPoint.x = a[3];
			currentPoint.y = b[3];

			for (int j = 1; j <= SPLINE_SEGMENT_DIVISIONS; j++) {
				t = ((float)j) / ((float)SPLINE_SEGMENT_DIVISIONS);

				nextPoint.x = a[3] + t * (a[2] + t * (a[1] + t * a[0]));
				nextPoint.y = b[3] + t * (b[2] + t * (b[1] + t * b[0]));

				if ((i == 0) && (j == 1)) {
					pathpoints.push_back(currentPoint);
				}

				pathpoints.push_back(nextPoint);

				currentPoint = nextPoint;
			}
		}
	}

	void calculate() {
		polyline_ize_spline();

		dist_presum.resize(pathpoints.size());

		cusp_indices.push_back(0);
		for (int i = 1; i < pathpoints.size(); ++i) {
			// Check for cusp
			if (i != pathpoints.size() - 1) {
				Vec2 dir_in = pathpoints[i] - pathpoints[i - 1];
				Vec2 dir_out = pathpoints[i + 1] - pathpoints[i];
				// Greater than 90deg turn
				if (Vec2::dot(dir_in, dir_out) <= 0.0) {
					cusp_indices.push_back(i);
				}
			}

			float seglen = Vec2::distance(pathpoints[i - 1], pathpoints[i]);

			// FIXME: Just for debug!! uses sqrt!!!
			dist_presum[i] = dist_presum[i - 1] + seglen;
		}

		cusp_indices.push_back(pathpoints.size() - 1);

		// std::cout << "Total path distance (mm): " << total_distance() << std::endl;
	}
};

#endif
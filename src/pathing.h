#ifndef PATHING_H
#define PATHING_H

#include <cmath>
#include "utils.h"
#include <vector>
#define SPLINE_SEGMENT_DIVISIONS 32

int three() {
	return 3;
}

class Path {
public:
	std::vector<Vec2> waypoints;
	std::vector<Vec2> pathpoints;
	float total_distance = 0.0f;

	Path(Vec2* points, int n) {
		for (int i = 0; i < n;++i)
			waypoints.push_back(points[n]);
	}

	Path(std::vector<Vec2> wpts) {
		ASSERT_PANIC(wpts.size() >= 4);

		waypoints.push_back(wpts[0] - (wpts[1] - wpts[0]));
		for (Vec2 v : wpts) {
			waypoints.push_back(v);
		}
		waypoints.push_back(wpts[wpts.size() - 1] + (wpts[wpts.size() - 1] - wpts[wpts.size() - 2]));

	}

	Vec2 point_at_t(float t) {
		return Vec2();
	}

	void rasterize_spline() {
		// if (pointCount < 4) return;

		float a[4] = { 0 };
		float b[4] = { 0 };
		float dy = 0.0f;
		float dx = 0.0f;
		float size = 0.0f;

		Vec2 currentPoint(0.0f);
		Vec2 nextPoint(0.0f);
		// Vec2 vertices[2 * SPLINE_SEGMENT_DIVISIONS + 2] = { 0 };

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

			// if (i == 0) DrawCircleV(currentPoint, thick / 2.0f, color);   // Draw init line circle-cap
			// if (i == 0) pathpoints.push_back(currentPoint);   // Draw init line circle-cap

			// if (i > 0) {
				// pathpoints.push_back(currentPoint);
				// vertices[0].x = currentPoint.x + dy * size;
				// vertices[0].y = currentPoint.y - dx * size;
				// vertices[1].x = currentPoint.x - dy * size;
				// vertices[1].y = currentPoint.y + dx * size;
			// }

			for (int j = 1; j <= SPLINE_SEGMENT_DIVISIONS; j++) {
				t = ((float)j) / ((float)SPLINE_SEGMENT_DIVISIONS);

				nextPoint.x = a[3] + t * (a[2] + t * (a[1] + t * a[0]));
				nextPoint.y = b[3] + t * (b[2] + t * (b[1] + t * b[0]));

				// dy = nextPoint.y - currentPoint.y;
				// dx = nextPoint.x - currentPoint.x;
				// size = 0.5f * thick / sqrtf(dx * dx + dy * dy);

				if ((i == 0) && (j == 1)) {
					pathpoints.push_back(currentPoint);
					// vertices[0].x = currentPoint.x + dy * size;
					// vertices[0].y = currentPoint.y - dx * size;
					// vertices[1].x = currentPoint.x - dy * size;
					// vertices[1].y = currentPoint.y + dx * size;
				}

				// vertices[2 * j + 1].x = nextPoint.x - dy * size;
				// vertices[2 * j + 1].y = nextPoint.y + dx * size;
				// vertices[2 * j].x = nextPoint.x + dy * size;
				// vertices[2 * j].y = nextPoint.y - dx * size;
				pathpoints.push_back(nextPoint);

				currentPoint = nextPoint;
			}

			// DrawTriangleStrip(vertices, 2 * SPLINE_SEGMENT_DIVISIONS + 2, color);
		}

		// DrawCircleV(currentPoint, thick / 2.0f, color);   // Draw end line circle-cap
	}

	void calculate() {
		// for (Vec2 p : waypoints) {
		// 	pathpoints.push_back(p);
		// }

		rasterize_spline();

		// FIXME: Just for debug!!
		for (int i = 1; i < pathpoints.size(); ++i) {
			total_distance += Vec2::distance(pathpoints[i - 1], pathpoints[i]);
		}
		std::cout << "Total path distance (mm): " << total_distance << std::endl;
	}
};

// class BezierCurve {

// }

// class Trajectory {

// }

#endif
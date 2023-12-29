#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef DEBUG_GUI
extern "C" {
#include "raylib.h"
}
#include "debug.h"
#endif

#include "pathing.h"
#include "utils.h"

// TODO: TUNE
const float TURN_P = 1.0f;

class PursuitController {
public:
	BSplinePath* path;
	Odometry* odom;
	MotorController* m_left, * m_right;
	float target_distance = LOOKAHEAD_DIST;

	// FIXME: Calibrate for optimal precision to the end goal!

	// Pursuit controller parameters
	const float THRESHOLD = 5.0f;
	const float THRESHOLD_SQ = THRESHOLD * THRESHOLD;

	const float LOOKAHEAD_DIST = 75.0f; // 100mm
	const float LOOKAHEAD_DIST_SQ = LOOKAHEAD_DIST_SQ * LOOKAHEAD_DIST_SQ; // 100mm

	const float END_THRESHOLD = 80.0f;
	const float END_THRESHOLD_SQ = END_THRESHOLD * END_THRESHOLD;

	// Throttle ramping (interpolation) control
	const float MAX_THROTTLE = 1.0f;
	const float MIN_THROTTLE = 0.25f;
	const float SLOWDOWN_THRESH = 400.f;

	PursuitController(BSplinePath* path, Odometry* odom, MotorController* left, MotorController* right) : path(path), odom(odom), m_left(left), m_right(right) {
		reset();
	}


	void reset() {
		target_distance = LOOKAHEAD_DIST;
		odom->reset_odometry();
		odom->pose = {
			path->dxy_at_distance(0.0f).direction_radians(),
			START_POS.x,
			START_POS.y
		};
		m_left->stop();
		m_right->stop();
	}

	float calculate_throttle() {
		float dist = 1e20;
		for (int i : path->cusp_indices) {
			dist = std::min(dist, std::abs(path->dist_presum[i] - (target_distance - LOOKAHEAD_DIST)));
		}

		return clamp(MIN_THROTTLE, MAX_THROTTLE, dist / SLOWDOWN_THRESH);
	}

	/// Returns `true` on command completion
	bool execute(Vec2 _debug) {
		Vec2 target_point = path->point_at_distance(target_distance);
		float tgt_dist = Vec2::distance(target_point, cur_pos());

		while (tgt_dist <= LOOKAHEAD_DIST && target_distance <= path->total_distance()) {
			target_distance += std::max(1.f, LOOKAHEAD_DIST - tgt_dist);
			target_point = path->point_at_distance(target_distance);
			tgt_dist = Vec2::distance(target_point, cur_pos());
		}

#ifdef DEBUG_GUI
		DrawCircleLinesV(robot2screenspace(cur_pos()), LOOKAHEAD_DIST * ROBOT_2_SCREEN, BLUE);
		DrawCircleV(robot2screenspace(target_point), 5.0f, BLACK);
#endif

		drive_towards_point(target_point, calculate_throttle());

		if (target_distance >= path->total_distance() && (cur_pos() - target_point).length_squared() <= END_THRESHOLD_SQ) {
			m_left->stop();
			m_right->stop();
			return true;
		}

		return false;
	}

	Vec2 cur_pos() {
		return Vec2(odom->pose.x, odom->pose.y);
	}

	bool drive_towards_point(Vec2 point, float throttle) {
#ifdef DEBUG_GUI
		DrawLineEx(robot2screenspace(cur_pos()), robot2screenspace(point), 2.0f, BLACK);
#endif

		if ((point - cur_pos()).length_squared() <= THRESHOLD_SQ) {
			return true;
		}

		float angle_tgt = (point - cur_pos()).direction_radians();
		float cur_heading = fmodf(odom->pose.rotation, _2PI);

		float error = (angle_tgt - cur_heading);
		if (error > PI || error < -PI) {
			error = -sign(error) * (_2PI - fabsf(error));
		}

		float dot = cosf(error);

		float turncommand = error * TURN_P;
		m_left->set(clamp(-1.0f, 1.0f, (-turncommand + dot * dot)) * throttle);
		m_right->set(clamp(-1.0f, 1.0f, (turncommand + dot * dot)) * throttle);
		return false;
	}
};

#endif
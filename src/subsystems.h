#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H
#include "utils.h"
#include <cmath>
#include "constants.h"

// Odometry, etc.
class Odometry {
	Encoder* el, * er;

	float last_el, last_er;
	// uint64_t _last_ts;
public:
	pose_t pose;

	Odometry(
		Encoder* e_left, Encoder* e_right, pose_t init_pose
	) : el(e_left), er(e_right), pose(init_pose) {
		last_el = last_er = 0.0f;
		el->set(0.f);
		er->set(0.f);
		// _last_ts = time_us_64();

		reset_odometry();
	}

	void reset_odometry() {
		last_el = el->get();
		last_er = er->get();
	}

	// FIXME: These are the "correct" trigonometric odometry calculations
	// Using the linear approximation might improve performance since the rp2040 
	// doesn't have a FPU
	void update_odometry() {
		float cer = er->get();
		float cel = el->get();
		float dr = cer - last_er;
		float dl = cel - last_el;
		last_er = cer;
		last_el = cel;

		float r = 0.0f; // MM
		bool straight = false; // Flag for if dl == dr
		if (dl == dr) {
			straight = true;
		} else {
			r = (WHEELBASE_2 * (dl + dr)) / (dl - dr);
		}

		float theta = 0.0f; // Radians
		float dx, dy; // MM

		if (straight) {
			dx = 0.0f; // or dl, just moved forward
			dy = dr;
		} else {
			theta = (dl == 0.0f) ? (dr / (r - WHEELBASE_2)) : (dl / (r + WHEELBASE_2));

			dx = cosf(theta) * r - r;
			dy = sinf(theta) * r;
		}

		// dx and dy are relative to the current pose (dy forwards, etc.)
		// transform them and add to pose
		float p_angle = pose.rotation - _PI2;
		float dx_pose_space = dx * cosf(p_angle) - dy * sinf(p_angle);
		float dy_pose_space = dx * sinf(p_angle) + dy * cosf(p_angle);

		// Update the pose!
		pose.rotation -= theta;
		pose.x += dx_pose_space;
		pose.y += dy_pose_space;
	}
};

#endif
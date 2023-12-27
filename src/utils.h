#ifndef UTILS_H
#define UTILS_H

#include <bit>
#include <limits>
#include <cstdint>
#include <cmath>

const int GRID_W = 4;
const int GRID_H = 4;

const float GRID_SIZE_MM = 2000.f;
const float GRIDSQUARE_SIZE_MM = GRID_SIZE_MM / GRID_W;

#ifdef PICO_FLASH_SIZE_BYTES
#include "pico/time.h"
#endif

template<typename T> T clamp(T mi, T ma, T val) {
	return (val > ma) ? ma : ((val < mi) ? mi : val);
}


// TODO: Create a macro that is functionally similar to `assert` (halts program,
// sets the red LED, etc.)
#define ASSERT_PANIC(cond)                                                     \
  if (!(cond)) {                                                               \
    printf("PANIC at %s:%d in `%s`\n", __FILE__, __LINE__, __func__);          \
    panic();                                                                   \
  }

void panic() {
	// while (1) {
	// }
	exit(1);
}

float lerp(float a, float b, float t) {
	return a + (b - a) * clamp(0.f, 1.f, t);
}

typedef struct pose_t {
	float rotation;
	float x;
	float y;
} pose_t;

// Fast inverse square root
float q_rsqrt(float number) {
	union {
		float    f;
		uint32_t i;
	} conv;
	conv.f = number;
	conv.i = 0x5f3759df - (conv.i >> 1);
	conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
	return conv.f;
}


union Vec2 {
public:
	float components[2];
	struct {
		float x;
		float y;
	};

	Vec2(float x, float y) : x(x), y(y) {}
	Vec2(float c) : x(c), y(c) {}
	Vec2() : x(0.f), y(0.f) {}

	static Vec2 from_polar(float theta, float r) {
		return Vec2(cosf(theta) * r, sinf(theta) * r);
	}

	static float distance(Vec2& a, Vec2& b) {
		return (a - b).norm();
	}

	Vec2 normalize() {
		float len_1 = q_rsqrt(length_squared());
		return Vec2(len_1 * x, len_1 * y);
	}

	float length_squared() {
		return (x * x) + (y * y);
	}

	// WARNING: Uses `sqrtf()`!
	// This function should be avoided at all costs
	float norm() {
		return 1.0f / q_rsqrt(length_squared());
	}

	float direction_radians() {
		return atan2f(y, x);
	}

	Vec2 operator+(Vec2& other) {
		return Vec2(x + other.x, y + other.y);
	}

	Vec2 operator-(Vec2& other) {
		return Vec2(x - other.x, y - other.y);
	}

	Vec2 operator*(Vec2& other) {
		return Vec2(x * other.x, y * other.y);
	}

	Vec2 operator*(float scalar) {
		return Vec2(x * scalar, y * scalar);
	}

	Vec2 operator/(float scalar) {
		float s_inv = 1.f / scalar;
		return *this * s_inv;
	}
};

#ifdef PICO_FLASH_SIZE_BYTES
float time_seconds() {
	return (float)(time_us_64()) * (1e-6);
}

class PIDController {
	float kP, kI, kD;

	float _integral = 0.f;
	uint64_t _last_ts;
	bool _fresh;
	float _last_e = 0.f;
	float _setpoint = 0.f;
public:
	PIDController(float P, float I, float D) : kP(P), kI(I), kD(D) {
		reset();
	}

	void reset() {
		_integral = 0;
		_fresh = true;
	}

	float calculate(float measurement) {
		float e = (_setpoint - measurement);

		uint64_t now_ts = time_us_64();
		float dt = (float)(now_ts - _last_ts) * (1e-6);

		float de_dt = (e - _last_e) / dt;
		_integral += dt * e;

		_last_ts = now_ts;
		_last_e = e;

		if (_fresh) {
			_fresh = false;
			return kP * e; // Only do proportional, D and I rely in prior conditions
		} else {
			return kP * e + kD * de_dt + kI * _integral;
		}
	}

	float calculate(float measurement, float setpoint) {
		_setpoint = setpoint;
		return calculate(measurement);
	}
};
#endif


typedef struct GridSquare {
public:
	int x, y;

	GridSquare() : x(-1), y(-1) {}
	GridSquare(int x, int y) : x(x), y(y) {}
	friend GridSquare operator+(GridSquare a, const GridSquare& b) {
		return GridSquare(a.x + b.x, a.y + b.y);
	}
	friend bool operator==(const GridSquare& a, const GridSquare& b) {
		return (a.x == b.x) && (a.y == b.y);
	}

	friend bool operator!=(const GridSquare& a, const GridSquare& b) {
		return (a.x != b.x) || (a.y != b.y);
	}

	friend bool operator<(const GridSquare& a, const GridSquare& b) {
		return a.to_i() < b.to_i();
	}

	friend bool operator>(const GridSquare& a, const GridSquare& b) {
		return a.to_i() > b.to_i();
	}

	GridSquare(int i) {
		x = i % GRID_H;
		y = i / GRID_H;
	}

	// std::string to_string() {
	// 	return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
	// }

	int to_i() const { return this->x + this->y * GRID_H; }

	bool isvalid() {
		return (x >= 0) && (x < GRID_W) && (y >= 0) && (y < GRID_H);
	}

	// FIXME: Check grid square size!
	Vec2 bottom_left_on_field_mm() {
		return Vec2(500.f * x, 500.f * y);
	}

	const Vec2 bottom_left_on_field_mm() const {
		return Vec2(500.f * x, 500.f * y);
	}
} GridSquare;

typedef struct Edge {
	GridSquare a, b;
	Edge(GridSquare a, GridSquare b) : a(a), b(b) {}
	Edge(int x1, int y1, int x2, int y2)
		: a(GridSquare(x1, y1)), b(GridSquare(x2, y2)) {
	}

} Edge;


#endif
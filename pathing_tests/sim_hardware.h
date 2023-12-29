#ifndef SIM_HARDWARE_H
#define SIM_HARDWARE_H

#include <algorithm>
#include "constants.h"

class MotorController {
public:
	float _throttle = 0.0f;
	float _position_rot = 0.0f;

	MotorController(
	) {
	}

	void set(float throttle) {
		_throttle = std::max(-1.0f, std::min(1.0f, throttle));
	}

	void set(bool forwards, float throttle) {
		set(throttle * (forwards ? 1.0f : -1.0f));
	}

	void stop() {
		_throttle = 0.0f;
	}

	void _simulate(float timedelta) {
		_position_rot += _throttle * timedelta * (MAX_RPM / 60.0f);
	}
};

class Encoder {
	float _offset = 0.0f;
	MotorController* _motor = nullptr;

public:
	Encoder(MotorController* motor) : _motor(motor) {
		set(0.0f);
	}

	float get() {
		return _motor->_position_rot * WHEEL_ROT2MM_FAC - _offset;
	}

	void set(float pos) {
		_offset = _motor->_position_rot * WHEEL_ROT2MM_FAC - pos;
	}
};
#endif
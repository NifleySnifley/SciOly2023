#ifndef HARDWARE_H
#define HARDWARE_H

#include "utils.h"
#include "constants.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "quad.pio.h"

class PWMPinWrapper {
	int pin;
	unsigned int slice;
	unsigned int chan;
public:
	PWMPinWrapper(int pin) : pin(pin) {
		slice = pwm_gpio_to_slice_num(pin);
		chan = pwm_gpio_to_channel(pin);
		gpio_set_function(pin, GPIO_FUNC_PWM);

		pwm_config cfg = pwm_get_default_config();
		pwm_config_set_clkdiv(&cfg, SYS_CLK_KHZ / MOTOR_FREQ);
		pwm_config_set_wrap(&cfg, MOTOR_CYCLES);

		pwm_init(slice, &cfg, true);
		set_raw(0);
	}

	/**
	 * @brief Set the PWM's compare value
	 * @param raw Raw compare value, `0` to `MOTOR_CYCLES`
	 */
	void set_raw(int raw) {
		pwm_set_chan_level(slice, chan, raw);
	}

	/**
	 * @brief Sets the PWM duty cycle
	 * @param throttle Duty cycle from 0 to 1
	**/
	void set(float duty) {
		set_raw(clamp(0, MOTOR_CYCLES, (int)(duty * (float)MOTOR_CYCLES)));
	}

	void set_bool(bool val) {
		set_raw(val ? 0 : MOTOR_CYCLES);
	}
};

class MotorController {
	PWMPinWrapper pin1, pin2;
	float _t_mult = 1.0;
	float _ff = 0.0f;

public:
	MotorController(
		int pin_1, int pin_2,
		float feedforward = 0.0f
	) : pin1(pin_1), pin2(pin_2), _ff(feedforward) {
		_t_mult = 1.0f / (1.0f - feedforward);
	}

	void set(float throttle) {
		set(throttle < 0.0f, fabsf(throttle) * _t_mult + _ff);
	}

	void set(bool forwards, float throttle) {
		if (throttle == 0.0) stop();
		else if (forwards) {
			pin1.set(1.0 - throttle);
			pin2.set(1.0);
		} else {
			pin2.set(1.0 - throttle);
			pin1.set(1.0);
		}
	}

	void stop() {
		pin1.set_bool(true);
		pin2.set_bool(true);
	}
};

class Encoder {
	PIO pio = pio0;
	unsigned int sm;
	int pin_A, pin_B;

	float _fac = 1.0;
public:
	Encoder(int A, int B, float conv_fac = 1.0f) : pin_A(A), pin_B(B), _fac(conv_fac) {
		unsigned int off = pio_add_program(pio, &quadratureA_program);
		sm = pio_claim_unused_sm(pio, true);
		quadratureA_program_init(pio, sm, off, pin_A, pin_B);
	}

	// TODO: Asynchronous reading (DMA) and then just return an instance variable!
	int32_t get_raw() {
		pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
		return (int32_t)pio_sm_get_blocking(pio, sm);
	}

	void set_raw(int32_t pos) {
		pio_sm_exec(pio, sm, pio_encode_set(pio_x, (uint32_t)pos));
	}

	float get() {
		return (float)get_raw() * _fac;
	}

	void set(float pos) {
		set_raw((int32_t)(pos / _fac));
	}
};

#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS 0x212
// TODO: Put CE up on one, change I2C address, 
class VL6180X {
	i2c_inst_t* bus;
	int pin_CE;

public:
	uint8_t addr = 0x29;

	VL6180X(
		i2c_inst_t* i2c,
		int CE,
		int address = 0x29
	) : pin_CE(CE), bus(i2c), addr(address) {
		gpio_init(pin_CE);
		gpio_set_dir(pin_CE, GPIO_OUT);
	}

	void power(bool pow) {
		gpio_put(pin_CE, pow);
	}

	uint16_t frac16_encode(float v) {
		return (uint8_t)roundf(v * (1 << 7));
	}

	uint8_t frac8_encode(float v) {
		return (uint8_t)roundf(v * (1 << 4));
	}

	float frac8_decode(uint8_t v) {
		return (float)v / (float)(1 << 4);
	}

	float frac16_decode(uint16_t v) {
		return (float)v / (float)(1 << 7);
	}

	void reg_write8(uint16_t regn, uint8_t val) {
		uint8_t buf[] = {
			(uint8_t)(regn >> 8),
			(uint8_t)(regn >> 0),
			val
		};
		i2c_write_blocking(bus, addr, buf, sizeof(buf), false);
	}

	void reg_write16(uint16_t regn, uint16_t val) {
		uint8_t buf[] = {
				   (uint8_t)(regn >> 8),
					(uint8_t)(regn & 0xFF),
				   (uint8_t)((val & 0xFF00) >> 8),
				   (uint8_t)(val & 0xFF)
		};
		i2c_write_blocking(bus, addr, buf, sizeof(buf), false);
	}

	uint8_t reg_read8(uint16_t regn) {
		uint8_t txbuf[] = {
			(uint8_t)(regn >> 8),
			(uint8_t)(regn & 0xFF)
		};
		uint8_t rxbuf[] = { 0x00 };

		i2c_write_blocking(bus, addr, txbuf, sizeof(txbuf), true);
		i2c_read_blocking(bus, addr, rxbuf, sizeof(rxbuf), false);
		return rxbuf[0];
	}

	uint16_t reg_read16(uint16_t regn) {
		uint8_t txbuf[] = { (uint8_t)(regn >> 8),
			(uint8_t)(regn & 0xFF) };
		uint8_t rxbuf[] = { 0x00,0x00 };

		i2c_write_blocking(bus, addr, txbuf, sizeof(txbuf), true);
		i2c_read_blocking(bus, addr, rxbuf, sizeof(rxbuf), false);
		return (uint16_t)(rxbuf[0]) || ((uint16_t)rxbuf[1] << 8);
	}

	void init() {
		reg_write8(0x207, 0x01);
		reg_write8(0x208, 0x01);
		reg_write8(0x096, 0x00);
		reg_write8(0x097, 0xFD); // RANGE_SCALER = 253
		reg_write8(0x0E3, 0x01);
		reg_write8(0x0E4, 0x03);
		reg_write8(0x0E5, 0x02);
		reg_write8(0x0E6, 0x01);
		reg_write8(0x0E7, 0x03);
		reg_write8(0x0F5, 0x02);
		reg_write8(0x0D9, 0x05);
		reg_write8(0x0DB, 0xCE);
		reg_write8(0x0DC, 0x03);
		reg_write8(0x0DD, 0xF8);
		reg_write8(0x09F, 0x00);
		reg_write8(0x0A3, 0x3C);
		reg_write8(0x0B7, 0x00);
		reg_write8(0x0BB, 0x3C);
		reg_write8(0x0B2, 0x09);
		reg_write8(0x0CA, 0x09);
		reg_write8(0x198, 0x01);
		reg_write8(0x1B0, 0x17);
		reg_write8(0x1AD, 0x00);
		reg_write8(0x0FF, 0x05);
		reg_write8(0x100, 0x05);
		reg_write8(0x199, 0x05);
		reg_write8(0x1A6, 0x1B);
		reg_write8(0x1AC, 0x3E);
		reg_write8(0x1A7, 0x1F);
		reg_write8(0x030, 0x00);
		reg_write8(0x016, 0x00);

		// readout__averaging_sample_period = 48
		reg_write8(0x10A, 0x30);
		// sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to table 
		// "Actual gain values" in datasheet)
		reg_write8(0x03F, 0x46);
		// sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration 
		// after every 255 range measurements)
		reg_write8(0x031, 0xFF);
		// sysals__integration_period = 99 (100 ms)
		reg_write16(0x040, 0x0063);
		// sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
		reg_write8(0x02E, 0x01);
		// sysrange__intermeasurement_period = 0 (10 ms)
		reg_write8(0x03E, 0x00);
		// sysals__intermeasurement_period = 49 (500 ms)
		reg_write8(0x03E, 0x31);
		// als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 
		// (range new sample ready interrupt)
		reg_write8(0x014, 0x24);
		// Reset other settings to power-on defaults
		// sysrange__max_convergence_time = 49 (49 ms)
		reg_write8(0x01C, 0x31);
		// disable interleaved mode
		reg_write8(0x2A3, 0);
	}

	void start_measuring() {
		int period = 50;
		int16_t period_reg = clamp(0, 254, (int16_t)(period / 10) - 1);
		reg_write8(0x01B, period_reg);

		// SYSRANGE__START
		reg_write8(0x018, 0x03);
	}

	int get_range_mm() {
		uint8_t range = reg_read8(0x062);
		reg_write8(0x015, 0x01);

		return (int)range;
	}
};

// Just a container/subsystem, not really any sort of abstraction
class DistanceSensors {
public:
	// LEFT
	VL6180X white;
	// RIGHT
	VL6180X green;

	DistanceSensors() : white(LIDAR_I2C, LIDAR_0_CE, 0x29), green(LIDAR_I2C, LIDAR_1_CE, 0x29) {

	}

	void init() {
		white.power(true);
		green.power(false);

		white.addr = 0x29;
		white.reg_write8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, 0x30 & 0x7F);
		white.addr = 0x30;
		white.reg_write8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, 0x30 & 0x7F);

		green.power(true);
		green.addr = 0x29;
		green.reg_write8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, 0x30 & 0x7F);

		green.init();
		white.init();

		green.start_measuring();
		white.start_measuring();
	}

	int get_right_distance() {
		return green.get_range_mm();
	}

	int get_left_distance() {
		return white.get_range_mm();
	}
};

#endif
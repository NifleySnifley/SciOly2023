#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "quad.pio.h"
#include "pico/time.h"
#include <stdint.h>
#include <math.h>

#define LED_R 21
#define LED_Y 20
#define LED_G 19
#define LED_B 18

#define START_BTN 26

#define MOT_IA1 2
#define MOT_IA2 3
#define MOT_IB1 4
#define MOT_IB2 5

#define ENC_A_A 10
#define ENC_A_B 11
#define ENC_B_A 12
#define ENC_B_B 13

#define IMU_SDA 14
#define IMU_SCL 15

const float MOTOR_KHZ = 30.f;
const int MOTOR_CYCLES = 4096;
const float MOTOR_FREQ = MOTOR_KHZ * MOTOR_CYCLES;

const float ENC_COUNT2ROT_FAC = 1.f / 700.f;

void set_leds(uint8_t ledval) {
    gpio_put(LED_B, (ledval & 0b0001) ? 1 : 0);
    gpio_put(LED_G, (ledval & 0b0010) ? 1 : 0);
    gpio_put(LED_Y, (ledval & 0b0100) ? 1 : 0);
    gpio_put(LED_R, (ledval & 0b1000) ? 1 : 0);
}

template<typename T> T clamp(T mi, T ma, T val) {
    return (val > ma) ? ma : ((val < mi) ? mi : val);
}

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
    MotorController(int pin_1, int pin_2, float feedforward = 0.0f) : pin1(pin_1), pin2(pin_2), _ff(feedforward) {
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
        pin1.set_bool(true);
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

float time_seconds() {
    return (float)(time_us_64()) * (1e-6);
}

class PIDController {
    float kP, kI, kD;

    float _integral = 0.f;
    uint64_t _last_ts;
    float _last_e = 0.f;
    float _setpoint = 0.f;
public:
    PIDController(float P, float I, float D) : kP(P), kI(I), kD(D) {}

    void reset() {
        _integral = 0;
    }

    float calculate(float measurement) {
        float e = (_setpoint - measurement);

        uint64_t now_ts = time_us_64();
        float dt = (float)(now_ts - _last_ts) * (1e-6);

        float de_dt = (e - _last_e) / dt;
        _integral += dt * e;

        _last_ts = now_ts;
        _last_e = e;

        return kP * e + kD * de_dt + kI * _integral;
    }

    float calculate(float measurement, float setpoint) {
        _setpoint = setpoint;
        return calculate(measurement);
    }
};

MotorController MA(MOT_IA1, MOT_IA2);
MotorController MB(MOT_IB2, MOT_IB1);
Encoder EB(ENC_A_B, ENC_A_A, ENC_COUNT2ROT_FAC);
Encoder EA(ENC_B_A, ENC_B_B, ENC_COUNT2ROT_FAC);

PIDController pa(5.0f, 0.5f, 0.2f);//(2.0f, 0.5f, 0.2f);
PIDController pb(5.0f, 0.5f, 0.2f);//(2.0f, 0.5f, 0.2f);

int main() {
    stdio_init_all();
    gpio_init(LED_R);
    gpio_init(LED_Y);
    gpio_init(LED_G);
    gpio_init(LED_B);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_set_dir(LED_Y, GPIO_OUT);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);

    pa.reset();

    while (true) {
        //0.5 * (EA.get() + EB.get());// Fun force feedbackish!
        float tgt = time_seconds() * 2.0f;
        float out_a = pa.calculate(EA.get(), tgt);
        float out_b = pb.calculate(EB.get(), tgt);
        MA.set(out_a);
        MB.set(out_b);
        printf("%.3f, %.3f\n", EA.get(), EB.get());
    }
}

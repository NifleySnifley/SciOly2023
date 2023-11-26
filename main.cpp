#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "quad.pio.h"
#include "pico/time.h"
#include <stdint.h>
#include <math.h>
#include "hardware/i2c.h"

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
#define IMU_I2C i2c1

#define LIDAR_SDA 8
#define LIDAR_SCL 9
#define LIDAR_I2C i2c0
// white
#define LIDAR_0_INT 6
#define LIDAR_0_CE 16
// green
#define LIDAR_1_INT 7
#define LIDAR_1_CE 17

const float MOTOR_KHZ = 30.f;
const int MOTOR_CYCLES = 4096;
const float MOTOR_FREQ = MOTOR_KHZ * MOTOR_CYCLES;
#define PI 3.1415926535
constexpr float _PI2 = PI / 2.0f;
constexpr float _2PI = 2.0f * PI;

// Mechanism Parameters
/* Units:
 - Millimeters
 - Radians

   Conventions:
 - Positive Y is FORWARDS
 - Positive X is LEFT
 - Positive rotation is COUNTERCLOCKWISE, 0 is POSITIVE X
*/
const float ODOMETRY_CALIBRATION_FAC = 0.98f * 0.9975f;
const float ENC_COUNT2ROT_FAC = (1.f / 700.f) * ODOMETRY_CALIBRATION_FAC; // Rot/Count
const float WHEEL_RADIUS = 40.4f / 2.f; // MM
const float WHEEL_ROT2MM_FAC = 2 * PI * WHEEL_RADIUS; // MM/Rot
const float WHEELBASE = 83.4; // MM
const float WHEELBASE_2 = WHEELBASE / 2.0f; // MM

typedef struct pose_t {
    float rotation;
    float x;
    float y;
} pose_t;

const float INITIAL_ROTATION = PI / 2.0f;
const pose_t INITIAL_POSE = (pose_t){
    .rotation = INITIAL_ROTATION,
    .x = 0.0f,
    .y = 0.0f
};

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

#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS 0x212
// TODO: Put CE up on one, change I2C address, 
class VL6180X {
    i2c_inst_t* bus;
    int pin_CE;

public:
    uint8_t addr = 0x29;

    VL6180X(i2c_inst_t* i2c, int CE, int address = 0x29) : pin_CE(CE), bus(i2c), addr(address) {
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
        return (uint16_t)(rxbuf[0]) || ((uint16_t)rxbuf[1] << 8); // ???
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
        // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to table "Actual gain values" in datasheet)
        reg_write8(0x03F, 0x46);
        // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
        reg_write8(0x031, 0xFF);
        // sysals__integration_period = 99 (100 ms)
        reg_write16(0x040, 0x0063);
        // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
        reg_write8(0x02E, 0x01);
        // sysrange__intermeasurement_period = 0 (10 ms)
        reg_write8(0x03E, 0x00);
        // sysals__intermeasurement_period = 49 (500 ms)
        reg_write8(0x03E, 0x31);
        // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
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

// Odometry, etc.
class DriveController {
    MotorController l, r;
    Encoder el, er;


    float last_el, last_er;
    uint64_t _last_ts;
public:
    pose_t pose;

    DriveController(MotorController m_left, MotorController m_right, Encoder e_left, Encoder e_right) : l(m_left), r(m_right), el(e_left), er(e_right) {
        last_el = last_er = 0.0f;
        el.set(0.f);
        er.set(0.f);
        _last_ts = time_us_64();


        reset_odometry();
    }

    void reset_odometry() {
        pose = INITIAL_POSE;
        last_el = el.get();
        last_er = er.get();
    }

    // FIXME: These are the "correct" trigonometric odometry calculations
    // Using the linear approximation might improve performance since the rp2040 doesn't have a FPU
    void update_odometry() {
        float cer = er.get();
        float cel = el.get();
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

// HARDWARE MODULES
MotorController MA(MOT_IA1, MOT_IA2);
MotorController MB(MOT_IB2, MOT_IB1);
Encoder EB(ENC_A_B, ENC_A_A, ENC_COUNT2ROT_FAC* WHEEL_ROT2MM_FAC);
Encoder EA(ENC_B_A, ENC_B_B, ENC_COUNT2ROT_FAC* WHEEL_ROT2MM_FAC);
DistanceSensors ds;

// SOFTWARE MODULES
PIDController pangle(1.0f, 0.0f, 0.0f);

PIDController pa(2.0f, 0.05f, 0.05f);//(2.0f, 0.5f, 0.2f);
PIDController pb(2.0f, 0.05f, 0.05f);//(2.0f, 0.5f, 0.2f);

float lerp(float a, float b, float t) {
    return a + (b - a) * clamp(0.f, 1.f, t);
}

float sin_profile(float start, float end, float tmax, float t) {
    return lerp(start, end, 0.5f * (1.f + sinf((PI * (clamp(0.f, tmax, t) - tmax * 0.5f)) / tmax)));
}

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

    gpio_init(START_BTN);
    gpio_pull_up(START_BTN);

    i2c_init(LIDAR_I2C, 100 * 1000);
    gpio_set_function(LIDAR_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LIDAR_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LIDAR_SDA);
    gpio_pull_up(LIDAR_SCL);

    pa.reset();
    pb.reset();
    ds.init();

    MA.set(0.0f);
    MB.set(0.0f);

    DriveController drive(MB, MA, EB, EA);

    sleep_ms(100);

    while (true) {
        gpio_put(LED_B, 1);

        while (true) {
            // printf("%d, %d\n", ds.get_left_distance(), ds.get_right_distance());

            if ((ds.get_left_distance() < 25) || (ds.get_right_distance() < 25)) {
                sleep_ms(5);
                if ((ds.get_left_distance() < 25) || (ds.get_right_distance() < 25))
                    break;
            }
        }
        gpio_put(LED_B, 0);

        EA.set(0.0f);
        EB.set(0.0f);
        drive.reset_odometry();

        float _tinit = time_seconds();
        while (drive.pose.y <= 2000.f) {
            // 0.5 * (EA.get() + EB.get());// Fun force feedbackish!
            float tgt = sin_profile(0.0f, 2001.0f, 20.0f, time_seconds() - _tinit); //time_seconds() * WHEEL_ROT2MM_FAC;
            float out_a = pa.calculate(EA.get(), tgt);
            float out_b = pb.calculate(EB.get(), tgt);
            MA.set(out_a);
            MB.set(out_b);
            drive.update_odometry();
            printf("%.3f, %.3f, %.3f\n", drive.pose.x, drive.pose.y, drive.pose.rotation);
            gpio_put(LED_Y, 1);
            gpio_put(LED_G, 0);
            // sleep_ms(10);
        }

        MA.stop();
        MB.stop();

        gpio_put(LED_G, 1);
        gpio_put(LED_Y, 0);
        gpio_put(LED_R, drive.pose.x > 1005.f);
    }

    while (true) {
        gpio_put(LED_R, 1);
    }

    // gpio_put(LIDAR_0_CE, 0); // Select!
    // gpio_put(LIDAR_1_CE, 1);


    // while (1) {
    //     printf("%d, %d\n", ds.get_left_distance(), ds.get_right_distance());
    //     sleep_ms(10);
    // }
}

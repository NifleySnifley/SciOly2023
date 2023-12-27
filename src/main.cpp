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
#include "utils.h"
#include "constants.h"
#include "hardware.h"

void set_leds(uint8_t ledval) {
    gpio_put(LED_B, (ledval & 0b0001) ? 1 : 0);
    gpio_put(LED_G, (ledval & 0b0010) ? 1 : 0);
    gpio_put(LED_Y, (ledval & 0b0100) ? 1 : 0);
    gpio_put(LED_R, (ledval & 0b1000) ? 1 : 0);
}

// Odometry, etc.
class DriveController {
    MotorController l, r;
    Encoder el, er;


    float last_el, last_er;
    uint64_t _last_ts;
public:
    pose_t pose;

    DriveController(
        MotorController m_left, MotorController m_right,
        Encoder e_left, Encoder e_right
    ) : l(m_left), r(m_right), el(e_left), er(e_right) {
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
    // Using the linear approximation might improve performance since the rp2040 
    // doesn't have a FPU
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

PIDController pa(2.5f, 0.05f, 0.08f);//(2.0f, 0.5f, 0.2f);
PIDController pb(2.5f, 0.05f, 0.08f);//(2.0f, 0.5f, 0.2f);

const float INITIAL_ROTATION = PI / 2.0f;
const pose_t INITIAL_POSE = (pose_t){
    INITIAL_ROTATION,
    0.0f,
    0.0f
};


// Max dret/dt is (PI*(end-start))/tmax
float sin_profile(float start, float end, float tmax, float t) {
    return lerp(
        start,
        end,
        0.5f * (1.f + sinf((PI * (clamp(0.f, tmax, t) - tmax * 0.5f)) / tmax))
    );
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
        pa.reset();
        pb.reset();

        float _tinit = time_seconds();
        // Calculate the optimal time so that the vehicle maximizes speed given the profile
        const float tmin = (2000.f * PI) / MAX_SPEED;
        while (drive.pose.y <= 2000.f) {
            // 0.5 * (EA.get() + EB.get());// Fun force feedbackish!
            float tgt = sin_profile(0.0f, 2000.0f, tmin, time_seconds() - _tinit);
            float out_a = pa.calculate(EA.get(), tgt);
            float out_b = pb.calculate(EB.get(), tgt);
            MA.set(out_a);
            MB.set(out_b);
            drive.update_odometry();
            printf("%f, %f\n", EA.get(), tgt);
            gpio_put(LED_Y, 1);
            gpio_put(LED_G, 0);
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
}

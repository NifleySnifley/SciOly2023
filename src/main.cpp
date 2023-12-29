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
#include "pathing.h"
#include "maze_solving.h"
#include "pindefs.h"
#include "subsystems.h"
#include "controller.h"

// TODO: Check!!!
const float INITIAL_ROTATION = PI / 2.0f;
const pose_t INITIAL_POSE = (pose_t){
    INITIAL_ROTATION,
    0.f, //START_POS.x,
    0.f // START_POS.y
};

void set_leds(uint8_t ledval) {
    gpio_put(LED_B, (ledval & 0b0001) ? 1 : 0);
    gpio_put(LED_G, (ledval & 0b0010) ? 1 : 0);
    gpio_put(LED_Y, (ledval & 0b0100) ? 1 : 0);
    gpio_put(LED_R, (ledval & 0b1000) ? 1 : 0);
}

// HARDWARE MODULES
MotorController MA(MOT_IA1, MOT_IA2);
MotorController MB(MOT_IB2, MOT_IB1);
Encoder EB(ENC_A_B, ENC_A_A, ENC_COUNT2ROT_FAC* WHEEL_ROT2MM_FAC);
Encoder EA(ENC_B_A, ENC_B_B, ENC_COUNT2ROT_FAC* WHEEL_ROT2MM_FAC);
DistanceSensors ds;

// SOFTWARE MODULES
// PIDController pangle(1.0f, 0.0f, 0.0f);

// PIDController pa(2.5f, 0.05f, 0.08f);//(2.0f, 0.5f, 0.2f);
// PIDController pb(2.5f, 0.05f, 0.08f);//(2.0f, 0.5f, 0.2f);


// Max dret/dt is (PI*(end-start))/tmax
float sin_profile(float start, float end, float tmax, float t) {
    return lerp(
        start,
        end,
        0.5f * (1.f + sinf((PI * (clamp(0.f, tmax, t) - tmax * 0.5f)) / tmax))
    );
}

int main() {
    ///////////////// Hardware Init /////////////////
    stdio_init_all();
    gpio_init(LED_R);
    gpio_init(LED_Y);
    gpio_init(LED_G);
    gpio_init(LED_B);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_set_dir(LED_Y, GPIO_OUT);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);
    set_leds(0);

    gpio_init(START_BTN);
    gpio_pull_up(START_BTN);

    i2c_init(LIDAR_I2C, 100 * 1000);
    gpio_set_function(LIDAR_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LIDAR_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LIDAR_SDA);
    gpio_pull_up(LIDAR_SCL);

    // pa.reset();
    // pb.reset();
    ds.init();

    MA.stop();
    MB.stop();

    Odometry odom(&EB, &EA, INITIAL_POSE);

    ///////////////// Pre-Calculations /////////////////
    PathFinding p;
    p.calculate();
    // gpio_put(LED_R, 1);
    p.optimize_routes();
    int pathlen = p.get_full_path();
    // gpio_put(LED_Y, 1);

    std::vector<Vec2> pps;
    pps.push_back(START_POS);
    for (int i = 0; i < pathlen; ++i) {
        pps.push_back(tmp[i].bottom_left_on_field_mm() + HALF_CELL);
    }

    // gpio_put(LED_G, 1);

    BSplinePath path(pps);
    path.calculate();
    // gpio_put(LED_B, 1);

    PursuitController controller(&path, &odom, &MB, &MA);


    ///////////////// Control Loop /////////////////
    // TODO: Multicore processing?
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

        odom.reset_odometry();
        controller.reset();
        // pa.reset();
        // pb.reset();

        do {
            odom.update_odometry();
            gpio_put(LED_Y, 1);
            gpio_put(LED_G, 0);
            if (!gpio_get(START_BTN)) {
                break;
            }
            sleep_ms(10);
            // printf("%.2f, %.2f, %.2f\n", odom.pose.x, odom.pose.y, odom.pose.rotation);
        } while (!controller.execute(Vec2()));

        // float _tinit = time_seconds();
        // // Calculate the optimal time so that the vehicle maximizes speed given the profile
        // const float tmin = (2000.f * PI) / MAX_SPEED;
        // while (odom.pose.y <= 2000.f) {
        //     // 0.5 * (EA.get() + EB.get());// Fun force feedbackish!
        //     float tgt = sin_profile(0.0f, 2000.0f, tmin, time_seconds() - _tinit);
        //     float out_a = pa.calculate(EA.get(), tgt);
        //     float out_b = pb.calculate(EB.get(), tgt);
        //     MA.set(out_a);
        //     MB.set(out_b);
        //     odom.update_odometry();
        //     printf("%f, %f\n", EA.get(), tgt);
        //     gpio_put(LED_Y, 1);
        //     gpio_put(LED_G, 0);
        // }

        /////////////////////////// CONTROL LOOP ///////////////////////////

        MA.stop();
        MB.stop();

        gpio_put(LED_G, 1);
        gpio_put(LED_Y, 0);
        // gpio_put(LED_R, odom.pose.x > 1005.f);
    }

    panic();
}

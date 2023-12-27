#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifdef PICO_FLASH_SIZE_BYTES
#include "hardware/i2c.h"
#endif
#include "utils.h"
#include <array>

#ifdef PICO_FLASH_SIZE_BYTES

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
#endif


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

const float MAX_RPM = 310.0f; // Rot/s
const float MAX_SPEED = (WHEEL_ROT2MM_FAC * MAX_RPM) / 60.0f; // MM/s

//////////// PATHFINDING CONSTANTS!! ///////////////

#define HALF_CELL Vec2(GRIDSQUARE_SIZE_MM * 0.5f)

const std::array<GridSquare, 3> GOALS = { GridSquare(0, 1), GridSquare(2, 3), GridSquare(3, 1) };

// TODO: Get walls from parsing a "ascii art" (string) representation of the
// field with box drawing characters, etc!
const std::array<Edge, 9> WALLS = {
	Edge(0, 1, 0, 2), Edge(1, 0, 1, 1), Edge(1, 1, 2, 1), Edge(2, 1, 2, 2),
	Edge(3, 0, 3, 1), Edge(1, 2, 1, 3), Edge(1, 3, 2, 3), Edge(2, 3, 3, 3), Edge(0,1,1,1) };

const GridSquare TARGET = GridSquare(1, 3);
const GridSquare START = GridSquare(1, 0);
const Vec2 START_POS = (Vec2)START.bottom_left_on_field_mm() + Vec2(HALF_CELL.x, 0.0f);

const GridSquare N_OFFSETS[4] = { GridSquare(1, 0), GridSquare(-1, 0),
								 GridSquare(0, 1), GridSquare(0, -1)

};

#endif
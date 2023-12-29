#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <array>

// HARDWARE CONFIGURATION
const float MOTOR_KHZ = 30.f;
const int MOTOR_CYCLES = 4096;
const float MOTOR_FREQ = MOTOR_KHZ * MOTOR_CYCLES;
#define PI 3.1415926535f
constexpr float _PI2 = PI / 2.0f;
constexpr float _2PI = 2.0f * PI;

//////////// Mechanism Parameters ////////////
/* Units:
 - Millimeters
 - Radians

   Conventions:
 - Positive Y is FORWARDS
 - Positive X is LEFT
 - Positive rotation is COUNTERCLOCKWISE, 0 is POSITIVE X
*/
const float ODOMETRY_CALIBRATION_FAC = 0.98f * 0.9975f * 0.99;
const float ENC_COUNT2ROT_FAC = (1.f / 700.f) * ODOMETRY_CALIBRATION_FAC; // Rot/Count
const float WHEEL_RADIUS = 40.4f / 2.f; // MM
const float WHEEL_ROT2MM_FAC = 2 * PI * WHEEL_RADIUS; // MM/Rot
const float WHEELBASE = 83.4f; // MM
const float WHEELBASE_2 = WHEELBASE / 2.0f; // MM

const float MAX_RPM = 310.0f; // Rot/s
const float MAX_SPEED = (WHEEL_ROT2MM_FAC * MAX_RPM) / 60.0f; // MM/s

//////////// PATHFINDING CONSTANTS!! ///////////////

#define HALF_CELL Vec2(GRIDSQUARE_SIZE_MM * 0.5f)

// const std::array<GridSquare, 3> GOALS = { GridSquare(0, 1), GridSquare(2, 3), GridSquare(3, 1) };

// // TODO: Get walls from parsing a "ascii art" (string) representation of the
// // field with box drawing characters, etc!
// const std::array<Edge, 8> WALLS = {
// 	Edge(0, 1, 0, 2), Edge(1, 0, 1, 1), Edge(1, 1, 2, 1), Edge(2, 1, 2, 2),
// 	Edge(3, 0, 3, 1), Edge(1, 2, 1, 3), Edge(1, 3, 2, 3), Edge(2, 3, 3, 3) };

// const GridSquare TARGET = GridSquare(1, 3);
// const GridSquare START = GridSquare(1, 0);
// const Vec2 START_POS = (Vec2)START.bottom_left_on_field_mm() + Vec2(HALF_CELL.x, 0.0f);

// // TODO: Properly refactor codebase (edge checking) to permit cutting corners (45deg)
// const GridSquare N_OFFSETS[4] = { GridSquare(1, 0), GridSquare(-1, 0),
// 								 GridSquare(0, 1), GridSquare(0, -1)
// };


/////////////// DEBUGGING (Mom's course) /////////////////
const std::array<GridSquare, 3> GOALS = { GridSquare(0,0), GridSquare(0,1), GridSquare(3,0) };

// TODO: Get walls from parsing a "ascii art" (string) representation of the
// field with box drawing characters, etc!
const std::array<Edge, 9> WALLS = { Edge(0,0,0,1), Edge(0,1,1,1), Edge(0,2,1,2), Edge(1,0,2,0), Edge(1,1,2,1), Edge(2,0,3,0), Edge(1,3,2,3), Edge(2,3,2,2), Edge(3,1,3,2) };

const GridSquare TARGET = GridSquare(3, 3);
const GridSquare START = GridSquare(2, 0);
const Vec2 START_POS = (Vec2)START.bottom_left_on_field_mm() + Vec2(HALF_CELL.x, 0.0f);

// TODO: Properly refactor codebase (edge checking) to permit cutting corners (45deg)
const GridSquare N_OFFSETS[4] = { GridSquare(1, 0), GridSquare(-1, 0),
								 GridSquare(0, 1), GridSquare(0, -1)
};

#endif
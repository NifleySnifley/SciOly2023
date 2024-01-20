#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <array>
#include <vector>

const char* MAZE_STR = R"(
+-----_-----+-----_-----+-----_-----+-----_-----+
|           |           |           |           |
|           |           |           |           |
_     _     _     _     W     _     _     _     _
|           |           |           |           |
|           |           |           |           |
+-----_-----+-----_-----+-----_-----+-----_-----+
|           |           |           |           |
|           |           |           |           |
_     G     W     E     _     _     _     _     _
|           |           |           |           |
|           |           |           |           |
+-----W-----+-----W-----+-----W-----+-----_-----+
|           |           |           |           |
|           |           |           |           |
_     _     _     _     _     G     W     _     _
|           |           |           |           |
|           |           |           |           |
+-----_-----+-----W-----+-----_-----+-----_-----+
|           |           |           |           |
|           |           |           |           |
_     _     _     G     W     S     _     _     _
|           |           |           |           |
|           |           |           |           |
+-----_-----+-----_-----+-----_-----+-----_-----+
)";
// Time goal, in seconds
const float TIME_GOAL = 30.0f;




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

 Conventions: (Facing "towards" the course)
 - Positive Y is FORWARDS
 - Positive X is LEFT
 - Positive rotation is COUNTERCLOCKWISE, 0 is POSITIVE X
*/
const float ODOMETRY_CALIBRATION_FAC = 0.989f; //0.98f * 0.9975f * 0.99f;
const float ENC_COUNT2ROT_FAC = (1.f / 700.f) * ODOMETRY_CALIBRATION_FAC; // Rot/Count
const float WHEEL_RADIUS = 40.4f / 2.f; // MM
const float WHEEL_ROT2MM_FAC = 2 * PI * WHEEL_RADIUS; // MM/Rot
// TODO: Use the math so it's possible to have a rotational calibration factor thats applied after `ODOMETRY_CALIBRATION_FAC`
const float WHEELBASE = 83.4f / 0.973; // MM
const float WHEELBASE_2 = WHEELBASE / 2.0f; // MM

const float MEAS_POINT_TO_WHEELBASE = 70.0f;

const float MAX_RPM = 310.0f; // Rot/s
const float MAX_SPEED = (WHEEL_ROT2MM_FAC * MAX_RPM) / 60.0f; // MM/s

//////////// PATHFINDING CONSTANTS!! ///////////////

#define HALF_CELL Vec2(GRIDSQUARE_SIZE_MM * 0.5f)

const GridSquare N_OFFSETS[4] = { GridSquare(1, 0), GridSquare(-1, 0),
								 GridSquare(0, 1), GridSquare(0, -1)
};

//////////// MAP "CONSTANTS" ////////////
// Generated by `maze_parser.h` from MAP_STR

static std::vector<GridSquare> GOALS = {}; //{ GridSquare(0,0), GridSquare(0,1), GridSquare(3,0) };
static std::vector<Edge> WALLS = {};//{ Edge(0,0,0,1), Edge(0,1,1,1), Edge(0,2,1,2), Edge(1,0,2,0), Edge(1,1,2,1), Edge(2,0,3,0), Edge(1,3,2,3), Edge(2,3,2,2), Edge(3,1,3,2) };

static GridSquare TARGET = GridSquare();
static GridSquare START = GridSquare();

#define START_POS ((Vec2)START.bottom_left_on_field_mm() + Vec2(HALF_CELL.x, -MEAS_POINT_TO_WHEELBASE))

#endif
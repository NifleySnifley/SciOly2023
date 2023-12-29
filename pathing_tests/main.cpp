#define DEBUG_GUI
extern "C" {
#include "raylib.h"
}
#include "utils.h"
#include "constants.h"
#include "maze_solving.h"
#include "pathing.h"
#include "sim_hardware.h"
#include "subsystems.h"
#include "controller.h"
#include "debug.h"
#include <string>

void drawWall(Edge edge) {
	Vec2 a = edge.a.bottom_left_on_field_mm() + Vec2(GRIDSQUARE_SIZE_MM / 2.0f);
	Vec2 b = edge.b.bottom_left_on_field_mm() + Vec2(GRIDSQUARE_SIZE_MM / 2.0f);

	Vec2 d = (edge.b.bottom_left_on_field_mm() - edge.a.bottom_left_on_field_mm()) / 2.0f;
	d = Vec2(d.y, -d.x);

	DrawLineEx(
		robot2screenspace((a + b) * 0.5f + d),
		robot2screenspace((a + b) * 0.5f - d),
		5.0f,
		RED
	);
}

int main(void) {
	SetConfigFlags(FLAG_MSAA_4X_HINT);
	InitWindow(screenWidth, screenHeight, "SciOly 2024 Robot Tour debug visualizer");

	PathFinding p;
	p.calculate();
	p.optimize_routes();
	int pathlen = p.get_full_path();

	std::vector<Vec2> pps;
	pps.push_back(START_POS);
	for (int i = 0; i < pathlen; ++i) {
		pps.push_back(tmp[i].bottom_left_on_field_mm() + HALF_CELL);
	}

	std::cout << "Waypoints done: " << pathlen << std::endl;
	BSplinePath path(pps);
	path.calculate();

	std::cout << "Calculations done! Size: " << path.pathpoints.size() << std::endl;

	SetTargetFPS(120);               // Set our game to run at 60 frames-per-second
	//--------------------------------------------------------------------------------------

	// const float INITIAL_ROTATION = PI / 2.0f;
	MotorController left_motor, right_motor;
	Encoder left_encoder(&left_motor), right_encoder(&right_motor);
	Odometry odometry(&left_encoder, &right_encoder, { 0.0f, 0.0f, 0.0f });
	PursuitController controller(&path, &odometry, &left_motor, &right_motor);

	//--------------------------------------------------------------------------------------


	const float BOT_WIDTH = 90.0f;
	const float BOT_LENGTH = 105.0f;
	Texture2D bot = LoadTexture("../../bot.png");
	Rectangle bot_tex_rect;
	bot_tex_rect.x = 0;
	bot_tex_rect.y = 0;
	bot_tex_rect.width = (float)bot.width;
	bot_tex_rect.height = (float)bot.height;

	float last_time = (float)GetTime();
	bool executing = true;

	float start_time = (float)GetTime();
	float end_time = (float)GetTime();

	// Main game loop
	while (!WindowShouldClose())    // Detect window close button or ESC key
	{
		// Update
		//----------------------------------------------------------------------------------
		Vector2 mouse = GetMousePosition();
		Vec2 mouse_robotspace;
		mouse_robotspace.x = (mouse.x / (float)screenWidth) * GRID_SIZE_MM;
		mouse_robotspace.y = ((screenHeight - mouse.y) / (float)screenHeight) * GRID_SIZE_MM;

		float tdelta = (float)GetTime() - last_time;
		last_time = (float)GetTime();

		// std::cout << "(" << odometry.pose.x << ", " << odometry.pose.y << ")" << std::endl;

		if (GetKeyPressed() == KEY_R) {
			controller.reset();
			start_time = (float)GetTime();
			executing = true;
		}

		BeginDrawing();
		{
			ClearBackground(RAYWHITE);

			// Inside drawing!
			left_motor._simulate(tdelta);
			right_motor._simulate(tdelta);
			odometry.update_odometry();

			if (executing) {
				// std::cout << "exec" << std::endl;
				executing = !controller.execute(mouse_robotspace);
				end_time = (float)GetTime();
			}
			// executing = true;

			// Draw grid
			for (int i = 1; i <= 3; ++i) {
				DrawLineV(
					robot2screenspace(Vec2(0.0f, i * 500.0f)),
					robot2screenspace(Vec2(2000.0f, i * 500.0f)),
					BLUE
				);

				DrawLineV(
					robot2screenspace(Vec2(i * 500.0f, 0.0f)),
					robot2screenspace(Vec2(i * 500.0f, 2000.0f)),
					BLUE
				);
			}

			// Draw walls (obstructions)
			for (Edge e : WALLS) {
				drawWall(e);
			}

			// Draw goal cells
			for (GridSquare g : GOALS) {
				Rectangle rect;
				rect.x = robot2screenspace(g.bottom_left_on_field_mm()).x + 10.f;
				rect.y = robot2screenspace(g.bottom_left_on_field_mm()).y - GRIDSQUARE_SIZE_SCREEN + 10.f;
				rect.width = GRIDSQUARE_SIZE_SCREEN - 20.0f;
				rect.height = GRIDSQUARE_SIZE_SCREEN - 20.0f;

				DrawRectangleLinesEx(rect, 5.0f, GREEN);
			}

			// Draw  target and start points
			DrawCircleV(robot2screenspace((Vec2)TARGET.bottom_left_on_field_mm() + HALF_CELL), 10.0f, GREEN);
			DrawCircleV(robot2screenspace((Vec2)START.bottom_left_on_field_mm() + HALF_CELL), 10.0f, ORANGE);

			// Draw cusp (reversal) points
			for (int i : path.cusp_indices) {
				DrawCircleV(robot2screenspace(path.pathpoints[i]), 5.0f, RED);
			}

			std::vector<Vector2> spoints;

			const float t_int = path.total_distance() / MAX_SPEED;
			const size_t t_frame = path.pathpoints.size() - 1;//(int)((fmodf(GetTime(), t_int) / t_int) * path.pathpoints.size());
			for (size_t i = 0; i <= t_frame; ++i) {
				spoints.push_back(robot2screenspace(path.pathpoints[i]));
			}

			// Draw the path
			DrawSplineLinear(spoints.data(), spoints.size(), 1.0f, PURPLE);

			// Draw the robot along the path
			{
				float dist = (fmodf((float)GetTime(), t_int) / t_int) * path.total_distance();
				// Vector2 position = robot2screenspace(path.point_at_distance(dist));
				Vector2 position = robot2screenspace(Vec2(odometry.pose.x, odometry.pose.y));

				// Vec2 dxy_dt = (path.pathpoints[t_frame] - path.pathpoints[t_frame == 0 ? 0 : t_frame - 1]).normalize();
				// Vec2 dxy_dt = path.dxy_at_distance(dist).normalize();
				Vec2 dxy_dt = Vec2::from_polar(odometry.pose.rotation, 1.0f);

				Vector2 bot_size_screen = robot2screenspace(Vec2(BOT_WIDTH, GRID_SIZE_MM - BOT_LENGTH));

				Rectangle dst_rect = { position.x, position.y, bot_size_screen.x, bot_size_screen.y };

				DrawTexturePro(bot, bot_tex_rect, dst_rect, { bot_size_screen.x * 0.5f, bot_size_screen.y * 0.5f }, -(dxy_dt.direction_radians() / PI) * 180.f + 90.f, WHITE);
			}
			std::string text = std::to_string(end_time - start_time);
			DrawText(text.c_str(), 5, 5, 24, BLACK);
		}
		EndDrawing();
	}

	CloseWindow();

	return 0;
}
/*******************************************************************************************
*
*   raylib [shapes] example - Cubic-bezier lines
*
*   Example originally created with raylib 1.7, last time updated with raylib 1.7
*
*   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software
*
*   Copyright (c) 2017-2023 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

extern "C" {
#include "raylib.h"
}
#include "utils.h"
#include "constants.h"
#include "maze_solving.h"
#include "pathing.h"

const int screenWidth = 800;
const int screenHeight = 800;

const int GRIDSQUARE_SIZE_SCREEN = screenWidth / GRID_W;

// Robot type vec2 in MM to screen space
Vector2 robot2screenspace(Vec2 v) {
	return { screenWidth * (v.x / 2000.f), screenHeight - screenHeight * (v.y / 2000.f) };
}

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

	// Vector2 startPoint = { 30, 30 };
	// Vector2 endPoint = { (float)screenWidth - 30, (float)screenHeight - 30 };
	// bool moveStartPoint = false;
	// bool moveEndPoint = false;

	PathFinding p;
	p.calculate();
	p.optimize_routes();
	int pathlen = p.get_full_path();

	std::vector<Vec2> pps;
	pps.push_back(START_POS);
	for (int i = 0; i < pathlen; ++i) {
		pps.push_back(tmp[i].bottom_left_on_field_mm() + HALF_CELL);
	}
	// pps.push_back((Vec2)TARGET.bottom_left_on_field_mm() + HALF_CELL);

	Path path(pps);
	path.calculate();

	std::cout << "Size: " << path.pathpoints.size() << std::endl;

	SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
	//--------------------------------------------------------------------------------------

	// Main game loop
	while (!WindowShouldClose())    // Detect window close button or ESC key
	{
		// Update
		//----------------------------------------------------------------------------------
		Vector2 mouse = GetMousePosition();

		// if (CheckCollisionPointCircle(mouse, startPoint, 10.0f) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) moveStartPoint = true;
		// else if (CheckCollisionPointCircle(mouse, endPoint, 10.0f) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) moveEndPoint = true;

		// if (moveStartPoint) {
		// 	startPoint = mouse;
		// 	if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) moveStartPoint = false;
		// }

		// if (moveEndPoint) {
		// 	endPoint = mouse;
		// 	if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) moveEndPoint = false;
		// }
		//----------------------------------------------------------------------------------

		// Draw
		//----------------------------------------------------------------------------------
		BeginDrawing();
		{
			ClearBackground(RAYWHITE);

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

			for (Edge e : WALLS) {
				drawWall(e);
			}

			for (GridSquare g : GOALS) {
				Rectangle rect;
				rect.x = robot2screenspace(g.bottom_left_on_field_mm()).x + 10.f;
				rect.y = robot2screenspace(g.bottom_left_on_field_mm()).y - GRIDSQUARE_SIZE_SCREEN + 10.f;
				rect.width = GRIDSQUARE_SIZE_SCREEN - 20.0f;
				rect.height = GRIDSQUARE_SIZE_SCREEN - 20.0f;

				DrawRectangleLinesEx(rect, 5.0f, GREEN);
			}

			DrawCircleV(robot2screenspace((Vec2)TARGET.bottom_left_on_field_mm() + HALF_CELL), 10.0f, GREEN);
			DrawCircleV(robot2screenspace((Vec2)START.bottom_left_on_field_mm() + HALF_CELL), 10.0f, ORANGE);

			std::vector<Vector2> spoints;

			// for (Vec2 v : path.pathpoints) {
				// spoints.push_back(robot2screenspace(v));
			// }

			const float t_int = path.total_distance / MAX_SPEED;
			for (int i = 0; i <= (int)((fmodf(GetTime(), t_int) / t_int) * path.pathpoints.size()); ++i) {
				spoints.push_back(robot2screenspace(path.pathpoints[i]));
			}

			DrawSplineLinear(spoints.data(), spoints.size(), 3.0, BLUE);
			// DrawSplineCatmullRom(spoints.data(), spoints.size(), 3.0, RED);
			// DrawSplineBasis(spoints.data(), spoints.size(), 3.0, PURPLE);
			// DrawSplineBezierQuadratic(spoints.data(), spoints.size(), 3.0, PURPLE);
		}
		EndDrawing();

		// DrawText("MOVE START-END POINTS WITH MOUSE", 15, 20, 20, GRAY);

		// // Draw line Cubic Bezier, in-out interpolation (easing), no control points
		// DrawLineBezier(startPoint, endPoint, 4.0f, BLUE);

		// // Draw start-end spline circles with some details
		// DrawCircleV(startPoint, CheckCollisionPointCircle(mouse, startPoint, 10.0f) ? 14 : 8, moveStartPoint ? RED : BLUE);
		// DrawCircleV(endPoint, CheckCollisionPointCircle(mouse, endPoint, 10.0f) ? 14 : 8, moveEndPoint ? RED : BLUE);

		//----------------------------------------------------------------------------------
	}

	CloseWindow();

	return 0;
}
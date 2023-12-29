#ifndef DEBUG_H
#define DEBUG_H

const int screenWidth = 1000;
const int screenHeight = 1000;

const int GRIDSQUARE_SIZE_SCREEN = screenWidth / GRID_W;

// Robot type vec2 in MM to screen space
Vector2 robot2screenspace(Vec2 v) {
	return { screenWidth * (v.x / 2000.f), screenHeight - screenHeight * (v.y / 2000.f) };
}

const float ROBOT_2_SCREEN = (1.0f / GRID_SIZE_MM) * screenWidth;


#endif
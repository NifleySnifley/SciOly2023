#ifndef MAZE_PARSER_H
#define MAZE_PARSER_H

#include "utils.h"
#include "constants.h"
#include "string.h"
#include <iostream>

const size_t STR_CELL_H = 6;
const size_t STR_CELL_W = 12;
const char START_CHAR = '+';

void parse_maze() {
	GOALS.clear();
	WALLS.clear();

	size_t starti = 0;
	size_t len = strlen(MAZE_STR);

	while (starti < len && MAZE_STR[starti] != START_CHAR) {
		++starti;
	}

	size_t col = 0;
	size_t row = 0;

	for (size_t i = starti; i < len; ++i) {
		if (MAZE_STR[i] == '\n') {
			row += 1;
			col = 0;
		} else {
			int celly = GRID_H - 1 - (row / STR_CELL_H);
			int cellx = (col / STR_CELL_W);
			if (((row - STR_CELL_H / 2) % STR_CELL_H) == 0 && (col % STR_CELL_W) == 0) {
				if (MAZE_STR[i] == 'W') {
					// VERTICAL WALL PARSE
					WALLS.push_back(Edge(cellx - 1, celly, cellx, celly));
				}
			} else if ((row % STR_CELL_H) == 0 && ((col - STR_CELL_W / 2) % STR_CELL_W) == 0) {
				if (MAZE_STR[i] == 'W') {
					// HORIZONTAL WALL PARSE
					WALLS.push_back(Edge(cellx, celly, cellx, celly + 1));
				}
			} else if (((row - STR_CELL_H / 2) % STR_CELL_H) == 0 && ((col - STR_CELL_W / 2) % STR_CELL_W) == 0) {
				// SQUARE PARSE
				switch (MAZE_STR[i]) {
					case 'E':
					case 'T':
						TARGET = GridSquare(cellx, celly);
						break;
					case 'S':
					case 'B':
						// Only one start!!!!
						// ASSERT_PANIC(!START.isvalid());
						START = GridSquare(cellx, celly);
						ASSERT_PANIC(START.y == 0);
						break;
					case 'G':
						GOALS.push_back(GridSquare(cellx, celly));
						break;
				}
			}
			col += 1;
		}
	}
	ASSERT_PANIC(START.isvalid());
	ASSERT_PANIC(TARGET.isvalid());
}

#endif
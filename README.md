# Science Olympiad 2024 Robot Tour
Apologies for the incorrect repository name, this is the code for the robot controller (Pi Pico) and simulator for Century High School's 2024 Robot Tour robot.

The `src/` directory contains the code that runs on the robot including:
- `maze_parser.h` (parsing the maze layout from a textual representation)
- `maze_solving.h` (maze solver to find the most efficient path around the maze using implementations of the floyd-warshall algorithm and a brute force travelling salesperson problem solver)
- `pathing.h` (B-Spline path representation used to convert a series of grid movements from the maze solver into a smooth, continuous path for following. Based on `raylib`'s B-Spline code)
- `controller.h` (An adaptation of the pure-pursuit path following controller tweaked for speed on the microcontroller)
- `hardware.h` (Wrappers for various hardware devices/interfaces (PWM output, motor controllers, distance sensors, etc.))
- `subsytems.h` (Odometry subsystem)
- `utils.h` (Miscellaneous data structures, PID controller implementation, etc.)
- `constants.h` (Parameters for controllers, maze input, etc.)
- `main.cpp` (Robot controller program, realtime loop, etc.)

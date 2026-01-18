# MoW

# Local Repo
C:\Users\Admin\Desktop\HULA Project\7 Detect Object

## Directory Structure

| Folder | Description |
|--------|-------------|
| `Tested/` | Scripts verified and validated on physical hardware |
| `Sim/` | Simulation scripts and code tested in simulated environments |

## Tested Scripts

### Tester7Coord.py
Coordinate-based maze navigation for the drone. Uses absolute coordinates to move between blocks in a grid, with obstacle detection and path optimization based on visit counts and Manhattan distance to goal.

### aMaze3.py
Enhanced maze navigation with real-time visualization. Builds on coordinate-based movement and adds a matplotlib display showing the drone's position, path history, and detected walls as the maze is explored.

## Sim Scripts

### Core Files

#### pyhula_simulator.py
Drop-in replacement for the pyhula API that simulates drone behavior. Supports all flight commands, obstacle detection, and maintains drone state without requiring physical hardware.

#### maze_config.py
Configuration file for maze parameters. Define maze dimensions, start/end positions, and wall placements here.

### Navigation Scripts

#### TestMaze.py
Basic maze navigation using visit counting and Manhattan distance heuristics. Good starting point for testing maze configurations.

#### MazeFixed.py
Improved maze solver with dead-end detection. Marks dead-ends to avoid revisiting them, reducing unnecessary backtracking.

#### TestmazeDstar.py
D* Lite pathfinding algorithm implementation. Displays distance values on each cell and dynamically replans when walls are discovered.

#### MazeGoHome.py
Three-phase navigation: (1) explore to end, (2) return to start, (3) follow optimal path. Demonstrates learned maze knowledge by showing the shortest route on the final run.

### Exploration Scripts

#### Explore.py
Full maze exploration using D* Lite. Visits every reachable block in the maze by finding and navigating to the nearest unvisited cell.

#### ExploreForward.py
Forward-facing exploration that always moves in the direction the drone is facing. Uses right-hand rule priority with preference for unvisited cells.

#### GoHomeTurbo.py
Optimized version of MazeGoHome with fast continuous flight. Phase 3 simplifies the optimal path to corner waypoints only, enabling high-speed flight without stopping at every block.

## Getting Started

1. Edit `maze_config.py` to set up your maze
2. Run any simulation script to test navigation algorithms
3. Use `Tested/` scripts for deployment on physical hardware


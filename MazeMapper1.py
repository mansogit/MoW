"""
MazeMapper - Discovers and maps the maze
Explores ALL cells systematically (discovering walls)
Lands at END and saves maze map to JSON file
"""

import pyhula
import time
import heapq
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Initialize and connect
api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

# Maze parameters
BLOCK_SIZE = 60  # cm per block
FLIGHT_HEIGHT = 80  # cm
BLOCK = 0.6  # Plot block size
OBSTACLE_CHECK_COUNT = 3  # Number of times to verify obstacle before confirming

# First block center coordinates
OFFSET_X = 0  # cm offset
OFFSET_Y = 0  # cm offset
FIRST_BLOCK_CENTER_X = 15 + OFFSET_X  # cm
FIRST_BLOCK_CENTER_Y = 15 + OFFSET_Y  # cm


def block_to_cm(block_x, block_y):
    """Convert block coordinates to absolute cm coordinates (center of block)"""
    abs_x = FIRST_BLOCK_CENTER_X + (block_x * BLOCK_SIZE)
    abs_y = FIRST_BLOCK_CENTER_Y + (block_y * BLOCK_SIZE)
    return abs_x, abs_y


def move_to_block(block_x, block_y):
    """Move to a block position"""
    abs_x, abs_y = block_to_cm(block_x, block_y)
    api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)


def get_verified_obstacles():
    """Get obstacle readings with verification - only confirm if detected multiple times"""
    readings = {'forward': 0, 'back': 0, 'left': 0, 'right': 0}

    # Take multiple readings
    for i in range(OBSTACLE_CHECK_COUNT):
        obstacles = api.Plane_getBarrier()
        for direction in readings:
            if obstacles.get(direction, False):
                readings[direction] += 1
        time.sleep(0.3)  # Short delay between readings

    # Only confirm obstacle if detected in majority of readings
    threshold = OBSTACLE_CHECK_COUNT // 2 + 1  # More than half
    verified = {}
    for direction, count in readings.items():
        verified[direction] = count >= threshold

    print(f"  Raw readings over {OBSTACLE_CHECK_COUNT} checks: {readings}")
    print(f"  Verified obstacles (>={threshold} detections): {verified}")

    return verified


def get_walls_from_obstacles(obstacles, current_x, current_y, rows, cols):
    """Convert obstacle detection to wall format"""
    new_walls = []

    # Forward
    if obstacles.get('forward', False):
        if current_y + 1 < rows:
            wall = tuple(sorted([(current_x, current_y), (current_x, current_y + 1)]))
            new_walls.append(wall)
            print(f"  -> INTERNAL WALL: forward (between ({current_x},{current_y}) and ({current_x},{current_y+1}))")
        else:
            print(f"  -> Boundary: forward (top edge of maze)")

    # Back
    if obstacles.get('back', False):
        if current_y - 1 >= 0:
            wall = tuple(sorted([(current_x, current_y), (current_x, current_y - 1)]))
            new_walls.append(wall)
            print(f"  -> INTERNAL WALL: back (between ({current_x},{current_y}) and ({current_x},{current_y-1}))")
        else:
            print(f"  -> Boundary: back (bottom edge of maze)")

    # Right
    if obstacles.get('right', False):
        if current_x + 1 < cols:
            wall = tuple(sorted([(current_x, current_y), (current_x + 1, current_y)]))
            new_walls.append(wall)
            print(f"  -> INTERNAL WALL: right (between ({current_x},{current_y}) and ({current_x+1},{current_y}))")
        else:
            print(f"  -> Boundary: right (right edge of maze)")

    # Left
    if obstacles.get('left', False):
        if current_x - 1 >= 0:
            wall = tuple(sorted([(current_x, current_y), (current_x - 1, current_y)]))
            new_walls.append(wall)
            print(f"  -> INTERNAL WALL: left (between ({current_x},{current_y}) and ({current_x-1},{current_y}))")
        else:
            print(f"  -> Boundary: left (left edge of maze)")

    return new_walls


# D* Lite implementation (used for navigation between cells)
class DStarLite:
    def __init__(self, start, goal, rows, cols):
        self.start = start
        self.goal = goal
        self.rows = rows
        self.cols = cols
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.U = []
        self.walls = set()

        for x in range(cols):
            for y in range(rows):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.rhs[goal] = 0
        heapq.heappush(self.U, (self._calculate_key(goal), goal))

    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _calculate_key(self, s):
        return (min(self.g[s], self.rhs[s]) + self._heuristic(self.start, s) + self.km,
                min(self.g[s], self.rhs[s]))

    def _get_neighbors(self, s):
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = s[0] + dx, s[1] + dy
            if 0 <= nx < self.cols and 0 <= ny < self.rows:
                edge = tuple(sorted([s, (nx, ny)]))
                if edge not in self.walls:
                    neighbors.append((nx, ny))
        return neighbors

    def _cost(self, a, b):
        edge = tuple(sorted([a, b]))
        if edge in self.walls:
            return float('inf')
        return 1

    def _update_vertex(self, u):
        if u != self.goal:
            min_rhs = float('inf')
            for s in self._get_neighbors(u):
                min_rhs = min(min_rhs, self._cost(u, s) + self.g[s])
            self.rhs[u] = min_rhs

        self.U = [(k, v) for k, v in self.U if v != u]
        heapq.heapify(self.U)

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self._calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self._calculate_key(self.start) or
                          self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self._calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self._get_neighbors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = float('inf')
                self._update_vertex(u)
                for s in self._get_neighbors(u):
                    self._update_vertex(s)

    def get_next_move(self):
        if self.g[self.start] == float('inf'):
            return None

        best = None
        best_cost = float('inf')
        for s in self._get_neighbors(self.start):
            cost = self._cost(self.start, s) + self.g[s]
            if cost < best_cost:
                best_cost = cost
                best = s
        return best

    def update_walls(self, new_walls):
        self.km += self._heuristic(self.start, self.start)

        for wall in new_walls:
            if wall not in self.walls:
                self.walls.add(wall)
                a, b = wall
                if 0 <= a[0] < self.cols and 0 <= a[1] < self.rows:
                    self._update_vertex(a)
                if 0 <= b[0] < self.cols and 0 <= b[1] < self.rows:
                    self._update_vertex(b)

        self.compute_shortest_path()

    def move_to(self, new_start):
        self.km += self._heuristic(self.start, new_start)
        self.start = new_start

    def set_new_goal(self, new_goal):
        """Reset for new goal while keeping known walls"""
        old_walls = self.walls.copy()
        self.goal = new_goal
        self.km = 0
        self.U = []

        for x in range(self.cols):
            for y in range(self.rows):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.rhs[new_goal] = 0
        heapq.heappush(self.U, (self._calculate_key(new_goal), new_goal))
        self.walls = old_walls

        for wall in old_walls:
            a, b = wall
            if 0 <= a[0] < self.cols and 0 <= a[1] < self.rows:
                self._update_vertex(a)
            if 0 <= b[0] < self.cols and 0 <= b[1] < self.rows:
                self._update_vertex(b)

        self.compute_shortest_path()


def get_unvisited_neighbors(current, visited, walls, rows, cols):
    """Get unvisited neighbors that are accessible"""
    neighbors = []
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = current[0] + dx, current[1] + dy
        if 0 <= nx < cols and 0 <= ny < rows:
            if (nx, ny) not in visited:
                edge = tuple(sorted([current, (nx, ny)]))
                if edge not in walls:
                    neighbors.append((nx, ny))
    return neighbors


def find_nearest_unvisited(current, visited, walls, rows, cols):
    """Find nearest unvisited cell using simple distance"""
    unvisited = []
    for x in range(cols):
        for y in range(rows):
            if (x, y) not in visited:
                # Manhattan distance
                dist = abs(x - current[0]) + abs(y - current[1])
                unvisited.append(((x, y), dist))
    
    if not unvisited:
        return None
    
    # Sort by distance and return closest
    unvisited.sort(key=lambda x: x[1])
    return unvisited[0][0]


# ============ GET MAZE PARAMETERS ============
print("\n===== MazeMapper - Complete Maze Exploration =====")
print("Explores ALL cells systematically")
print("Lands and saves maze map to JSON")
print("=" * 50)

rows = int(input("\nEnter number of rows (Y-axis, forward): "))
cols = int(input("Enter number of columns (X-axis, left/right): "))

start_x = int(input(f"Enter start X (column 0 to {cols - 1}): "))
start_y = int(input(f"Enter start Y (row 0 to {rows - 1}): "))

end_x = int(input(f"Enter end X (column 0 to {cols - 1}): "))
end_y = int(input(f"Enter end Y (row 0 to {rows - 1}): "))

START = (start_x, start_y)
END = (end_x, end_y)

print(f"\nMaze: {rows} rows (Y) x {cols} columns (X)")
print(f"Start: (X={start_x}, Y={start_y})")
print(f"End: (X={end_x}, Y={end_y})")
print(f"Offset: X={OFFSET_X}cm, Y={OFFSET_Y}cm")

# Initialize D* Lite (for navigation between cells)
dstar = DStarLite(START, END, rows, cols)
dstar.compute_shortest_path()

# ============ SETUP MATPLOTLIB VISUALIZATION ============
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-0.1, cols * BLOCK + 0.1)
ax.set_ylim(-0.1, rows * BLOCK + 0.1)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (columns)')
ax.set_ylabel('Y (rows)')
title = ax.set_title('MazeMapper - Exploring All Cells')

# Draw grid
for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

# Draw start (green) and end (red) blocks
ax.add_patch(Rectangle((start_x * BLOCK + 0.02, start_y * BLOCK + 0.02),
                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4, label='Start'))
ax.add_patch(Rectangle((end_x * BLOCK + 0.02, end_y * BLOCK + 0.02),
                        BLOCK - 0.04, BLOCK - 0.04, facecolor='red', alpha=0.4, label='End'))

# Draw coordinate labels
for x in range(cols):
    for y in range(rows):
        ax.text(x * BLOCK + BLOCK/2, y * BLOCK + BLOCK/2, f"({x},{y})",
                ha='center', va='center', fontsize=8, color='gray')

# Visited blocks patches
visited_patches = {}

# Drone marker
drone_marker, = ax.plot([], [], 'bo', markersize=20, zorder=10, label='Drone')

# Path line
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Path')

# Store detected walls for drawing
detected_walls = set()

ax.legend(loc='upper right', fontsize=8)
plt.tight_layout()


def update_plot(current_x, current_y, obstacles, path_history, visited_count, total_cells):
    """Update the matplotlib visualization"""
    title.set_text(f'MazeMapper - Exploring All Cells | Pos: ({current_x},{current_y}) | Visited: {visited_count}/{total_cells}')

    drone_marker.set_data([current_x * BLOCK + BLOCK/2], [current_y * BLOCK + BLOCK/2])

    if path_history:
        px = [p[0] * BLOCK + BLOCK/2 for p in path_history]
        py = [p[1] * BLOCK + BLOCK/2 for p in path_history]
        path_line.set_data(px, py)

    # Draw new walls from obstacles
    for direction, has_obstacle in obstacles.items():
        if has_obstacle:
            if direction == 'forward':
                wall = tuple(sorted([(current_x, current_y), (current_x, current_y + 1)]))
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = (current_y + 1) * BLOCK
            elif direction == 'back':
                wall = tuple(sorted([(current_x, current_y), (current_x, current_y - 1)]))
                x1, x2 = current_x * BLOCK, (current_x + 1) * BLOCK
                y1 = y2 = current_y * BLOCK
            elif direction == 'right':
                wall = tuple(sorted([(current_x, current_y), (current_x + 1, current_y)]))
                x1 = x2 = (current_x + 1) * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            elif direction == 'left':
                wall = tuple(sorted([(current_x, current_y), (current_x - 1, current_y)]))
                x1 = x2 = current_x * BLOCK
                y1, y2 = current_y * BLOCK, (current_y + 1) * BLOCK
            else:
                continue

            if wall not in detected_walls:
                detected_walls.add(wall)
                ax.plot([x1, x2], [y1, y2], 'r-', linewidth=6, zorder=5)

    # Mark visited blocks
    if (current_x, current_y) not in visited_patches:
        patch = ax.add_patch(Rectangle((current_x * BLOCK + 0.05, current_y * BLOCK + 0.05),
                                        BLOCK - 0.1, BLOCK - 0.1,
                                        facecolor='lightblue', alpha=0.3, zorder=1))
        visited_patches[(current_x, current_y)] = patch

    plt.draw()
    plt.pause(0.1)


# ============ START FLIGHT ============

# Enable obstacle avoidance
api.single_fly_barrier_aircraft(True)

# Turn on QR code positioning
print("\nTurning on QR code positioning...")
api.Plane_cmd_switch_QR(0)
time.sleep(2)

# Takeoff
print("\nTaking off...")
api.single_fly_takeoff()
time.sleep(3)

# Move to starting position
print(f"\nMoving to starting position: Block ({start_x}, {start_y})")
move_to_block(start_x, start_y)
time.sleep(2)

current_x, current_y = start_x, start_y
visited = {(current_x, current_y)}
path_history = [(current_x, current_y)]
all_walls = set()

total_cells = rows * cols

# Initial plot update
update_plot(current_x, current_y, {}, path_history, len(visited), total_cells)

# ============ EXPLORE ALL CELLS ============
print("\n" + "=" * 50)
print("=== EXPLORING ALL CELLS ===")
print("=" * 50)

steps = 0
max_steps = total_cells * 10  # Safety limit

while len(visited) < total_cells and steps < max_steps:
    # Get verified obstacle readings
    obstacles = get_verified_obstacles()

    print(f"\nStep {steps + 1}: Position ({current_x},{current_y}), Visited: {len(visited)}/{total_cells}")
    print(f"Verified Obstacles: {obstacles}")

    # Convert obstacles to walls
    new_walls = get_walls_from_obstacles(obstacles, current_x, current_y, rows, cols)
    if new_walls:
        all_walls.update(new_walls)
        dstar.update_walls(new_walls)

    # Update visualization
    update_plot(current_x, current_y, obstacles, path_history, len(visited), total_cells)

    # Find next unvisited cell to explore
    # First, try unvisited neighbors (depth-first style exploration)
    unvisited_neighbors = get_unvisited_neighbors((current_x, current_y), visited, all_walls, rows, cols)
    
    if unvisited_neighbors:
        # Go to nearest unvisited neighbor
        next_pos = unvisited_neighbors[0]
        print(f"Moving to unvisited neighbor: ({next_pos[0]},{next_pos[1]})")
    else:
        # No unvisited neighbors, find nearest unvisited cell
        next_target = find_nearest_unvisited((current_x, current_y), visited, all_walls, rows, cols)
        
        if next_target is None:
            print("All cells visited!")
            break
        
        print(f"No unvisited neighbors. Navigating to nearest unvisited cell: ({next_target[0]},{next_target[1]})")
        
        # Use D* Lite to navigate to the unvisited cell
        dstar.set_new_goal(next_target)
        dstar.start = (current_x, current_y)
        dstar.compute_shortest_path()
        
        next_pos = dstar.get_next_move()
        
        if next_pos is None:
            print("Cannot reach any unvisited cells!")
            break

    # Move to next position
    next_x, next_y = next_pos
    move_to_block(next_x, next_y)

    current_x, current_y = next_x, next_y
    visited.add((current_x, current_y))
    path_history.append((current_x, current_y))
    steps += 1

    time.sleep(2)

# Final update
update_plot(current_x, current_y, {}, path_history, len(visited), total_cells)

print(f"\n✓ Exploration complete! Visited {len(visited)}/{total_cells} cells in {steps} steps")

# ============ NAVIGATE TO END ============
if (current_x, current_y) != (end_x, end_y):
    print("\n" + "=" * 50)
    print("=== NAVIGATING TO END POSITION ===")
    print("=" * 50)
    
    dstar.set_new_goal(END)
    dstar.start = (current_x, current_y)
    dstar.compute_shortest_path()
    
    while (current_x, current_y) != (end_x, end_y) and steps < max_steps:
        next_pos = dstar.get_next_move()
        
        if next_pos is None:
            print("Cannot reach end position!")
            break
        
        next_x, next_y = next_pos
        print(f"Moving to: ({next_x},{next_y})")
        
        move_to_block(next_x, next_y)
        
        current_x, current_y = next_x, next_y
        dstar.move_to(next_pos)
        path_history.append((current_x, current_y))
        steps += 1
        
        update_plot(current_x, current_y, {}, path_history, len(visited), total_cells)
        time.sleep(2)
    
    if (current_x, current_y) == (end_x, end_y):
        print(f"\n✓ Reached end position!")

time.sleep(2)

# ============ SAVE MAZE MAP TO JSON ============
print("\n" + "=" * 50)
print("=== SAVING MAZE MAP ===")
print("=" * 50)

# Convert walls to JSON-serializable format
walls_list = [[[w[0][0], w[0][1]], [w[1][0], w[1][1]]] for w in all_walls]

# Create maze data (without optimal path or waypoints)
maze_data = {
    "maze_info": {
        "rows": rows,
        "cols": cols,
        "block_size_cm": BLOCK_SIZE,
        "flight_height_cm": FLIGHT_HEIGHT,
        "offset_x": OFFSET_X,
        "offset_y": OFFSET_Y
    },
    "start": [start_x, start_y],
    "end": [end_x, end_y],
    "walls": walls_list,
    "exploration_stats": {
        "total_steps": steps,
        "cells_visited": len(visited),
        "total_cells": total_cells,
        "walls_detected": len(all_walls),
        "exploration_path": [[p[0], p[1]] for p in path_history]
    }
}

# Save to JSON file
json_filename = "maze_map.json"
with open(json_filename, 'w') as f:
    json.dump(maze_data, f, indent=2)

print(f"\n✓ Maze map saved to: {json_filename}")
print(f"  Cells visited: {len(visited)}/{total_cells}")
print(f"  Walls detected: {len(all_walls)}")
print(f"  Total steps: {steps}")

# ============ LANDING ============
print("\n" + "=" * 50)
print("=== LANDING ===")
print("=" * 50)

title.set_text(f"MazeMapper - Complete! Saved to {json_filename}")
plt.draw()

api.single_fly_touchdown()
print("✓ Landed successfully!")

# ============ SUMMARY ============
print("\n" + "=" * 50)
print("========== SUMMARY ==========")
print("=" * 50)
print(f"Maze: {rows} x {cols}")
print(f"Start: ({start_x},{start_y}), End: ({end_x},{end_y})")
print(f"Total exploration steps: {steps}")
print(f"Cells visited: {len(visited)}/{total_cells} ({100*len(visited)/total_cells:.1f}%)")
print(f"Walls detected: {len(all_walls)}")
print(f"Map saved to: {json_filename}")
print("=" * 50)

# Keep plot open
plt.ioff()
plt.show()
"""
GoHomeTurbo - D* Lite maze navigation for real drone
Phase 1: Explore to end (discovering walls)
Phase 2: Return home using known map
Phase 3: FAST optimal path to end (continuous flight through waypoints)

With real-time matplotlib visualization
"""

import pyhula
import time
import heapq
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
SPEED = 50  # Normal speed (0-100 cm/s)
MAX_SPEED = 100  # Maximum speed for fast flight
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


def move_to_block_fast(block_x, block_y):
    """Move to a block position (same as normal - API doesn't support speed parameter)"""
    abs_x, abs_y = block_to_cm(block_x, block_y)
    # Note: pyhula API doesn't support speed parameter for straight_flight
    # The drone will fly at its default speed
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


# D* Lite implementation
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

    def get_full_path(self):
        """Get complete optimal path from current start to goal"""
        path = [self.start]
        current = self.start
        while current != self.goal:
            next_pos = None
            best_cost = float('inf')
            for s in self._get_neighbors(current):
                cost = self._cost(current, s) + self.g[s]
                if cost < best_cost:
                    best_cost = cost
                    next_pos = s
            if next_pos is None:
                return None
            path.append(next_pos)
            current = next_pos
        return path

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

        # Re-apply walls
        for wall in old_walls:
            a, b = wall
            if 0 <= a[0] < self.cols and 0 <= a[1] < self.rows:
                self._update_vertex(a)
            if 0 <= b[0] < self.cols and 0 <= b[1] < self.rows:
                self._update_vertex(b)

        self.compute_shortest_path()


def simplify_path(path):
    """
    Simplify path by removing intermediate points on straight lines.
    Only keep waypoints where direction changes (corners).
    """
    if len(path) <= 2:
        return path

    simplified = [path[0]]

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_pt = path[i + 1]

        # Calculate direction vectors
        dir1 = (curr[0] - prev[0], curr[1] - prev[1])
        dir2 = (next_pt[0] - curr[0], next_pt[1] - curr[1])

        # If direction changes, this is a corner - keep it
        if dir1 != dir2:
            simplified.append(curr)

    simplified.append(path[-1])
    return simplified


def get_walls_from_obstacles(obstacles, current_x, current_y, rows, cols):
    """Convert obstacle detection to wall format for D* Lite

    Note: Boundary detections (edges of maze) are logged but not added as walls
    since they're implicit in the maze bounds.
    """
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


# ============ GET MAZE PARAMETERS ============
print("\n===== GoHomeTurbo - D* Lite Navigation =====")
print("Phase 1: Explore to end")
print("Phase 2: Return home")
print("Phase 3: FAST optimal path to end")
print("=" * 45)

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

# Initialize D* Lite
dstar = DStarLite(START, END, rows, cols)
dstar.compute_shortest_path()

# ============ SETUP MATPLOTLIB VISUALIZATION ============
plt.ion()  # Interactive mode
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-0.1, cols * BLOCK + 0.1)
ax.set_ylim(-0.1, rows * BLOCK + 0.1)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (columns)')
ax.set_ylabel('Y (rows)')
title = ax.set_title('GoHomeTurbo - Phase 1: Exploring to End')

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
drone_direction = None

# Path lines
path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Path')
optimal_line, = ax.plot([], [], 'g--', linewidth=3, alpha=0.7, label='Optimal')
fast_line, = ax.plot([], [], 'orange', linewidth=4, alpha=0.8, label='Fast path')

# Store detected walls for drawing
detected_walls = set()

# Distance text display
distance_texts = {}
for x in range(cols):
    for y in range(rows):
        g_val = dstar.g[(x, y)]
        txt_str = str(int(g_val)) if g_val != float('inf') else '∞'
        txt = ax.text(x * BLOCK + BLOCK/2, y * BLOCK + 0.1, txt_str,
                      ha='center', va='bottom', fontsize=10, fontweight='bold',
                      color='blue', alpha=0.7)
        distance_texts[(x, y)] = txt

ax.legend(loc='upper right', fontsize=8)
plt.tight_layout()


def update_distance_display():
    """Update distance numbers based on D* Lite g values"""
    for x in range(cols):
        for y in range(rows):
            g_val = dstar.g[(x, y)]
            txt_str = str(int(g_val)) if g_val != float('inf') else '∞'
            distance_texts[(x, y)].set_text(txt_str)


def update_plot(current_x, current_y, obstacles, path_history, phase_name):
    """Update the matplotlib visualization"""
    global drone_direction

    # Update title
    title.set_text(f'GoHomeTurbo - {phase_name} | Pos: ({current_x},{current_y})')

    # Update drone position
    drone_marker.set_data([current_x * BLOCK + BLOCK/2], [current_y * BLOCK + BLOCK/2])

    # Update path
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


def show_optimal_path(path):
    """Display the optimal path on the plot"""
    if path:
        ox = [p[0] * BLOCK + BLOCK/2 for p in path]
        oy = [p[1] * BLOCK + BLOCK/2 for p in path]
        optimal_line.set_data(ox, oy)
        plt.draw()
        plt.pause(0.1)


def show_fast_path(waypoints):
    """Display the fast waypoint path on the plot"""
    if waypoints:
        fx = [p[0] * BLOCK + BLOCK/2 for p in waypoints]
        fy = [p[1] * BLOCK + BLOCK/2 for p in waypoints]
        fast_line.set_data(fx, fy)

        # Mark waypoints with orange circles
        for wp in waypoints:
            ax.plot(wp[0] * BLOCK + BLOCK/2, wp[1] * BLOCK + BLOCK/2,
                    'o', color='orange', markersize=12, zorder=8)

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

# Initial plot update
update_plot(current_x, current_y, {}, path_history, "Phase 1: Exploring")

# ============ PHASE 1: Explore to End ============
print("\n" + "=" * 50)
print("=== PHASE 1: Exploring to End ===")
print("=" * 50)

steps = 0
max_steps = rows * cols * 4  # Safety limit

while (current_x, current_y) != (end_x, end_y) and steps < max_steps:
    # Get verified obstacle readings (checks multiple times)
    obstacles = get_verified_obstacles()

    print(f"\nPosition: ({current_x},{current_y}), Verified Obstacles: {obstacles}")

    # Update visualization
    update_plot(current_x, current_y, obstacles, path_history, "Phase 1: Exploring")

    # Convert obstacles to walls and update D* Lite
    new_walls = get_walls_from_obstacles(obstacles, current_x, current_y, rows, cols)
    if new_walls:
        dstar.update_walls(new_walls)
        update_distance_display()

    # Get next move from D* Lite
    next_pos = dstar.get_next_move()

    if next_pos is None:
        print("No path found!")
        break

    # Move to next position
    next_x, next_y = next_pos
    print(f"Moving to: ({next_x},{next_y})")

    move_to_block(next_x, next_y)

    current_x, current_y = next_x, next_y
    dstar.move_to(next_pos)
    visited.add((current_x, current_y))
    path_history.append((current_x, current_y))
    steps += 1

    time.sleep(2)

# Final update for Phase 1
update_plot(current_x, current_y, {}, path_history, "Phase 1: Complete")

if (current_x, current_y) == (end_x, end_y):
    print(f"\n✓ Reached end in {steps} steps!")
else:
    print(f"\n✗ Stopped at ({current_x},{current_y})")

time.sleep(2)

# ============ PHASE 2: Return Home ============
print("\n" + "=" * 50)
print("=== PHASE 2: Returning Home ===")
print("=" * 50)

# Reset D* Lite for return journey (goal is now START)
dstar.set_new_goal(START)
dstar.move_to((current_x, current_y))
update_distance_display()

# Clear path for Phase 2
path_line.set_data([], [])
path_history = [(current_x, current_y)]

return_steps = 0

while (current_x, current_y) != (start_x, start_y) and return_steps < max_steps:
    # Get verified obstacle readings
    obstacles = get_verified_obstacles()

    print(f"\nPosition: ({current_x},{current_y})")

    # Update visualization
    update_plot(current_x, current_y, obstacles, path_history, "Phase 2: Returning Home")

    # Update walls if any new ones detected
    new_walls = get_walls_from_obstacles(obstacles, current_x, current_y, rows, cols)
    if new_walls:
        dstar.update_walls(new_walls)
        update_distance_display()

    # Get next move
    next_pos = dstar.get_next_move()

    if next_pos is None:
        print("No path found!")
        break

    # Move to next position
    next_x, next_y = next_pos
    print(f"Moving to: ({next_x},{next_y})")

    move_to_block(next_x, next_y)

    current_x, current_y = next_x, next_y
    dstar.move_to(next_pos)
    path_history.append((current_x, current_y))
    return_steps += 1

    time.sleep(2)

# Final update for Phase 2
update_plot(current_x, current_y, {}, path_history, "Phase 2: Complete")

if (current_x, current_y) == (start_x, start_y):
    print(f"\n✓ Returned home in {return_steps} steps!")
else:
    print(f"\n✗ Stopped at ({current_x},{current_y})")

time.sleep(2)

# ============ PHASE 3: FAST Optimal Path to End ============
print("\n" + "=" * 50)
print("=== PHASE 3: FAST Optimal Path to End ===")
print("=" * 50)

# Reset D* Lite for optimal journey (goal is END again)
dstar.set_new_goal(END)
dstar.move_to((current_x, current_y))
update_distance_display()

# Get optimal path and simplify to waypoints
optimal_path = dstar.get_full_path()

if optimal_path:
    # Show optimal path
    show_optimal_path(optimal_path)

    # Simplify path to only corners/waypoints
    waypoints = simplify_path(optimal_path)

    # Show fast path
    show_fast_path(waypoints)

    print(f"\nFull optimal path ({len(optimal_path)} points): {optimal_path}")
    print(f"Simplified waypoints ({len(waypoints)} points): {waypoints}")

    # Calculate total distance
    total_distance = 0
    for i in range(len(optimal_path) - 1):
        dx = optimal_path[i + 1][0] - optimal_path[i][0]
        dy = optimal_path[i + 1][1] - optimal_path[i][1]
        total_distance += abs(dx) + abs(dy)

    print(f"Path length: {total_distance} blocks ({total_distance * BLOCK_SIZE} cm)")

    title.set_text("Phase 3: FAST Flight - Starting...")
    plt.draw()
    plt.pause(1)

    # Clear path for Phase 3
    path_line.set_data([], [])
    path_history = [(current_x, current_y)]

    # Execute FAST continuous flight through waypoints
    print("\n--- Executing FAST flight at MAX SPEED ---")

    for i, waypoint in enumerate(waypoints[1:], 1):  # Skip first (current position)
        wp_x, wp_y = waypoint
        abs_x, abs_y = block_to_cm(wp_x, wp_y)

        print(f"Flying to waypoint {i}/{len(waypoints)-1}: ({wp_x},{wp_y}) -> ({abs_x}cm, {abs_y}cm)")

        # Fly at maximum speed to next waypoint
        move_to_block_fast(wp_x, wp_y)

        current_x, current_y = wp_x, wp_y
        path_history.append((current_x, current_y))

        # Update plot
        title.set_text(f"Phase 3: FAST Flight - Waypoint {i}/{len(waypoints)-1}")
        drone_marker.set_data([current_x * BLOCK + BLOCK/2], [current_y * BLOCK + BLOCK/2])
        px = [p[0] * BLOCK + BLOCK/2 for p in path_history]
        py = [p[1] * BLOCK + BLOCK/2 for p in path_history]
        path_line.set_data(px, py)
        plt.draw()
        plt.pause(0.5)

        time.sleep(1)  # Shorter delay for fast flight

    print(f"\n✓ FAST flight complete!")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Flight commands: {len(waypoints) - 1} (vs {len(optimal_path) - 1} block-by-block)")

else:
    print("No path found!")
    waypoints = []
    optimal_path = []

# ============ SUMMARY ============
title.set_text("Mission Complete!")
plt.draw()

print("\n" + "=" * 50)
print("========== SUMMARY ==========")
print("=" * 50)
print(f"Phase 1 (Explore to end): {steps} steps")
print(f"Phase 2 (Return home):    {return_steps} steps")
if optimal_path:
    print(f"Phase 3 (FAST optimal):   {len(waypoints) - 1} flight commands")
    print(f"         (vs {len(optimal_path) - 1} block-by-block)")
print(f"Total blocks explored: {len(visited)}")
print(f"Total walls detected: {len(detected_walls)}")
print("=" * 50)

# Land
print("\nLanding...")
api.single_fly_touchdown()
print("Mission complete!")

# Keep plot open
plt.ioff()
plt.show()
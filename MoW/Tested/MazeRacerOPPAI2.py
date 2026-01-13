"""
MazeRacerOptimized - Fast maze navigation with adjustable aggressiveness
Loads maze map from JSON and executes optimal waypoint flight
User can adjust pause times for faster/more aggressive flight
"""

import pyhula
import time
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os

# Initialize and connect
api = pyhula.UserApi()
if not api.connect():
    print("Connection failed")
    exit()

print("Connected successfully")

# ============ LOAD MAZE MAP ============
print("\n===== MazeRacerOptimized - Fast Maze Navigation =====")
print("Adjustable aggressiveness for faster flights")
print("=" * 52)

json_filename = "maze_map.json"

if not os.path.exists(json_filename):
    print(f"\nError: File '{json_filename}' not found!")
    print("Run MazeMapper.py first to create the maze map.")
    exit()

with open(json_filename, 'r') as f:
    maze_data = json.load(f)

# Extract maze info
maze_info = maze_data["maze_info"]
rows = maze_info["rows"]
cols = maze_info["cols"]
BLOCK_SIZE = maze_info["block_size_cm"]
FLIGHT_HEIGHT = maze_info["flight_height_cm"]
OFFSET_X = maze_info["offset_x"]
OFFSET_Y = maze_info["offset_y"]
BLOCK = 0.6  # Plot block size

FIRST_BLOCK_CENTER_X = 15 + OFFSET_X
FIRST_BLOCK_CENTER_Y = 15 + OFFSET_Y

# Load walls
walls = set()
for wall in maze_data["walls"]:
    walls.add(tuple(sorted([tuple(wall[0]), tuple(wall[1])])))

# Load saved start/end and paths
saved_start = tuple(maze_data["start"])
saved_end = tuple(maze_data["end"])
optimal_path = [tuple(p) for p in maze_data["optimal_path"]]
waypoints = [tuple(p) for p in maze_data["waypoints"]]

print(f"\n✓ Loaded maze map: {rows} x {cols}")
print(f"  Walls: {len(walls)}")
print(f"  Saved route: {saved_start} -> {saved_end}")
print(f"  Waypoints: {waypoints}")

# ============ AGGRESSIVENESS SETTINGS ============
print("\n" + "=" * 52)
print("AGGRESSIVENESS SETTINGS")
print("=" * 52)
print("""
Level 1 - Normal:     Balanced speed
Level 2 - Fast:       Reduced pauses
Level 3 - Aggressive: Minimal pauses
Level 4 - TURBO:      No pauses
Level 5 - INSANE:     Negative delays (overlapping commands!)
""")

while True:
    try:
        level = int(input("Enter aggressiveness level (1-5): "))
        if 1 <= level <= 5:
            break
        print("Please enter a number between 1 and 5")
    except ValueError:
        print("Please enter a valid number")

# Define timing settings based on aggressiveness
TIMING_PRESETS = {
    1: {  # Normal
        "name": "Normal",
        "takeoff_wait": 2.5,
        "start_position_wait": 1.5,
        "waypoint_delay": 1.0,
        "plot_pause": 0.3,
    },
    2: {  # Fast
        "name": "Fast",
        "takeoff_wait": 2.0,
        "start_position_wait": 1.0,
        "waypoint_delay": 0.5,
        "plot_pause": 0.2,
    },
    3: {  # Aggressive
        "name": "Aggressive",
        "takeoff_wait": 1.5,
        "start_position_wait": 0.5,
        "waypoint_delay": 0.2,
        "plot_pause": 0.1,
    },
    4: {  # TURBO
        "name": "TURBO",
        "takeoff_wait": 0.5,
        "start_position_wait": 0.1,
        "waypoint_delay": 0.0,
        "plot_pause": 0.01,
    },
    5: {  # INSANE
        "name": "INSANE",
        "takeoff_wait": 0.001,
        "start_position_wait": 0.001,
        "waypoint_delay": 0.0,
        "plot_pause": 0.001,
    },
}

timing = TIMING_PRESETS[level]

print(f"\n✓ Selected: Level {level} - {timing['name']}")
print(f"  Takeoff wait:        {timing['takeoff_wait']}s")
print(f"  Start position wait: {timing['start_position_wait']}s")
print(f"  Waypoint delay:      {timing['waypoint_delay']}s")
print(f"  Plot pause:          {timing['plot_pause']}s")

if level >= 4:
    print("\n⚠️  WARNING: High aggressiveness may cause instability!")
if level == 5:
    print("🔥 INSANE MODE: Maximum speed, minimal safety!")


def block_to_cm(block_x, block_y):
    """Convert block coordinates to absolute cm coordinates"""
    abs_x = FIRST_BLOCK_CENTER_X + (block_x * BLOCK_SIZE)
    abs_y = FIRST_BLOCK_CENTER_Y + (block_y * BLOCK_SIZE)
    return abs_x, abs_y


def move_to_block(block_x, block_y):
    """Move to a block position"""
    abs_x, abs_y = block_to_cm(block_x, block_y)
    api.single_fly_straight_flight(abs_x, abs_y, FLIGHT_HEIGHT)


# ============ GET RACE PARAMETERS ============
print("\n" + "-" * 52)

# Use saved start and end from maze map
start_x, start_y = saved_start
end_x, end_y = saved_end

print(f"\nUsing saved route:")
print(f"  Start: ({start_x}, {start_y})")
print(f"  End: ({end_x}, {end_y})")
print(f"  Path length: {len(optimal_path) - 1} blocks")
print(f"  Waypoints: {len(waypoints)}")

if not optimal_path:
    print("\nError: No path found in maze map!")
    exit()

# ============ SETUP VISUALIZATION ============
plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-0.1, cols * BLOCK + 0.1)
ax.set_ylim(-0.1, rows * BLOCK + 0.1)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (columns)')
ax.set_ylabel('Y (rows)')
title = ax.set_title(f'MazeRacerOptimized - Level {level} ({timing["name"]})')

# Draw grid
for i in range(cols + 1):
    ax.axvline(i * BLOCK, color='black', linewidth=2)
for i in range(rows + 1):
    ax.axhline(i * BLOCK, color='black', linewidth=2)

# Draw walls
for wall in walls:
    p1, p2 = wall
    if p1[0] == p2[0]:  # Same X - horizontal wall between cells
        wall_x = p1[0] * BLOCK
        wall_y = max(p1[1], p2[1]) * BLOCK
        ax.plot([wall_x, wall_x + BLOCK], [wall_y, wall_y], 'r-', linewidth=6)
    else:  # Same Y - vertical wall between cells
        wall_x = max(p1[0], p2[0]) * BLOCK
        wall_y = p1[1] * BLOCK
        ax.plot([wall_x, wall_x], [wall_y, wall_y + BLOCK], 'r-', linewidth=6)

# Draw start (green) and end (red)
ax.add_patch(Rectangle((start_x * BLOCK + 0.02, start_y * BLOCK + 0.02),
                       BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4, label='Start'))
ax.add_patch(Rectangle((end_x * BLOCK + 0.02, end_y * BLOCK + 0.02),
                       BLOCK - 0.04, BLOCK - 0.04, facecolor='red', alpha=0.4, label='End'))

# Draw coordinate labels
for x in range(cols):
    for y in range(rows):
        ax.text(x * BLOCK + BLOCK / 2, y * BLOCK + BLOCK / 2, f"({x},{y})",
                ha='center', va='center', fontsize=8, color='gray')

# Draw optimal path
path_x = [p[0] * BLOCK + BLOCK / 2 for p in optimal_path]
path_y = [p[1] * BLOCK + BLOCK / 2 for p in optimal_path]
ax.plot(path_x, path_y, 'g--', linewidth=2, alpha=0.5, label='Full path')

# Draw waypoints
wp_x = [p[0] * BLOCK + BLOCK / 2 for p in waypoints]
wp_y = [p[1] * BLOCK + BLOCK / 2 for p in waypoints]
ax.plot(wp_x, wp_y, 'orange', linewidth=4, alpha=0.8, label='Waypoints')
for i, wp in enumerate(waypoints):
    ax.plot(wp[0] * BLOCK + BLOCK / 2, wp[1] * BLOCK + BLOCK / 2,
            'o', color='orange', markersize=12, zorder=8)
    ax.text(wp[0] * BLOCK + BLOCK / 2 + 0.1, wp[1] * BLOCK + BLOCK / 2 + 0.1,
            str(i), fontsize=10, fontweight='bold', color='darkorange')

# Drone marker
drone_marker, = ax.plot([], [], 'bo', markersize=20, zorder=10, label='Drone')

# Flight path line
flight_line, = ax.plot([], [], 'b-', linewidth=3, alpha=0.7)

ax.legend(loc='upper right', fontsize=8)
plt.tight_layout()
plt.draw()
plt.pause(0.5)

# ============ CONFIRM AND START ============
print("\n" + "=" * 52)
input(f"Press ENTER to start the race (Level {level} - {timing['name']})...")
print("=" * 52)

# ============ START FLIGHT ============

# Enable obstacle avoidance (as safety)
api.single_fly_barrier_aircraft(True)

# Turn on QR code positioning
print("\nTurning on QR code positioning...")
api.Plane_cmd_switch_QR(0)
time.sleep(2)

# Takeoff
print("\nTaking off...")
api.single_fly_takeoff()
time.sleep(timing["takeoff_wait"])

# Move to starting position
print(f"\nMoving to start position: ({start_x}, {start_y})")
move_to_block(start_x, start_y)
time.sleep(timing["start_position_wait"])

current_x, current_y = start_x, start_y
flight_history = [(current_x, current_y)]

# Update drone position
drone_marker.set_data([current_x * BLOCK + BLOCK / 2], [current_y * BLOCK + BLOCK / 2])
title.set_text(f'MazeRacerOptimized - Level {level} | At Start ({current_x},{current_y})')
plt.draw()
plt.pause(timing["plot_pause"])

# ============ EXECUTE FAST FLIGHT ============
print("\n" + "=" * 52)
print(f"=== RACING - Level {level} ({timing['name']}) ===")
print("=" * 52)

race_start_time = time.time()

for i, waypoint in enumerate(waypoints[1:], 1):  # Skip first (start position)
    wp_x, wp_y = waypoint
    abs_x, abs_y = block_to_cm(wp_x, wp_y)

    print(f"\nWaypoint {i}/{len(waypoints) - 1}: ({wp_x},{wp_y}) -> ({abs_x}cm, {abs_y}cm)")

    # Update title
    title.set_text(f'MazeRacerOptimized - Level {level} | Waypoint {i}/{len(waypoints) - 1}')
    plt.draw()

    # Fly to waypoint
    move_to_block(wp_x, wp_y)

    # Update position
    current_x, current_y = wp_x, wp_y
    flight_history.append((current_x, current_y))

    # Update visualization
    drone_marker.set_data([current_x * BLOCK + BLOCK / 2], [current_y * BLOCK + BLOCK / 2])
    fx = [p[0] * BLOCK + BLOCK / 2 for p in flight_history]
    fy = [p[1] * BLOCK + BLOCK / 2 for p in flight_history]
    flight_line.set_data(fx, fy)
    plt.draw()
    plt.pause(timing["plot_pause"])

    # Waypoint delay (this is the main speed control)
    if timing["waypoint_delay"] > 0:
        time.sleep(timing["waypoint_delay"])

race_end_time = time.time()
race_duration = race_end_time - race_start_time

# ============ FINISH ============
print("\n" + "=" * 52)
print("=== RACE COMPLETE ===")
print("=" * 52)

title.set_text(f'MazeRacerOptimized - FINISHED! Time: {race_duration:.1f}s (Level {level})')
plt.draw()

print(f"\n✓ Reached destination: ({end_x},{end_y})")
print(f"  Aggressiveness: Level {level} ({timing['name']})")
print(f"  Race time: {race_duration:.1f} seconds")
print(f"  Waypoints flown: {len(waypoints) - 1}")
print(f"  Distance: {len(optimal_path) - 1} blocks ({(len(optimal_path) - 1) * BLOCK_SIZE} cm)")

# ============ LANDING ============
print("\nLanding...")
api.single_fly_touchdown()
print("✓ Landed successfully!")

# ============ SUMMARY ============
print("\n" + "=" * 52)
print("========== RACE SUMMARY ==========")
print("=" * 52)
print(f"Aggressiveness: Level {level} ({timing['name']})")
print(f"Start: ({start_x},{start_y})")
print(f"End: ({end_x},{end_y})")
print(f"Path length: {len(optimal_path) - 1} blocks")
print(f"Waypoints: {len(waypoints)}")
print(f"Flight commands: {len(waypoints) - 1}")
print(f"Total time: {race_duration:.1f} seconds")
print("=" * 52)

# Keep plot open
plt.ioff()
plt.show()
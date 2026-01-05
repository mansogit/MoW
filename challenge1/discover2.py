import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ---------------------------------------------------------
# Maze configuration
# ---------------------------------------------------------
ROWS, COLS = 5, 5
CELL_SIZE_CM = 30

DIRS = {
    "up": (-1, 0),
    "down": (1, 0),
    "left": (0, -1),
    "right": (0, 1)
}

OPPOSITE = {"up": "down", "down": "up", "left": "right", "right": "left"}


# ---------------------------------------------------------
# Maze creation
# ---------------------------------------------------------
def empty_maze():
    return [[{"up": True, "down": True, "left": True, "right": True}
             for _ in range(COLS)] for _ in range(ROWS)]


def generate_maze(r, c, visited, maze):
    visited.add((r, c))
    directions = list(DIRS.keys())
    random.shuffle(directions)

    for d in directions:
        dr, dc = DIRS[d]
        nr, nc = r + dr, c + dc

        if 0 <= nr < ROWS and 0 <= nc < COLS and (nr, nc) not in visited:
            maze[r][c][d] = False
            maze[nr][nc][OPPOSITE[d]] = False
            generate_maze(nr, nc, visited, maze)


# ---------------------------------------------------------
# Robot exploration with explicit backtracking (1-step moves)
# ---------------------------------------------------------
def explore_maze_with_backtracking(start_r, start_c, maze):
    """
    DFS-style exploration.
    Robot moves one cell at a time.
    Path includes forward moves and backtracking steps.
    """
    visited = set()
    path = []

    stack = [(start_r, start_c)]
    visited.add((start_r, start_c))
    path.append((start_r, start_c))

    while stack:
        r, c = stack[-1]  # look at current cell (top of stack)

        # Find all unvisited neighbors reachable through open walls
        neighbors = []
        for d in DIRS:
            if not maze[r][c][d]:
                dr, dc = DIRS[d]
                nr, nc = r + dr, c + dc
                if (nr, nc) not in visited:
                    neighbors.append((nr, nc))

        if neighbors:
            # Choose one neighbor to visit
            nr, nc = neighbors[0]
            stack.append((nr, nc))
            visited.add((nr, nc))

            # Robot moves one step: record this move
            path.append((nr, nc))
        else:
            # Dead end: backtrack one step if possible
            stack.pop()
            if stack:
                back_r, back_c = stack[-1]
                # Robot moves back one step: record this move
                path.append((back_r, back_c))

    return path, visited


# ---------------------------------------------------------
# BFS shortest path between two cells
# ---------------------------------------------------------
def shortest_path(maze, start, goal):
    queue = deque([start])
    parent = {start: None}
    visited = {start}

    while queue:
        r, c = queue.popleft()

        if (r, c) == goal:
            break

        for d in DIRS:
            if not maze[r][c][d]:
                dr, dc = DIRS[d]
                nr, nc = r + dr, c + dc
                if (nr, nc) not in visited:
                    visited.add((nr, nc))
                    parent[(nr, nc)] = (r, c)
                    queue.append((nr, nc))

    # Reconstruct path from goal back to start
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path


# ---------------------------------------------------------
# MAIN LOGIC
# ---------------------------------------------------------
maze = empty_maze()

start = (random.randint(0, ROWS - 1), random.randint(0, COLS - 1))
target = (random.randint(0, ROWS - 1), random.randint(0, COLS - 1))
while target == start:
    target = (random.randint(0, ROWS - 1), random.randint(0, COLS - 1))

generate_maze(start[0], start[1], set(), maze)

# Exploration with backtracking (one-step moves, revisits allowed)
exploration_path, visited_cells = explore_maze_with_backtracking(start[0], start[1], maze)

# Sanity check: all cells visited
assert len(visited_cells) == ROWS * COLS, "Not all cells were visited during exploration!"

# After exploration, go to target with shortest path
current_pos = exploration_path[-1]
to_target_path = shortest_path(maze, current_pos, target)

# Avoid duplicating last position: skip the first cell of to_target_path if same
if to_target_path and to_target_path[0] == current_pos:
    to_target_path = to_target_path[1:]

full_path = exploration_path + to_target_path


# ---------------------------------------------------------
# Animation setup
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(6, 6))

robot_dot, = ax.plot([], [], "ro", markersize=10)
path_line, = ax.plot([], [], "b-", linewidth=2)

line_x, line_y = [], []


def draw_maze():
    for r in range(ROWS):
        for c in range(COLS):
            x, y = c, ROWS - r - 1

            if maze[r][c]["up"]:
                ax.plot([x, x+1], [y+1, y+1], color="black")
            if maze[r][c]["down"]:
                ax.plot([x, x+1], [y, y], color="black")
            if maze[r][c]["left"]:
                ax.plot([x, x], [y, y+1], color="black")
            if maze[r][c]["right"]:
                ax.plot([x+1, x+1], [y, y+1], color="black")

    # highlight target cell
    tx, ty = target[1], ROWS - target[0] - 1
    ax.add_patch(plt.Rectangle((tx, ty), 1, 1, color="yellow", alpha=0.3))


def init():
    draw_maze()
    ax.set_xlim(0, COLS)
    ax.set_ylim(0, ROWS)
    ax.set_aspect("equal")
    ax.axis("off")
    return robot_dot, path_line


def update(frame):
    r, c = full_path[frame]
    x = c + 0.5
    y = ROWS - r - 1 + 0.5

    robot_dot.set_data(x, y)

    line_x.append(x)
    line_y.append(y)
    path_line.set_data(line_x, line_y)

    return robot_dot, path_line


ani = animation.FuncAnimation(
    fig, update, frames=len(full_path),
    init_func=init, interval=400, blit=True
)

plt.show()

# Optional: total physical distance travelled
total_distance_cm = (len(full_path) - 1) * CELL_SIZE_CM
print("Start:", start)
print("Target:", target)
print("Number of steps:", len(full_path) - 1)
print("Approximate distance travelled:", total_distance_cm, "cm")

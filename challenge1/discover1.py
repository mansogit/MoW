import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

ROWS, COLS = 5, 5

DIRS = {
    "up": (-1, 0),
    "down": (1, 0),
    "left": (0, -1),
    "right": (0, 1)
}

OPPOSITE = {"up": "down", "down": "up", "left": "right", "right": "left"}

# ---------------------------------------------------------
# OPTIONAL: Replace this with your own imported maze
# ---------------------------------------------------------
imported_maze = None
# imported_maze = [...]  # <-- paste your maze here
# ---------------------------------------------------------

def empty_maze():
    return [[{"up": True, "down": True, "left": True, "right": True}
             for _ in range(COLS)] for _ in range(ROWS)]

maze = empty_maze()

# Random maze generation
def generate_maze(r, c, visited):
    visited.add((r, c))
    directions = list(DIRS.keys())
    random.shuffle(directions)

    for d in directions:
        dr, dc = DIRS[d]
        nr, nc = r + dr, c + dc

        if 0 <= nr < ROWS and 0 <= nc < COLS and (nr, nc) not in visited:
            maze[r][c][d] = False
            maze[nr][nc][OPPOSITE[d]] = False
            generate_maze(nr, nc, visited)

# Load imported maze or generate one
if imported_maze is not None:
    maze = imported_maze
    start_r = random.randint(0, ROWS - 1)
    start_c = random.randint(0, COLS - 1)
else:
    start_r = random.randint(0, ROWS - 1)
    start_c = random.randint(0, COLS - 1)
    generate_maze(start_r, start_c, set())

# ---------------------------------------------------------
# Build robot movement path with 1‑step backtracking
# ---------------------------------------------------------
path = []
dead_end_flags = []  # True when robot is at a dead end

visited = set()
stack = [(start_r, start_c)]

while stack:
    r, c = stack.pop()
    path.append((r, c))

    visited.add((r, c))

    # Find all possible unvisited neighbors
    neighbors = []
    for d in DIRS:
        dr, dc = DIRS[d]
        if not maze[r][c][d]:
            nr, nc = r + dr, c + dc
            if (nr, nc) not in visited:
                neighbors.append((nr, nc))

    if len(neighbors) == 0:
        # Dead end → mark it
        dead_end_flags.append(True)

        # Backtrack exactly 1 step if possible
        if len(path) > 1:
            prev = path[-2]
            path.append(prev)  # move back one cell
            dead_end_flags.append(False)
    else:
        dead_end_flags.append(False)
        # Push neighbors to stack (DFS)
        for n in neighbors:
            stack.append(n)

# ---------------------------------------------------------
# Plotting
# ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(5, 5))

def draw_maze():
    for r in range(ROWS):
        for c in range(COLS):
            x = c
            y = ROWS - r - 1

            if maze[r][c]["up"]:
                ax.plot([x, x+1], [y+1, y+1], color="black")
            if maze[r][c]["down"]:
                ax.plot([x, x+1], [y, y], color="black")
            if maze[r][c]["left"]:
                ax.plot([x, x], [y, y+1], color="black")
            if maze[r][c]["right"]:
                ax.plot([x+1, x+1], [y, y+1], color="black")

robot_dot, = ax.plot([], [], "ro", markersize=10)
path_line, = ax.plot([], [], "b-", linewidth=2)

line_x = []
line_y = []

def init():
    draw_maze()
    ax.set_xlim(0, COLS)
    ax.set_ylim(0, ROWS)
    ax.set_aspect("equal")
    ax.axis("off")
    return robot_dot, path_line

def update(frame):
    r, c = path[frame]
    x = c + 0.5
    y = ROWS - r - 1 + 0.5

    # Change robot color at dead ends
    if dead_end_flags[frame]:
        robot_dot.set_color("yellow")
    else:
        robot_dot.set_color("red")

    robot_dot.set_data(x, y)

    # Extend path line
    line_x.append(x)
    line_y.append(y)
    path_line.set_data(line_x, line_y)

    return robot_dot, path_line

# Slower animation: 900 ms per step
ani = animation.FuncAnimation(
    fig, update, frames=len(path),
    init_func=init, interval=700, blit=True
)

plt.show()
# add a stop condition when all cells have been visited
# add an end point for the drone to land once all cells have been visited
# store the cells coordinates in memory to use in the next iteration and go to the end goal
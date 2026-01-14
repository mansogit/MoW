"""
Challenge2.py - Complete Multi-Target Maze Challenge with GUI
Phase 1: Map the maze by visiting all targets
Phase 2: Race through all targets using optimal paths
         Drone rotates to face target direction at each target

GUI version with tkinter
"""

import pyhula
import time
import heapq
import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
from collections import deque

# Global variables
api = None
dstar = None
all_walls = set()
optimal_paths = {}
maze_data = {}
is_running = False
stop_requested = False

# Facing directions mapping
DIRECTIONS = {
    "North": 0,    # +Y
    "East": 90,    # +X
    "South": 180,  # -Y
    "West": 270    # -X
}

DIRECTION_ANGLES = {
    "North": 0,
    "East": 90,
    "South": 180,
    "West": 270
}


class Challenge2GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Challenge 2 - Multi-Target Maze Navigation")
        self.root.geometry("1400x900")

        # Variables
        self.rows = tk.IntVar(value=3)
        self.cols = tk.IntVar(value=3)
        self.start_x = tk.IntVar(value=0)
        self.start_y = tk.IntVar(value=0)
        self.race_start_x = tk.IntVar(value=0)
        self.race_start_y = tk.IntVar(value=0)
        self.aggressiveness = tk.IntVar(value=4)

        # Facing directions
        self.drone_facing = tk.StringVar(value="North")
        self.target_facing = tk.StringVar(value="North")

        # Targets list: [(x, y, facing), ...]
        self.targets = []
        self.target_x = tk.IntVar(value=0)
        self.target_y = tk.IntVar(value=0)

        # Current drone facing (updated during flight)
        self.current_facing = "North"

        # Connection status
        self.connected = False

        # Create main frames
        self.create_gui()

    def create_gui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Left panel - Controls
        left_panel = ttk.Frame(main_frame, padding="5")
        left_panel.grid(row=0, column=0, sticky="ns")

        # Right panel - Visualization and Log
        right_panel = ttk.Frame(main_frame, padding="5")
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=3)
        right_panel.rowconfigure(1, weight=1)

        # ============ LEFT PANEL ============

        # Connection Frame
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, sticky="ew", pady=5)

        self.conn_status = ttk.Label(conn_frame, text="* Disconnected", foreground="red")
        self.conn_status.grid(row=0, column=0, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_drone)
        self.connect_btn.grid(row=0, column=1, padx=5)

        # Maze Setup Frame
        maze_frame = ttk.LabelFrame(left_panel, text="Maze Setup", padding="5")
        maze_frame.grid(row=1, column=0, sticky="ew", pady=5)

        ttk.Label(maze_frame, text="Rows (Y):").grid(row=0, column=0, sticky="w")
        tk.Spinbox(maze_frame, from_=1, to=10, textvariable=self.rows, width=5).grid(row=0, column=1)

        ttk.Label(maze_frame, text="Cols (X):").grid(row=1, column=0, sticky="w")
        tk.Spinbox(maze_frame, from_=1, to=10, textvariable=self.cols, width=5).grid(row=1, column=1)

        ttk.Button(maze_frame, text="Update Grid", command=self.update_grid).grid(row=2, column=0, columnspan=2, pady=5)

        # Start Position Frame
        start_frame = ttk.LabelFrame(left_panel, text="Start Position", padding="5")
        start_frame.grid(row=2, column=0, sticky="ew", pady=5)

        ttk.Label(start_frame, text="Start X:").grid(row=0, column=0, sticky="w")
        tk.Spinbox(start_frame, from_=0, to=9, textvariable=self.start_x, width=5).grid(row=0, column=1)

        ttk.Label(start_frame, text="Start Y:").grid(row=1, column=0, sticky="w")
        tk.Spinbox(start_frame, from_=0, to=9, textvariable=self.start_y, width=5).grid(row=1, column=1)

        ttk.Label(start_frame, text="Drone Facing:").grid(row=2, column=0, sticky="w")
        drone_facing_combo = ttk.Combobox(start_frame, textvariable=self.drone_facing, width=8, state="readonly")
        drone_facing_combo['values'] = ('North', 'East', 'South', 'West')
        drone_facing_combo.current(0)
        drone_facing_combo.grid(row=2, column=1)

        # Targets Frame
        targets_frame = ttk.LabelFrame(left_panel, text="Targets", padding="5")
        targets_frame.grid(row=3, column=0, sticky="ew", pady=5)

        ttk.Label(targets_frame, text="Target X:").grid(row=0, column=0, sticky="w")
        tk.Spinbox(targets_frame, from_=0, to=9, textvariable=self.target_x, width=5).grid(row=0, column=1)

        ttk.Label(targets_frame, text="Target Y:").grid(row=1, column=0, sticky="w")
        tk.Spinbox(targets_frame, from_=0, to=9, textvariable=self.target_y, width=5).grid(row=1, column=1)

        ttk.Label(targets_frame, text="Face Direction:").grid(row=2, column=0, sticky="w")
        target_facing_combo = ttk.Combobox(targets_frame, textvariable=self.target_facing, width=8, state="readonly")
        target_facing_combo['values'] = ('North', 'East', 'South', 'West')
        target_facing_combo.current(0)
        target_facing_combo.grid(row=2, column=1)

        ttk.Button(targets_frame, text="Add Target", command=self.add_target).grid(row=3, column=0, columnspan=2, pady=2)
        ttk.Button(targets_frame, text="Clear Targets", command=self.clear_targets).grid(row=4, column=0, columnspan=2, pady=2)

        # Targets listbox
        self.targets_listbox = tk.Listbox(targets_frame, height=5, width=25)
        self.targets_listbox.grid(row=5, column=0, columnspan=2, pady=5)

        # Mapping Frame
        mapping_frame = ttk.LabelFrame(left_panel, text="Phase 1: Mapping", padding="5")
        mapping_frame.grid(row=4, column=0, sticky="ew", pady=5)

        self.map_btn = ttk.Button(mapping_frame, text="START MAPPING", command=self.start_mapping)
        self.map_btn.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")

        # Racing Frame
        racing_frame = ttk.LabelFrame(left_panel, text="Phase 2: Racing", padding="5")
        racing_frame.grid(row=5, column=0, sticky="ew", pady=5)

        ttk.Label(racing_frame, text="Race Start X:").grid(row=0, column=0, sticky="w")
        tk.Spinbox(racing_frame, from_=0, to=9, textvariable=self.race_start_x, width=5).grid(row=0, column=1)

        ttk.Label(racing_frame, text="Race Start Y:").grid(row=1, column=0, sticky="w")
        tk.Spinbox(racing_frame, from_=0, to=9, textvariable=self.race_start_y, width=5).grid(row=1, column=1)

        ttk.Label(racing_frame, text="Aggressiveness:").grid(row=2, column=0, sticky="w")
        agg_combo = ttk.Combobox(racing_frame, textvariable=self.aggressiveness, width=15, state="readonly")
        agg_combo['values'] = ('1 - Normal', '2 - Fast', '3 - Aggressive', '4 - TURBO', '5 - INSANE')
        agg_combo.current(3)
        agg_combo.grid(row=2, column=1)
        agg_combo.bind('<<ComboboxSelected>>', self.on_agg_select)

        self.race_btn = ttk.Button(racing_frame, text="START RACE", command=self.start_race)
        self.race_btn.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")
        self.race_btn.state(['disabled'])

        # Control Frame
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding="5")
        control_frame.grid(row=6, column=0, sticky="ew", pady=5)

        self.stop_btn = ttk.Button(control_frame, text="STOP", command=self.stop_flight)
        self.stop_btn.grid(row=0, column=0, padx=2, sticky="ew")
        self.stop_btn.state(['disabled'])

        self.land_btn = ttk.Button(control_frame, text="LAND", command=self.land_drone)
        self.land_btn.grid(row=0, column=1, padx=2, sticky="ew")

        # Status
        self.status_label = ttk.Label(left_panel, text="Status: Ready", font=('Arial', 10, 'bold'))
        self.status_label.grid(row=7, column=0, pady=10)

        # ============ RIGHT PANEL ============

        # Matplotlib Figure
        viz_frame = ttk.LabelFrame(right_panel, text="Maze Visualization", padding="5")
        viz_frame.grid(row=0, column=0, sticky="nsew")
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)

        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Log Frame
        log_frame = ttk.LabelFrame(right_panel, text="Log", padding="5")
        log_frame.grid(row=1, column=0, sticky="nsew", pady=5)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=80)
        self.log_text.grid(row=0, column=0, sticky="nsew")

        # Initialize grid
        self.update_grid()

    def on_agg_select(self, event):
        selection = event.widget.get()
        self.aggressiveness.set(int(selection[0]))

    def log(self, message):
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see(tk.END)
        self.root.update()

    def set_status(self, status):
        self.status_label.config(text=f"Status: {status}")
        self.root.update()

    def connect_drone(self):
        global api
        self.log("Connecting to drone...")
        try:
            api = pyhula.UserApi()
            if api.connect():
                self.connected = True
                self.conn_status.config(text="* Connected", foreground="green")
                self.log("[OK] Connected successfully!")
            else:
                self.log("[X] Connection failed!")
                messagebox.showerror("Error", "Failed to connect to drone")
        except Exception as e:
            self.log(f"[X] Error: {e}")
            messagebox.showerror("Error", f"Connection error: {e}")

    def update_grid(self):
        self.ax.clear()
        rows = self.rows.get()
        cols = self.cols.get()
        BLOCK = 0.6

        self.ax.set_xlim(-0.1, cols * BLOCK + 0.1)
        self.ax.set_ylim(-0.1, rows * BLOCK + 0.1)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (columns)')
        self.ax.set_ylabel('Y (rows)')
        self.ax.set_title('Maze Grid')

        # Draw grid
        for i in range(cols + 1):
            self.ax.axvline(i * BLOCK, color='black', linewidth=2)
        for i in range(rows + 1):
            self.ax.axhline(i * BLOCK, color='black', linewidth=2)

        # Draw coordinates
        for x in range(cols):
            for y in range(rows):
                self.ax.text(x * BLOCK + BLOCK/2, y * BLOCK + BLOCK/2, f"({x},{y})",
                            ha='center', va='center', fontsize=8, color='gray')

        # Draw start
        sx, sy = self.start_x.get(), self.start_y.get()
        if 0 <= sx < cols and 0 <= sy < rows:
            self.ax.add_patch(Rectangle((sx * BLOCK + 0.02, sy * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4))
            self.ax.text(sx * BLOCK + BLOCK/2, sy * BLOCK + BLOCK - 0.1, "START",
                        ha='center', va='top', fontsize=8, fontweight='bold', color='darkgreen')
            # Draw facing arrow
            self.draw_facing_arrow(sx, sy, self.drone_facing.get(), 'darkgreen')

        # Draw targets
        target_colors = ['red', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
        for i, (tx, ty, facing) in enumerate(self.targets):
            if 0 <= tx < cols and 0 <= ty < rows:
                color = target_colors[i % len(target_colors)]
                self.ax.add_patch(Rectangle((tx * BLOCK + 0.02, ty * BLOCK + 0.02),
                                            BLOCK - 0.04, BLOCK - 0.04, facecolor=color, alpha=0.4))
                self.ax.text(tx * BLOCK + BLOCK/2, ty * BLOCK + BLOCK - 0.1, f"T{i+1}",
                            ha='center', va='top', fontsize=10, fontweight='bold', color='black')
                # Draw target facing arrow
                self.draw_facing_arrow(tx, ty, facing, 'black')

        self.canvas.draw()

    def draw_facing_arrow(self, x, y, facing, color):
        """Draw an arrow showing facing direction"""
        BLOCK = 0.6
        cx = x * BLOCK + BLOCK/2
        cy = y * BLOCK + BLOCK/2
        arrow_len = 0.15

        if facing == "North":
            dx, dy = 0, arrow_len
        elif facing == "East":
            dx, dy = arrow_len, 0
        elif facing == "South":
            dx, dy = 0, -arrow_len
        else:  # West
            dx, dy = -arrow_len, 0

        self.ax.arrow(cx, cy, dx, dy, head_width=0.08, head_length=0.05,
                     fc=color, ec=color, linewidth=2)

    def add_target(self):
        tx, ty = self.target_x.get(), self.target_y.get()
        facing = self.target_facing.get()
        if tx < self.cols.get() and ty < self.rows.get():
            self.targets.append((tx, ty, facing))
            self.targets_listbox.insert(tk.END, f"T{len(self.targets)}: ({tx},{ty}) -> {facing}")
            self.update_grid()
            self.log(f"Added Target {len(self.targets)}: ({tx},{ty}) facing {facing}")
        else:
            messagebox.showwarning("Warning", "Target position out of bounds!")

    def clear_targets(self):
        self.targets = []
        self.targets_listbox.delete(0, tk.END)
        self.update_grid()
        self.log("Cleared all targets")

    def rotate_to_facing(self, target_facing):
        """Rotate drone to face the target direction using turnleft/turnright API"""
        global api

        current_angle = DIRECTION_ANGLES.get(self.current_facing, 0)
        target_angle = DIRECTION_ANGLES.get(target_facing, 0)

        # Calculate turn needed
        turn = target_angle - current_angle

        # Normalize to -180 to 180
        while turn > 180:
            turn -= 360
        while turn < -180:
            turn += 360

        if turn != 0:
            self.log(f"Rotating from {self.current_facing} to {target_facing} (turn: {turn} degrees)")

            if turn > 0:
                # Positive turn = turn right
                self.log(f"Turning RIGHT {turn} degrees")
                api.single_fly_turnright(abs(turn))
            else:
                # Negative turn = turn left
                self.log(f"Turning LEFT {abs(turn)} degrees")
                api.single_fly_turnleft(abs(turn))

            time.sleep(1.5)  # Wait for rotation to complete
            self.current_facing = target_facing

    def start_mapping(self):
        if not self.connected:
            messagebox.showwarning("Warning", "Please connect to drone first!")
            return
        if not self.targets:
            messagebox.showwarning("Warning", "Please add at least one target!")
            return

        global is_running, stop_requested
        is_running = True
        stop_requested = False

        self.map_btn.state(['disabled'])
        self.stop_btn.state(['!disabled'])
        self.set_status("MAPPING...")

        # Set initial facing
        self.current_facing = self.drone_facing.get()

        # Run mapping in separate thread
        thread = threading.Thread(target=self.run_mapping)
        thread.daemon = True
        thread.start()

    def run_mapping(self):
        global api, dstar, all_walls, optimal_paths, is_running, stop_requested

        try:
            rows = self.rows.get()
            cols = self.cols.get()
            start = (self.start_x.get(), self.start_y.get())
            targets = [(t[0], t[1]) for t in self.targets]  # Just x,y for mapping
            target_facings = [t[2] for t in self.targets]  # Store facings separately

            BLOCK = 0.6
            BLOCK_SIZE = 60
            FLIGHT_HEIGHT = 90
            OFFSET_X = 0
            OFFSET_Y = 0
            FIRST_BLOCK_CENTER_X = 15 + OFFSET_X
            FIRST_BLOCK_CENTER_Y = 15 + OFFSET_Y
            OBSTACLE_CHECK_COUNT = 3

            def block_to_cm(bx, by):
                return FIRST_BLOCK_CENTER_X + (bx * BLOCK_SIZE), FIRST_BLOCK_CENTER_Y + (by * BLOCK_SIZE)

            def move_to_block(bx, by):
                ax, ay = block_to_cm(bx, by)
                api.single_fly_straight_flight(ax, ay, FLIGHT_HEIGHT)

            def get_verified_obstacles():
                readings = {'forward': 0, 'back': 0, 'left': 0, 'right': 0}
                for _ in range(OBSTACLE_CHECK_COUNT):
                    obs = api.Plane_getBarrier()
                    for d in readings:
                        if obs.get(d, False):
                            readings[d] += 1
                    time.sleep(0.3)
                threshold = OBSTACLE_CHECK_COUNT // 2 + 1
                return {d: c >= threshold for d, c in readings.items()}

            def get_walls_from_obstacles(obs, cx, cy):
                walls = []
                if obs.get('forward', False) and cy + 1 < rows:
                    walls.append(tuple(sorted([(cx, cy), (cx, cy + 1)])))
                if obs.get('back', False) and cy - 1 >= 0:
                    walls.append(tuple(sorted([(cx, cy), (cx, cy - 1)])))
                if obs.get('right', False) and cx + 1 < cols:
                    walls.append(tuple(sorted([(cx, cy), (cx + 1, cy)])))
                if obs.get('left', False) and cx - 1 >= 0:
                    walls.append(tuple(sorted([(cx, cy), (cx - 1, cy)])))
                return walls

            # Initialize D* Lite
            dstar = DStarLite(start, targets[0], rows, cols)
            dstar.compute_shortest_path()

            self.log("Enabling obstacle detection...")
            api.single_fly_barrier_aircraft(True)

            self.log("Turning on QR positioning...")
            api.Plane_cmd_switch_QR(0)
            time.sleep(2)

            self.log("Taking off...")
            api.single_fly_takeoff()
            time.sleep(3)

            self.log(f"Moving to start: {start}")
            move_to_block(start[0], start[1])
            time.sleep(2)

            current_x, current_y = start
            path_history = [(current_x, current_y)]
            detected_walls = set()

            # Navigate to all targets
            for target_idx, target in enumerate(targets):
                if stop_requested:
                    break

                target_num = target_idx + 1
                self.log(f"=== Navigating to Target {target_num}: {target} ===")
                self.set_status(f"MAPPING - Target {target_num}/{len(targets)}")

                dstar.set_new_goal(target)
                dstar.move_to((current_x, current_y))

                max_steps = rows * cols * 4
                steps = 0

                while (current_x, current_y) != target and steps < max_steps:
                    if stop_requested:
                        break

                    obstacles = get_verified_obstacles()
                    self.log(f"Pos: ({current_x},{current_y}) | Obstacles: {obstacles}")

                    # Update plot
                    self.update_mapping_plot(current_x, current_y, obstacles, path_history,
                                           detected_walls, rows, cols, targets, start)

                    new_walls = get_walls_from_obstacles(obstacles, current_x, current_y)
                    for w in new_walls:
                        detected_walls.add(w)
                    if new_walls:
                        dstar.update_walls(new_walls)

                    next_pos = dstar.get_next_move()
                    if next_pos is None:
                        self.log(f"No path to target {target_num}!")
                        break

                    next_x, next_y = next_pos
                    self.log(f"Moving to: ({next_x},{next_y})")
                    move_to_block(next_x, next_y)

                    current_x, current_y = next_x, next_y
                    dstar.move_to(next_pos)
                    path_history.append((current_x, current_y))
                    steps += 1
                    time.sleep(2)

                if (current_x, current_y) == target:
                    self.log(f"[OK] Target {target_num} REACHED!")
                time.sleep(1)

            # Landing
            self.log("Mapping complete - Landing...")
            api.single_fly_touchdown()
            self.log("[OK] Landed!")

            # Save walls and compute paths
            all_walls = dstar.walls
            optimal_paths.clear()

            for i, target in enumerate(targets):
                path = self.find_path_bfs(start, target, all_walls, rows, cols)
                if path:
                    waypoints = self.simplify_path(path)
                    optimal_paths[f"start_to_target_{i+1}"] = {
                        "target_num": i + 1,
                        "full_path": [[p[0], p[1]] for p in path],
                        "waypoints": [[p[0], p[1]] for p in waypoints],
                        "length": len(path) - 1
                    }

            for i in range(len(targets) - 1):
                path = self.find_path_bfs(targets[i], targets[i + 1], all_walls, rows, cols)
                if path:
                    waypoints = self.simplify_path(path)
                    optimal_paths[f"target_{i+1}_to_target_{i+2}"] = {
                        "full_path": [[p[0], p[1]] for p in path],
                        "waypoints": [[p[0], p[1]] for p in waypoints],
                        "length": len(path) - 1
                    }

            # Save to JSON (include target facings)
            maze_data = {
                "maze_info": {"rows": rows, "cols": cols},
                "start": list(start),
                "drone_initial_facing": self.drone_facing.get(),
                "targets": [[t[0], t[1], t[2]] for t in self.targets],  # Include facing
                "walls": [[[w[0][0], w[0][1]], [w[1][0], w[1][1]]] for w in all_walls],
                "optimal_paths": optimal_paths
            }

            with open("challenge2_map.json", 'w') as f:
                json.dump(maze_data, f, indent=2)

            self.log(f"[OK] Map saved! Walls: {len(all_walls)}, Paths: {len(optimal_paths)}")
            self.set_status("MAPPING COMPLETE")

            # Enable race button
            self.root.after(0, lambda: self.race_btn.state(['!disabled']))

        except Exception as e:
            self.log(f"Error: {e}")
            self.set_status("ERROR")
        finally:
            is_running = False
            self.root.after(0, lambda: self.map_btn.state(['!disabled']))
            self.root.after(0, lambda: self.stop_btn.state(['disabled']))

    def update_mapping_plot(self, cx, cy, obstacles, path_history, detected_walls, rows, cols, targets, start):
        BLOCK = 0.6

        def update():
            self.ax.clear()

            self.ax.set_xlim(-0.1, cols * BLOCK + 0.1)
            self.ax.set_ylim(-0.1, rows * BLOCK + 0.1)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title(f'MAPPING | Pos: ({cx},{cy})')

            # Draw grid
            for i in range(cols + 1):
                self.ax.axvline(i * BLOCK, color='black', linewidth=2)
            for i in range(rows + 1):
                self.ax.axhline(i * BLOCK, color='black', linewidth=2)

            # Draw start
            self.ax.add_patch(Rectangle((start[0] * BLOCK + 0.02, start[1] * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4))

            # Draw targets
            target_colors = ['red', 'orange', 'purple', 'cyan', 'magenta']
            for i, target in enumerate(targets):
                tx, ty = target[0], target[1]
                color = target_colors[i % len(target_colors)]
                self.ax.add_patch(Rectangle((tx * BLOCK + 0.02, ty * BLOCK + 0.02),
                                            BLOCK - 0.04, BLOCK - 0.04, facecolor=color, alpha=0.4))
                self.ax.text(tx * BLOCK + BLOCK/2, ty * BLOCK + BLOCK - 0.1, f"T{i+1}",
                            ha='center', va='top', fontsize=10, fontweight='bold')

            # Draw detected walls
            for wall in detected_walls:
                p1, p2 = wall
                if p1[0] == p2[0]:  # Horizontal wall
                    x1, x2 = p1[0] * BLOCK, (p1[0] + 1) * BLOCK
                    y1 = y2 = max(p1[1], p2[1]) * BLOCK
                else:  # Vertical wall
                    x1 = x2 = max(p1[0], p2[0]) * BLOCK
                    y1, y2 = p1[1] * BLOCK, (p1[1] + 1) * BLOCK
                self.ax.plot([x1, x2], [y1, y2], 'r-', linewidth=6)

            # Draw path
            if path_history:
                px = [p[0] * BLOCK + BLOCK/2 for p in path_history]
                py = [p[1] * BLOCK + BLOCK/2 for p in path_history]
                self.ax.plot(px, py, 'b-', linewidth=2, alpha=0.6)

            # Draw drone
            self.ax.plot(cx * BLOCK + BLOCK/2, cy * BLOCK + BLOCK/2, 'bo', markersize=20)

            self.canvas.draw()

        self.root.after(0, update)

    def start_race(self):
        if not self.connected:
            messagebox.showwarning("Warning", "Please connect to drone first!")
            return
        if not optimal_paths:
            messagebox.showwarning("Warning", "Please complete mapping first!")
            return

        global is_running, stop_requested
        is_running = True
        stop_requested = False

        self.race_btn.state(['disabled'])
        self.stop_btn.state(['!disabled'])
        self.set_status("RACING...")

        # Reset facing to initial drone facing for race
        self.current_facing = self.drone_facing.get()

        thread = threading.Thread(target=self.run_race)
        thread.daemon = True
        thread.start()

    def run_race(self):
        global api, all_walls, optimal_paths, is_running, stop_requested

        try:
            rows = self.rows.get()
            cols = self.cols.get()
            race_start = (self.race_start_x.get(), self.race_start_y.get())
            targets = self.targets.copy()  # Include facing info
            level = self.aggressiveness.get()

            TIMING = {
                1: {"name": "Normal", "takeoff": 2.5, "start": 1.5, "wp": 1.0, "target": 2.0},
                2: {"name": "Fast", "takeoff": 2.0, "start": 1.0, "wp": 0.5, "target": 1.0},
                3: {"name": "Aggressive", "takeoff": 1.5, "start": 0.5, "wp": 0.2, "target": 0.5},
                4: {"name": "TURBO", "takeoff": 1.0, "start": 0.3, "wp": 0.0, "target": 0.2},
                5: {"name": "INSANE", "takeoff": 0.5, "start": 0.1, "wp": 0.0, "target": 0.1},
            }
            timing = TIMING[level]

            BLOCK_SIZE = 60
            FLIGHT_HEIGHT = 80
            FIRST_BLOCK_CENTER_X = 15
            FIRST_BLOCK_CENTER_Y = 15

            def block_to_cm(bx, by):
                return FIRST_BLOCK_CENTER_X + (bx * BLOCK_SIZE), FIRST_BLOCK_CENTER_Y + (by * BLOCK_SIZE)

            def move_to_block(bx, by):
                ax, ay = block_to_cm(bx, by)
                api.single_fly_straight_flight(ax, ay, FLIGHT_HEIGHT)

            self.log(f"=== RACE START - Level {level} ({timing['name']}) ===")
            self.log(f"Initial facing: {self.current_facing}")

            self.log("Taking off...")
            api.single_fly_takeoff()
            time.sleep(timing["takeoff"])

            self.log(f"Moving to race start: {race_start}")
            move_to_block(race_start[0], race_start[1])
            time.sleep(timing["start"])

            current_x, current_y = race_start
            flight_history = [(current_x, current_y)]
            race_start_time = time.time()

            for target_idx, target_info in enumerate(targets):
                if stop_requested:
                    break

                target_num = target_idx + 1
                tx, ty, target_facing = target_info
                target = (tx, ty)

                self.log(f"Racing to Target {target_num}: {target}, Face: {target_facing}")
                self.set_status(f"RACING - Target {target_num}/{len(targets)}")

                # Get path
                if target_idx == 0:
                    path = self.find_path_bfs(race_start, target, all_walls, rows, cols)
                    waypoints = self.simplify_path(path) if path else []
                else:
                    prev_target = (targets[target_idx - 1][0], targets[target_idx - 1][1])
                    key = f"target_{target_idx}_to_target_{target_num}"
                    if key in optimal_paths:
                        waypoints = [tuple(p) for p in optimal_paths[key]["waypoints"]]
                    else:
                        path = self.find_path_bfs(prev_target, target, all_walls, rows, cols)
                        waypoints = self.simplify_path(path) if path else []

                if not waypoints:
                    self.log(f"No path to Target {target_num}!")
                    continue

                # Skip first if already there
                start_idx = 1 if waypoints and tuple(waypoints[0]) == (current_x, current_y) else 0

                for wp in waypoints[start_idx:]:
                    if stop_requested:
                        break

                    wp_x, wp_y = wp if isinstance(wp, tuple) else tuple(wp)
                    move_to_block(wp_x, wp_y)

                    current_x, current_y = wp_x, wp_y
                    flight_history.append((current_x, current_y))

                    self.update_race_plot(current_x, current_y, flight_history, rows, cols,
                                         targets, race_start, self.current_facing)

                    if timing["wp"] > 0:
                        time.sleep(timing["wp"])

                # Target reached - now rotate to face target direction
                self.log(f"[OK] Target {target_num} REACHED!")

                # Rotate to target facing direction
                if target_facing != self.current_facing:
                    self.log(f"Rotating to face {target_facing}...")
                    self.rotate_to_facing(target_facing)
                    self.update_race_plot(current_x, current_y, flight_history, rows, cols,
                                         targets, race_start, self.current_facing)

                if timing["target"] > 0:
                    time.sleep(timing["target"])

            race_time = time.time() - race_start_time

            self.log("Landing...")
            api.single_fly_touchdown()
            self.log("[OK] Landed!")

            self.log(f"=== RACE COMPLETE: {race_time:.1f} seconds ===")
            self.set_status(f"FINISHED - {race_time:.1f}s")

        except Exception as e:
            self.log(f"Error: {e}")
            self.set_status("ERROR")
        finally:
            is_running = False
            self.root.after(0, lambda: self.race_btn.state(['!disabled']))
            self.root.after(0, lambda: self.stop_btn.state(['disabled']))

    def update_race_plot(self, cx, cy, flight_history, rows, cols, targets, race_start, current_facing):
        BLOCK = 0.6

        def update():
            self.ax.clear()

            self.ax.set_xlim(-0.1, cols * BLOCK + 0.1)
            self.ax.set_ylim(-0.1, rows * BLOCK + 0.1)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title(f'RACING | Pos: ({cx},{cy}) | Facing: {current_facing}')

            # Draw grid
            for i in range(cols + 1):
                self.ax.axvline(i * BLOCK, color='black', linewidth=2)
            for i in range(rows + 1):
                self.ax.axhline(i * BLOCK, color='black', linewidth=2)

            # Draw walls
            for wall in all_walls:
                p1, p2 = wall
                if p1[0] == p2[0]:
                    x1, x2 = p1[0] * BLOCK, (p1[0] + 1) * BLOCK
                    y1 = y2 = max(p1[1], p2[1]) * BLOCK
                else:
                    x1 = x2 = max(p1[0], p2[0]) * BLOCK
                    y1, y2 = p1[1] * BLOCK, (p1[1] + 1) * BLOCK
                self.ax.plot([x1, x2], [y1, y2], 'r-', linewidth=6)

            # Draw race start
            self.ax.add_patch(Rectangle((race_start[0] * BLOCK + 0.02, race_start[1] * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='lime', alpha=0.6))

            # Draw targets with facing arrows
            target_colors = ['red', 'orange', 'purple', 'cyan', 'magenta']
            for i, (tx, ty, facing) in enumerate(targets):
                color = target_colors[i % len(target_colors)]
                self.ax.add_patch(Rectangle((tx * BLOCK + 0.02, ty * BLOCK + 0.02),
                                            BLOCK - 0.04, BLOCK - 0.04, facecolor=color, alpha=0.4))
                self.ax.text(tx * BLOCK + BLOCK/2, ty * BLOCK + BLOCK - 0.1, f"T{i+1}",
                            ha='center', va='top', fontsize=10, fontweight='bold')
                # Draw target facing arrow
                self.draw_facing_arrow_on_ax(self.ax, tx, ty, facing, 'black')

            # Draw flight path
            if flight_history:
                px = [p[0] * BLOCK + BLOCK/2 for p in flight_history]
                py = [p[1] * BLOCK + BLOCK/2 for p in flight_history]
                self.ax.plot(px, py, 'b-', linewidth=3, alpha=0.7)

            # Draw drone with facing arrow
            self.ax.plot(cx * BLOCK + BLOCK/2, cy * BLOCK + BLOCK/2, 'bo', markersize=20)
            self.draw_facing_arrow_on_ax(self.ax, cx, cy, current_facing, 'blue')

            self.canvas.draw()

        self.root.after(0, update)

    def draw_facing_arrow_on_ax(self, ax, x, y, facing, color):
        """Draw an arrow showing facing direction on given axes"""
        BLOCK = 0.6
        cx = x * BLOCK + BLOCK/2
        cy = y * BLOCK + BLOCK/2
        arrow_len = 0.15

        if facing == "North":
            dx, dy = 0, arrow_len
        elif facing == "East":
            dx, dy = arrow_len, 0
        elif facing == "South":
            dx, dy = 0, -arrow_len
        else:  # West
            dx, dy = -arrow_len, 0

        ax.arrow(cx, cy, dx, dy, head_width=0.08, head_length=0.05,
                fc=color, ec=color, linewidth=2)

    def stop_flight(self):
        global stop_requested
        stop_requested = True
        self.log("Stop requested...")
        self.set_status("STOPPING...")

    def land_drone(self):
        global api
        if api and self.connected:
            self.log("Emergency landing...")
            api.single_fly_touchdown()
            self.log("[OK] Landed!")
            self.set_status("Landed")

    def find_path_bfs(self, start, goal, walls, rows, cols):
        if start == goal:
            return [start]
        queue = deque([(start, [start])])
        visited = {start}
        while queue:
            current, path = queue.popleft()
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current[0] + dx, current[1] + dy
                if not (0 <= nx < cols and 0 <= ny < rows):
                    continue
                if (nx, ny) in visited:
                    continue
                edge = tuple(sorted([current, (nx, ny)]))
                if edge in walls:
                    continue
                new_path = path + [(nx, ny)]
                if (nx, ny) == goal:
                    return new_path
                visited.add((nx, ny))
                queue.append(((nx, ny), new_path))
        return None

    def simplify_path(self, path):
        if not path or len(path) <= 2:
            return path
        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            prev, curr, next_pt = path[i-1], path[i], path[i+1]
            dir1 = (curr[0] - prev[0], curr[1] - prev[1])
            dir2 = (next_pt[0] - curr[0], next_pt[1] - curr[1])
            if dir1 != dir2:
                simplified.append(curr)
        simplified.append(path[-1])
        return simplified


# D* Lite class
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
        return float('inf') if edge in self.walls else 1

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


# Main
if __name__ == "__main__":
    root = tk.Tk()
    app = Challenge2GUI(root)
    root.mainloop()
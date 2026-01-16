"""
Challenge2.py - Complete Multi-Target Maze Challenge with GUI
Phase 1: Map the ENTIRE maze by exploring all cells
Phase 2: Input targets during race and navigate through them
         Drone rotates to face target direction at each target
         Hover at each target for 10 seconds to detect objects

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
import cv2
import numpy as np
from PIL import Image, ImageTk

# Import video and detection modules
try:
    from hula_video import hula_video
    from onnxdetector import onnxdetector

    VIDEO_AVAILABLE = True
except ImportError:
    VIDEO_AVAILABLE = False
    print("Warning: hula_video or onnxdetector not available. Object detection disabled.")

# Global variables
api = None
all_walls = set()
maze_data = {}
is_running = False
stop_requested = False

# Video and detection
vid = None
huladetector = None
detected_objects = []  # List of detected objects at each target: [(target_num, object_name), ...]

# Facing directions mapping
DIRECTIONS = {
    "North": 0,  # +Y
    "East": 90,  # +X
    "South": 180,  # -Y
    "West": 270  # -X
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
        self.root.title("Challenge 2 - Complete Maze Mapping & Multi-Target Navigation")
        self.root.geometry("1200x800")  # Reduced from 1400x1100

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

        # Object detection option
        self.enable_detection = tk.BooleanVar(value=True)
        self.detection_hover_time = tk.IntVar(value=10)  # Hover time in seconds

        # Targets list for race: [(x, y, facing), ...]
        self.race_targets = []
        self.target_x = tk.IntVar(value=0)
        self.target_y = tk.IntVar(value=0)

        # Current drone facing (updated during flight)
        self.current_facing = "North"

        # Connection status
        self.connected = False

        # Mapped cells
        self.mapped_cells = set()

        # Detection image storage
        self.current_detection_image = None
        self.detection_photo = None

        # Create main frames
        self.create_gui()

    def create_gui(self):
        # Try to load saved maze data
        self.load_saved_map()

        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Left panel - Controls (Scrollable)
        left_container = ttk.Frame(main_frame, padding="5")
        left_container.grid(row=0, column=0, sticky="ns")

        # Create canvas and scrollbar for left panel
        left_canvas = tk.Canvas(left_container, width=280, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        left_panel = ttk.Frame(left_canvas)

        left_panel.bind(
            "<Configure>",
            lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        )

        left_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_canvas.pack(side="left", fill="both", expand=True)
        left_scrollbar.pack(side="right", fill="y")

        # Enable mouse wheel scrolling
        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        left_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # Right panel - Visualization and Log
        right_panel = ttk.Frame(main_frame, padding="5")
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=3)  # Maze visualization (larger)
        right_panel.rowconfigure(1, weight=2)  # Detection image (medium)
        right_panel.rowconfigure(2, weight=1)  # Log (smaller)

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
        start_frame = ttk.LabelFrame(left_panel, text="Mapping Start Position", padding="5")
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

        # Mapping Frame
        mapping_frame = ttk.LabelFrame(left_panel, text="Phase 1: Complete Maze Mapping", padding="5")
        mapping_frame.grid(row=3, column=0, sticky="ew", pady=5)

        ttk.Label(mapping_frame, text="Maps entire maze", font=('Arial', 8, 'italic')).grid(row=0, column=0,
                                                                                            columnspan=2)

        self.map_btn = ttk.Button(mapping_frame, text="START MAPPING", command=self.start_mapping)
        self.map_btn.grid(row=1, column=0, columnspan=2, pady=5, sticky="ew")

        ttk.Button(mapping_frame, text="Load Saved Map", command=self.reload_map).grid(row=2, column=0, columnspan=2,
                                                                                       pady=2, sticky="ew")

        # Race Targets Frame
        race_targets_frame = ttk.LabelFrame(left_panel, text="Phase 2: Race Targets", padding="5")
        race_targets_frame.grid(row=4, column=0, sticky="ew", pady=5)

        ttk.Label(race_targets_frame, text="Add targets for race:", font=('Arial', 8, 'italic')).grid(row=0, column=0,
                                                                                                      columnspan=2)

        ttk.Label(race_targets_frame, text="Target X:").grid(row=1, column=0, sticky="w")
        tk.Spinbox(race_targets_frame, from_=0, to=9, textvariable=self.target_x, width=5).grid(row=1, column=1)

        ttk.Label(race_targets_frame, text="Target Y:").grid(row=2, column=0, sticky="w")
        tk.Spinbox(race_targets_frame, from_=0, to=9, textvariable=self.target_y, width=5).grid(row=2, column=1)

        ttk.Label(race_targets_frame, text="Face Direction:").grid(row=3, column=0, sticky="w")
        target_facing_combo = ttk.Combobox(race_targets_frame, textvariable=self.target_facing, width=8,
                                           state="readonly")
        target_facing_combo['values'] = ('North', 'East', 'South', 'West')
        target_facing_combo.current(0)
        target_facing_combo.grid(row=3, column=1)

        ttk.Button(race_targets_frame, text="Add Target", command=self.add_race_target).grid(row=4, column=0,
                                                                                             columnspan=2, pady=2)
        ttk.Button(race_targets_frame, text="Clear Targets", command=self.clear_race_targets).grid(row=5, column=0,
                                                                                                   columnspan=2, pady=2)

        # Targets listbox
        self.race_targets_listbox = tk.Listbox(race_targets_frame, height=4,
                                               width=23)  # Reduced from height=5, width=25
        self.race_targets_listbox.grid(row=6, column=0, columnspan=2, pady=5)

        # Racing Frame
        racing_frame = ttk.LabelFrame(left_panel, text="Race Configuration", padding="5")
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

        ttk.Checkbutton(racing_frame, text="Object Detection", variable=self.enable_detection).grid(row=3, column=0,
                                                                                                    columnspan=2,
                                                                                                    sticky="w")

        ttk.Label(racing_frame, text="Hover Time (sec):").grid(row=4, column=0, sticky="w")
        tk.Spinbox(racing_frame, from_=1, to=30, textvariable=self.detection_hover_time, width=5).grid(row=4, column=1)

        self.race_btn = ttk.Button(racing_frame, text="START RACE", command=self.start_race)
        self.race_btn.grid(row=5, column=0, columnspan=2, pady=5, sticky="ew")
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

        # Detected Objects Frame
        detected_frame = ttk.LabelFrame(left_panel, text="Detected Objects", padding="5")
        detected_frame.grid(row=8, column=0, sticky="ew", pady=5)

        self.detected_listbox = tk.Listbox(detected_frame, height=4, width=23)  # Reduced from height=6, width=25
        self.detected_listbox.grid(row=0, column=0, pady=5)

        ttk.Button(detected_frame, text="Clear Detections", command=self.clear_detections).grid(row=1, column=0, pady=2)

        # ============ RIGHT PANEL ============

        # Matplotlib Figure
        viz_frame = ttk.LabelFrame(right_panel, text="Maze Visualization", padding="5")
        viz_frame.grid(row=0, column=0, sticky="nsew")
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)

        self.fig, self.ax = plt.subplots(figsize=(7, 5))  # Reduced from (8, 6)
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Detection Image Display Frame
        detection_frame = ttk.LabelFrame(right_panel, text="Object Detection View", padding="5")
        detection_frame.grid(row=1, column=0, sticky="nsew", pady=5)
        detection_frame.columnconfigure(0, weight=1)
        detection_frame.rowconfigure(0, weight=1)

        # Create canvas for detection image
        self.detection_canvas = tk.Canvas(detection_frame, width=480, height=270, bg='black')  # Reduced from 640x360
        self.detection_canvas.grid(row=0, column=0, sticky="nsew")

        # Label for detection info
        self.detection_info_label = ttk.Label(detection_frame, text="No detection yet",
                                              font=('Arial', 10, 'bold'), foreground='blue')
        self.detection_info_label.grid(row=1, column=0, pady=5)

        # Log Frame
        log_frame = ttk.LabelFrame(right_panel, text="Log", padding="5")
        log_frame.grid(row=2, column=0, sticky="nsew", pady=5)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=6, width=70)  # Reduced from height=8, width=80
        self.log_text.grid(row=0, column=0, sticky="nsew")

        # Initialize grid
        self.update_grid()

        # Check if map was loaded and update UI accordingly
        self.check_loaded_map_status()

    def on_agg_select(self, event):
        selection = event.widget.get()
        self.aggressiveness.set(int(selection[0]))

    def log(self, message):
        """Thread-safe logging to GUI"""

        def _log():
            self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
            self.log_text.see(tk.END)

        # Use after() to safely update GUI from any thread
        self.root.after(0, _log)

    def clear_log(self):
        """Clear the log window"""
        self.log_text.delete(1.0, tk.END)

    def load_saved_map(self):
        """Load previously saved maze map from JSON file"""
        global all_walls, maze_data

        filename = "challenge2_complete_map.json"
        try:
            with open(filename, 'r') as f:
                maze_data = json.load(f)

            # Extract walls
            walls_list = maze_data.get("walls", [])
            all_walls = set()
            for wall in walls_list:
                p1 = tuple(wall[0])
                p2 = tuple(wall[1])
                all_walls.add(tuple(sorted([p1, p2])))

            # Extract maze info
            maze_info = maze_data.get("maze_info", {})
            rows = maze_info.get("rows", 3)
            cols = maze_info.get("cols", 3)

            # Update GUI with loaded dimensions
            self.rows.set(rows)
            self.cols.set(cols)

            # Extract mapped cells
            mapped_cells_list = maze_data.get("mapped_cells", [])
            self.mapped_cells = {tuple(cell) for cell in mapped_cells_list}

            # Extract mapping start
            mapping_start = maze_data.get("mapping_start", [0, 0])
            self.start_x.set(mapping_start[0])
            self.start_y.set(mapping_start[1])

            # Extract facing
            initial_facing = maze_data.get("drone_initial_facing", "North")
            self.drone_facing.set(initial_facing)

            return True, maze_info

        except FileNotFoundError:
            return False, None
        except Exception as e:
            print(f"Error loading map: {e}")
            return False, None

    def check_loaded_map_status(self):
        """Check if a saved map was loaded and update GUI accordingly"""
        if all_walls:
            # Map was loaded successfully
            maze_info = maze_data.get("maze_info", {})
            cells_mapped = maze_info.get("cells_mapped", 0)
            total_cells = maze_info.get("total_cells", 0)
            walls_count = maze_info.get("walls_detected", 0)
            mapping_date = maze_info.get("mapping_date", "Unknown")

            self.log("=" * 60)
            self.log("SAVED MAP LOADED SUCCESSFULLY!")
            self.log("=" * 60)
            self.log(f"Map Date: {mapping_date}")
            self.log(f"Maze Size: {self.rows.get()} x {self.cols.get()}")
            self.log(f"Cells Mapped: {cells_mapped} / {total_cells}")
            self.log(f"Walls Detected: {walls_count}")
            self.log("=" * 60)
            self.log("You can now:")
            self.log("1. Add race targets")
            self.log("2. Set race start position")
            self.log("3. Start racing immediately!")
            self.log("=" * 60)

            # Enable race button since map exists
            self.race_btn.state(['!disabled'])
            self.set_status("MAP LOADED - READY TO RACE")

            # Update visualization to show loaded map
            self.update_loaded_map_visualization()
        else:
            self.log("No saved map found. Please complete Phase 1: Mapping first.")
            self.set_status("Ready - No Map")

    def reload_map(self):
        """Manually reload the saved map file"""
        self.log("Reloading saved map...")
        success, maze_info = self.load_saved_map()
        if success:
            self.check_loaded_map_status()
            messagebox.showinfo("Success", "Map loaded successfully!")
        else:
            messagebox.showwarning("Warning", "No saved map file found. Please complete mapping first.")
            self.log("[X] Failed to load map - file not found")

    def update_loaded_map_visualization(self):
        """Update visualization to show the loaded map with walls"""
        rows = self.rows.get()
        cols = self.cols.get()
        BLOCK = 0.6

        self.ax.clear()
        self.ax.set_xlim(-0.1, cols * BLOCK + 0.1)
        self.ax.set_ylim(-0.1, rows * BLOCK + 0.1)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (columns)')
        self.ax.set_ylabel('Y (rows)')
        self.ax.set_title('Loaded Maze Map (Ready for Racing)')

        # Draw grid
        for i in range(cols + 1):
            self.ax.axvline(i * BLOCK, color='black', linewidth=2)
        for i in range(rows + 1):
            self.ax.axhline(i * BLOCK, color='black', linewidth=2)

        # Draw mapped cells
        for mx, my in self.mapped_cells:
            self.ax.add_patch(Rectangle((mx * BLOCK + 0.02, my * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='lightgreen', alpha=0.3))

        # Draw walls from loaded map
        for wall in all_walls:
            p1, p2 = wall
            if p1[0] == p2[0]:  # Horizontal wall
                x1, x2 = p1[0] * BLOCK, (p1[0] + 1) * BLOCK
                y1 = y2 = max(p1[1], p2[1]) * BLOCK
            else:  # Vertical wall
                x1 = x2 = max(p1[0], p2[0]) * BLOCK
                y1, y2 = p1[1] * BLOCK, (p1[1] + 1) * BLOCK
            self.ax.plot([x1, x2], [y1, y2], 'r-', linewidth=6)

        # Draw mapping start
        sx, sy = self.start_x.get(), self.start_y.get()
        if 0 <= sx < cols and 0 <= sy < rows:
            self.ax.add_patch(Rectangle((sx * BLOCK + 0.02, sy * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4))
            self.ax.text(sx * BLOCK + BLOCK / 2, sy * BLOCK + BLOCK - 0.1, "MAP START",
                         ha='center', va='top', fontsize=7, fontweight='bold', color='darkgreen')

        self.canvas.draw()

    def set_status(self, status):
        """Thread-safe status update"""

        def _update():
            self.status_label.config(text=f"Status: {status}")

        self.root.after(0, _update)

    def connect_drone(self):
        global api
        self.log("Connecting to drone...")
        try:
            api = pyhula.UserApi()
            if api.connect():
                self.connected = True
                self.conn_status.config(text="* Connected", foreground="green")
                self.log("[OK] Connected successfully!")

                # Initialize video and detection
                if VIDEO_AVAILABLE:
                    self.initialize_video_detection()
            else:
                self.log("[X] Connection failed!")
                messagebox.showerror("Error", "Failed to connect to drone")
        except Exception as e:
            self.log(f"[X] Error: {e}")
            messagebox.showerror("Error", f"Connection error: {e}")

    def initialize_video_detection(self):
        """Initialize video streaming and object detection"""
        global vid, huladetector

        try:
            self.log("Initializing video streaming...")
            vid = hula_video(hula_api=api, display=False)

            self.log("Loading object detection model...")
            huladetector = onnxdetector(
                model="detect_3_object_12_11.onnx",
                label="object.txt",
                confidence_thres=0.3
            )

            self.log("[OK] Video and detection initialized!")
        except Exception as e:
            self.log(f"[X] Failed to initialize video/detection: {e}")
            messagebox.showwarning("Warning", f"Video/detection initialization failed: {e}")

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
                self.ax.text(x * BLOCK + BLOCK / 2, y * BLOCK + BLOCK / 2, f"({x},{y})",
                             ha='center', va='center', fontsize=8, color='gray')

        # Draw mapping start
        sx, sy = self.start_x.get(), self.start_y.get()
        if 0 <= sx < cols and 0 <= sy < rows:
            self.ax.add_patch(Rectangle((sx * BLOCK + 0.02, sy * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4))
            self.ax.text(sx * BLOCK + BLOCK / 2, sy * BLOCK + BLOCK - 0.1, "MAP START",
                         ha='center', va='top', fontsize=7, fontweight='bold', color='darkgreen')

        # Draw race start
        rx, ry = self.race_start_x.get(), self.race_start_y.get()
        if 0 <= rx < cols and 0 <= ry < rows:
            self.ax.add_patch(Rectangle((rx * BLOCK + 0.02, ry * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='lime', alpha=0.4))
            self.ax.text(rx * BLOCK + BLOCK / 2, ry * BLOCK + 0.1, "RACE START",
                         ha='center', va='bottom', fontsize=7, fontweight='bold', color='darkgreen')

        # Draw race targets with facing arrows
        target_colors = ['red', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
        for i, (tx, ty, facing) in enumerate(self.race_targets):
            if 0 <= tx < cols and 0 <= ty < rows:
                color = target_colors[i % len(target_colors)]
                self.ax.add_patch(Rectangle((tx * BLOCK + 0.02, ty * BLOCK + 0.02),
                                            BLOCK - 0.04, BLOCK - 0.04, facecolor=color, alpha=0.4))
                self.ax.text(tx * BLOCK + BLOCK / 2, ty * BLOCK + BLOCK - 0.1, f"T{i + 1}",
                             ha='center', va='top', fontsize=10, fontweight='bold', color='black')
                # Draw target facing arrow
                self.draw_facing_arrow(tx, ty, facing, 'black')

        self.canvas.draw()

    def draw_facing_arrow(self, x, y, facing, color):
        """Draw an arrow showing facing direction"""
        BLOCK = 0.6
        cx = x * BLOCK + BLOCK / 2
        cy = y * BLOCK + BLOCK / 2
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

    def add_race_target(self):
        tx, ty = self.target_x.get(), self.target_y.get()
        facing = self.target_facing.get()
        if tx < self.cols.get() and ty < self.rows.get():
            self.race_targets.append((tx, ty, facing))
            self.race_targets_listbox.insert(tk.END, f"T{len(self.race_targets)}: ({tx},{ty}) -> {facing}")
            self.update_grid()
            self.log(f"Added Race Target {len(self.race_targets)}: ({tx},{ty}) facing {facing}")
        else:
            messagebox.showwarning("Warning", "Target position out of bounds!")

    def clear_race_targets(self):
        self.race_targets = []
        self.race_targets_listbox.delete(0, tk.END)
        self.update_grid()
        self.log("Cleared all race targets")

    def clear_detections(self):
        global detected_objects
        detected_objects = []
        self.detected_listbox.delete(0, tk.END)
        self.log("Cleared all detected objects")

        # Clear detection image
        self.detection_canvas.delete("all")
        self.detection_info_label.config(text="No detection yet")
        self.current_detection_image = None

    def display_detection_image(self, frame, obj_name, confidence, target_num):
        """Display detected object image with bounding box, label and score in GUI (thread-safe)"""
        try:
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize frame to fit canvas (480x270) - reduced size
            height, width = frame_rgb.shape[:2]
            target_width = 480
            target_height = 270

            # Calculate scaling to fit
            scale = min(target_width / width, target_height / height)
            new_width = int(width * scale)
            new_height = int(height * scale)

            resized_frame = cv2.resize(frame_rgb, (new_width, new_height))

            # Convert to PIL Image
            img = Image.fromarray(resized_frame)
            self.current_detection_image = img

            # Thread-safe GUI update
            def _update_display():
                # Convert to PhotoImage for tkinter
                self.detection_photo = ImageTk.PhotoImage(image=img)

                # Display on canvas
                self.detection_canvas.delete("all")

                # Center the image on canvas
                x_offset = (target_width - new_width) // 2
                y_offset = (target_height - new_height) // 2

                self.detection_canvas.create_image(x_offset, y_offset,
                                                   anchor=tk.NW,
                                                   image=self.detection_photo)

                # Update info label with detection details
                info_text = f"Target {target_num}: {obj_name} (Confidence: {confidence:.2%})"
                self.detection_info_label.config(text=info_text, foreground='green')

            self.root.after(0, _update_display)
            self.log(f"[GUI] Displayed detection image for Target {target_num}")

        except Exception as e:
            self.log(f"[X] Error displaying detection image: {e}")

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

    def detect_object_at_target(self, target_num):
        """Hover at target and detect object using configurable hover time"""
        global vid, huladetector, detected_objects, api

        if not VIDEO_AVAILABLE:
            self.log(f"[!] VIDEO_AVAILABLE is False - detection disabled")
            return None

        if vid is None:
            self.log(f"[!] vid is None - trying to reinitialize...")
            try:
                vid = hula_video(hula_api=api, display=False)
                self.log("[OK] Video reinitialized")
            except Exception as e:
                self.log(f"[X] Could not reinitialize video: {e}")
                return None

        if huladetector is None:
            self.log(f"[!] huladetector is None - trying to reinitialize...")
            try:
                huladetector = onnxdetector(
                    model="detect_3_object_12_11.onnx",
                    label="object.txt",
                    confidence_thres=0.3
                )
                self.log("[OK] Detector reinitialized")
            except Exception as e:
                self.log(f"[X] Could not reinitialize detector: {e}")
                return None

        hover_time = self.detection_hover_time.get()
        self.log(f"Starting object detection at Target {target_num}...")
        self.log(f"Hovering for {hover_time} seconds...")

        try:
            # Start video recording
            self.log("Starting video stream...")
            vid.video_mode_on()
            time.sleep(0.5)  # Give video mode time to initialize

            self.log("Video mode on - starting recording...")
            vid.startrecording()
            time.sleep(0.5)  # Give recording time to start

            self.log("Recording started - beginning detection loop...")

            detected_obj = None
            detected_confidence = 0.0
            detected_frame = None
            detection_count = 0
            detection_threshold = 10  # Need 10 consecutive detections

            start_time = time.time()
            last_detected = None
            frame_count = 0

            while time.time() - start_time < hover_time:  # Configurable detection window
                try:
                    frame = vid.get_video()

                    if frame is None:
                        if frame_count % 10 == 0:
                            self.log(f"[DEBUG] Frame {frame_count} is None - waiting for video...")
                        time.sleep(0.1)
                        frame_count += 1
                        continue

                    # Log first frame received
                    if frame_count == 0:
                        self.log(f"[DEBUG] Got first frame, shape: {frame.shape}")

                    frame_count += 1

                    # CRITICAL FIX: Check if frame is valid before detection
                    if frame.size == 0 or frame.shape[0] == 0 or frame.shape[1] == 0:
                        self.log(f"[DEBUG] Invalid frame dimensions: {frame.shape}")
                        time.sleep(0.1)
                        continue

                    # Get detection result - IMPORTANT: detector returns (obj_name, annotated_frame)
                    obj_found, annotated_frame = huladetector.detect(frame)

                    # Only process if we got a valid annotated frame back
                    if annotated_frame is None:
                        time.sleep(0.1)
                        continue

                    # Try to extract confidence score from detector
                    confidence = 0.0
                    if hasattr(huladetector, 'last_confidence'):
                        confidence = huladetector.last_confidence
                    else:
                        # Default confidence if not available
                        confidence = 0.8

                    if obj_found is not None:
                        self.log(f"Detected: {obj_found} (conf: {confidence:.2f})")

                        # Count consecutive detections of same object
                        if obj_found == last_detected:
                            detection_count += 1
                        else:
                            detection_count = 1
                            last_detected = obj_found

                        # Confirm detection after threshold
                        if detection_count >= detection_threshold and detected_obj is None:
                            detected_obj = obj_found
                            detected_confidence = confidence
                            # Make a copy of the frame to avoid memory issues
                            detected_frame = annotated_frame.copy()
                            self.log(f"[OK] Confirmed detection: {detected_obj}")
                            break
                    else:
                        # Only log occasionally to reduce log spam
                        if frame_count % 10 == 0:
                            self.log("Looking for object...")
                        detection_count = 0
                        last_detected = None

                    # Display current frame (like in working code)
                    cv2.imshow("Detection", annotated_frame)
                    cv2.waitKey(1)

                    time.sleep(0.1)

                except Exception as e:
                    self.log(f"Frame processing error: {e}")
                    time.sleep(0.1)

            # Stop recording and cleanup
            self.log("Stopping video recording...")
            vid.stoprecording()
            cv2.destroyAllWindows()

            # Log result
            elapsed = time.time() - start_time
            if detected_obj:
                self.log(f"[OK] Object detected at Target {target_num}: {detected_obj} (in {elapsed:.1f}s)")
                detected_objects.append((target_num, detected_obj, detected_confidence))

                # Update GUI with detected object
                self.root.after(0, lambda: self.detected_listbox.insert(
                    tk.END, f"T{target_num}: {detected_obj} ({detected_confidence:.2%})"
                ))

                # Display detection image in GUI
                if detected_frame is not None:
                    # Create a closure to capture the correct frame
                    frame_to_display = detected_frame.copy()
                    self.root.after(0, lambda f=frame_to_display: self.display_detection_image(
                        f, detected_obj, detected_confidence, target_num
                    ))
            else:
                self.log(f"[X] No object detected at Target {target_num} after {hover_time} seconds")

            return detected_obj

        except Exception as e:
            import traceback
            self.log(f"[X] Detection error at Target {target_num}: {e}")
            self.log(f"Traceback: {traceback.format_exc()}")
            try:
                vid.stoprecording()
                cv2.destroyAllWindows()
            except:
                pass
            return None

    def start_mapping(self):
        if not self.connected:
            messagebox.showwarning("Warning", "Please connect to drone first!")
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
        thread = threading.Thread(target=self.run_complete_mapping)
        thread.daemon = True
        thread.start()

    def run_complete_mapping(self):
        global api, all_walls, is_running, stop_requested

        try:
            rows = self.rows.get()
            cols = self.cols.get()
            start = (self.start_x.get(), self.start_y.get())

            BLOCK_SIZE = 60
            FLIGHT_HEIGHT = 90  # Standard height matching other working scripts
            OFFSET_X = 0
            OFFSET_Y = 0
            FIRST_BLOCK_CENTER_X = 15 + OFFSET_X
            FIRST_BLOCK_CENTER_Y = 15 + OFFSET_Y
            OBSTACLE_CHECK_COUNT = 3

            def block_to_cm(bx, by):
                return FIRST_BLOCK_CENTER_X + (bx * BLOCK_SIZE), FIRST_BLOCK_CENTER_Y + (by * BLOCK_SIZE)

            def move_to_block(bx, by):
                ax, ay = block_to_cm(bx, by)
                self.log(f"  [DEBUG] Flying to block ({bx},{by}) -> cm coords: x={ax}, y={ay}, z={FLIGHT_HEIGHT}")
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

            self.log("Enabling obstacle detection...")
            api.single_fly_barrier_aircraft(True)

            self.log("Turning on QR positioning...")
            api.Plane_cmd_switch_QR(0)
            time.sleep(3)  # Wait longer for QR positioning to stabilize

            # Try to get current position to verify QR is working
            try:
                pos = api.Plane_getPos()
                self.log(f"  [DEBUG] Current position from QR: {pos}")
            except Exception as e:
                self.log(f"  [DEBUG] Could not get position: {e}")

            self.log("Taking off...")
            api.single_fly_takeoff()
            time.sleep(3)

            # Check position after takeoff
            try:
                pos = api.Plane_getPos()
                self.log(f"  [DEBUG] Position after takeoff: {pos}")
            except Exception as e:
                self.log(f"  [DEBUG] Could not get position: {e}")

            self.log(f"Moving to mapping start: {start}")
            move_to_block(start[0], start[1])
            time.sleep(2)

            # BFS to explore entire maze efficiently
            current_pos = start
            visited = set()
            visited.add(current_pos)
            self.mapped_cells.add(current_pos)
            detected_walls = set()
            path_history = [current_pos]

            self.log("=== Starting Complete Maze Exploration ===")
            self.log("Strategy: Depth-first exploration with backtracking")
            self.log("- Always move to adjacent unvisited cells when possible (efficient)")
            self.log("- Only use BFS pathfinding when backtracking is needed")
            self.log("- No diagonal movements, only up/down/left/right")

            def explore_cell(pos):
                """Explore current cell and detect walls"""
                cx, cy = pos
                obstacles = get_verified_obstacles()
                self.log(f"Exploring ({cx},{cy}) | Obstacles: {obstacles}")

                # Update visualization
                self.update_mapping_plot(cx, cy, obstacles, path_history,
                                         detected_walls, rows, cols, start)

                # Record walls
                new_walls = get_walls_from_obstacles(obstacles, cx, cy)
                for w in new_walls:
                    detected_walls.add(w)

                return obstacles

            def get_unvisited_accessible_neighbors(pos, walls):
                """Get unvisited neighbors that are accessible (no wall blocking)"""
                cx, cy = pos
                accessible = []

                # Check all 4 directions (no diagonals)
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nx, ny = cx + dx, cy + dy

                    # Check bounds
                    if not (0 <= nx < cols and 0 <= ny < rows):
                        continue

                    # Check if already visited
                    if (nx, ny) in visited:
                        continue

                    # Check if wall blocks this edge
                    edge = tuple(sorted([(cx, cy), (nx, ny)]))
                    if edge not in walls:
                        accessible.append((nx, ny))

                return accessible

            def find_nearest_unvisited(from_pos, walls):
                """Find nearest unvisited cell using BFS"""
                if from_pos in visited:
                    # BFS to find nearest unvisited cell
                    queue = deque([(from_pos, [from_pos])])
                    bfs_visited = {from_pos}

                    while queue:
                        pos, path = queue.popleft()

                        # Get accessible neighbors
                        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                            nx, ny = pos[0] + dx, pos[1] + dy

                            if not (0 <= nx < cols and 0 <= ny < rows):
                                continue

                            if (nx, ny) in bfs_visited:
                                continue

                            # Check wall
                            edge = tuple(sorted([pos, (nx, ny)]))
                            if edge in walls:
                                continue

                            new_path = path + [(nx, ny)]

                            # If this is unvisited, return it
                            if (nx, ny) not in visited:
                                return (nx, ny), new_path

                            bfs_visited.add((nx, ny))
                            queue.append(((nx, ny), new_path))

                return None, None

            # Start exploration from initial position
            explore_cell(current_pos)

            # Continue until all accessible cells are visited
            exploration_steps = 0
            max_steps = rows * cols * 3  # Safety limit

            while exploration_steps < max_steps and not stop_requested:
                exploration_steps += 1

                # Try to move to an adjacent unvisited cell first (greedy approach)
                accessible_neighbors = get_unvisited_accessible_neighbors(current_pos, detected_walls)

                if accessible_neighbors:
                    # Move to nearest unvisited neighbor (depth-first style for efficiency)
                    next_pos = accessible_neighbors[0]

                    self.log(f"Moving to adjacent unvisited cell: {next_pos}")
                    move_to_block(next_pos[0], next_pos[1])
                    time.sleep(1.5)

                    current_pos = next_pos
                    visited.add(current_pos)
                    self.mapped_cells.add(current_pos)
                    path_history.append(current_pos)

                    # Explore this cell
                    explore_cell(current_pos)
                    time.sleep(0.5)

                else:
                    # No adjacent unvisited cells, find nearest unvisited cell via BFS
                    self.log(f"No adjacent unvisited cells, searching for nearest...")
                    nearest_unvisited, path_to_nearest = find_nearest_unvisited(current_pos, detected_walls)

                    if nearest_unvisited is None:
                        # All accessible cells have been visited
                        self.log("All accessible cells have been explored!")
                        break

                    self.log(f"Navigating to nearest unvisited cell: {nearest_unvisited}")

                    # Navigate to nearest unvisited cell
                    for i in range(1, len(path_to_nearest)):
                        if stop_requested:
                            break

                        next_pos = path_to_nearest[i]
                        move_to_block(next_pos[0], next_pos[1])
                        current_pos = next_pos
                        path_history.append(current_pos)
                        time.sleep(1.5)

                    if stop_requested:
                        break

                    # Mark as visited and explore
                    visited.add(current_pos)
                    self.mapped_cells.add(current_pos)
                    explore_cell(current_pos)
                    time.sleep(0.5)

            # Landing
            self.log("Complete maze mapping finished - Landing...")
            api.single_fly_touchdown()
            self.log("[OK] Landed!")

            # Save walls
            all_walls = detected_walls

            # Save comprehensive maze data to JSON
            maze_data = {
                "maze_info": {
                    "rows": rows,
                    "cols": cols,
                    "total_cells": rows * cols,
                    "cells_mapped": len(self.mapped_cells),
                    "walls_detected": len(all_walls),
                    "mapping_date": time.strftime("%Y-%m-%d %H:%M:%S")
                },
                "mapping_start": list(start),
                "drone_initial_facing": self.drone_facing.get(),
                "mapped_cells": [list(cell) for cell in self.mapped_cells],
                "walls": [[[w[0][0], w[0][1]], [w[1][0], w[1][1]]] for w in all_walls],
            }

            filename = "challenge2_complete_map.json"
            with open(filename, 'w') as f:
                json.dump(maze_data, f, indent=2)

            self.log(f"[OK] Complete map saved to: {filename}")
            self.log(f"Mapped {len(self.mapped_cells)} out of {rows * cols} cells")
            self.log(f"Detected {len(all_walls)} walls")
            self.log("=" * 50)
            self.log("You can now close and restart the GUI")
            self.log("The saved map will be automatically loaded for racing")
            self.log("=" * 50)
            self.set_status("MAPPING COMPLETE - MAP SAVED")

            # Enable race button
            self.root.after(0, lambda: self.race_btn.state(['!disabled']))

        except Exception as e:
            self.log(f"Error: {e}")
            self.set_status("ERROR")
        finally:
            is_running = False
            self.root.after(0, lambda: self.map_btn.state(['!disabled']))
            self.root.after(0, lambda: self.stop_btn.state(['disabled']))

    def update_mapping_plot(self, cx, cy, obstacles, path_history, detected_walls, rows, cols, start):
        BLOCK = 0.6

        def update():
            self.ax.clear()

            self.ax.set_xlim(-0.1, cols * BLOCK + 0.1)
            self.ax.set_ylim(-0.1, rows * BLOCK + 0.1)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title(f'MAPPING | Pos: ({cx},{cy}) | Cells: {len(self.mapped_cells)}/{rows * cols}')

            # Draw grid
            for i in range(cols + 1):
                self.ax.axvline(i * BLOCK, color='black', linewidth=2)
            for i in range(rows + 1):
                self.ax.axhline(i * BLOCK, color='black', linewidth=2)

            # Draw mapped cells
            for mx, my in self.mapped_cells:
                self.ax.add_patch(Rectangle((mx * BLOCK + 0.02, my * BLOCK + 0.02),
                                            BLOCK - 0.04, BLOCK - 0.04, facecolor='lightblue', alpha=0.3))

            # Draw start
            self.ax.add_patch(Rectangle((start[0] * BLOCK + 0.02, start[1] * BLOCK + 0.02),
                                        BLOCK - 0.04, BLOCK - 0.04, facecolor='green', alpha=0.4))

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
                px = [p[0] * BLOCK + BLOCK / 2 for p in path_history]
                py = [p[1] * BLOCK + BLOCK / 2 for p in path_history]
                self.ax.plot(px, py, 'b-', linewidth=2, alpha=0.6)

            # Draw drone
            self.ax.plot(cx * BLOCK + BLOCK / 2, cy * BLOCK + BLOCK / 2, 'bo', markersize=20)

            self.canvas.draw()

        self.root.after(0, update)

    def start_race(self):
        if not self.connected:
            messagebox.showwarning("Warning", "Please connect to drone first!")
            return
        if not all_walls:
            messagebox.showwarning("Warning", "Please complete mapping first!")
            return
        if not self.race_targets:
            messagebox.showwarning("Warning", "Please add at least one race target!")
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
        global api, all_walls, is_running, stop_requested

        try:
            rows = self.rows.get()
            cols = self.cols.get()
            race_start = (self.race_start_x.get(), self.race_start_y.get())
            targets = self.race_targets.copy()  # Include facing info
            level = self.aggressiveness.get()

            TIMING = {
                1: {"name": "Normal", "takeoff": 2.5, "start": 1.5, "wp": 1.0, "target": 2.0, "center": 1.0},
                2: {"name": "Fast", "takeoff": 2.0, "start": 1.0, "wp": 0.5, "target": 1.0, "center": 0.8},
                3: {"name": "Aggressive", "takeoff": 1.5, "start": 0.5, "wp": 0.2, "target": 0.5, "center": 0.5},
                4: {"name": "TURBO", "takeoff": 1.0, "start": 0.3, "wp": 0.0, "target": 0.2, "center": 0.3},
                5: {"name": "INSANE", "takeoff": 0.5, "start": 0.1, "wp": 0.0, "target": 0.1, "center": 0.2},
            }
            timing = TIMING[level]

            BLOCK_SIZE = 60
            FLIGHT_HEIGHT = 90  # Standard height matching mapping phase
            FIRST_BLOCK_CENTER_X = 15
            FIRST_BLOCK_CENTER_Y = 15

            def block_to_cm(bx, by):
                return FIRST_BLOCK_CENTER_X + (bx * BLOCK_SIZE), FIRST_BLOCK_CENTER_Y + (by * BLOCK_SIZE)

            def move_to_block(bx, by):
                ax, ay = block_to_cm(bx, by)
                self.log(f"  [DEBUG] Flying to block ({bx},{by}) -> cm: x={ax}, y={ay}, z={FLIGHT_HEIGHT}")
                api.single_fly_straight_flight(ax, ay, FLIGHT_HEIGHT)

            def center_on_cell(bx, by):
                """Explicitly center the drone on the cell before rotating"""
                ax, ay = block_to_cm(bx, by)
                self.log(f"  [CENTERING] Centering on cell ({bx},{by}) -> cm: x={ax}, y={ay}, z={FLIGHT_HEIGHT}")
                api.single_fly_straight_flight(ax, ay, FLIGHT_HEIGHT)

            self.log(f"=== RACE START - Level {level} ({timing['name']}) ===")
            self.log(f"Initial facing: {self.current_facing}")

            # Enable QR positioning for race (critical!)
            self.log("Enabling QR positioning...")
            api.Plane_cmd_switch_QR(0)
            time.sleep(2)

            self.log("Taking off...")
            api.single_fly_takeoff()
            time.sleep(timing["takeoff"])

            # Move to race start position at correct altitude
            # This single command handles both position and altitude
            self.log(f"Moving to race start: {race_start}")
            move_to_block(race_start[0], race_start[1])
            time.sleep(timing["start"])

            current_pos = race_start
            flight_history = [current_pos]
            race_start_time = time.time()

            for target_idx, target_info in enumerate(targets):
                if stop_requested:
                    break

                target_num = target_idx + 1
                tx, ty, target_facing = target_info
                target = (tx, ty)

                self.log(f"Racing to Target {target_num}: {target}, Face: {target_facing}")
                self.set_status(f"RACING - Target {target_num}/{len(targets)}")

                # Find path from current position to target
                path = self.find_path_bfs(current_pos, target, all_walls, rows, cols)
                if not path:
                    self.log(f"No path to Target {target_num}!")
                    continue

                waypoints = self.simplify_path(path)

                # Skip first waypoint if it's current position
                start_idx = 1 if waypoints and tuple(waypoints[0]) == current_pos else 0

                for wp in waypoints[start_idx:]:
                    if stop_requested:
                        break

                    wp_x, wp_y = wp if isinstance(wp, tuple) else tuple(wp)
                    move_to_block(wp_x, wp_y)

                    current_pos = (wp_x, wp_y)
                    flight_history.append(current_pos)

                    self.update_race_plot(current_pos[0], current_pos[1], flight_history, rows, cols,
                                          targets, race_start, self.current_facing)

                    if timing["wp"] > 0:
                        time.sleep(timing["wp"])

                # Target reached - EXPLICITLY CENTER on the target cell
                self.log(f"[OK] Target {target_num} REACHED!")
                self.log(f"Centering on target cell before rotation...")

                # Explicitly center on the target cell
                center_on_cell(tx, ty)
                time.sleep(timing["center"])  # Wait for centering to complete

                self.log(f"Centered on target - ready to rotate")

                # Now rotate to target facing direction
                if target_facing != self.current_facing:
                    self.log(f"Rotating to face {target_facing}...")
                    self.rotate_to_facing(target_facing)
                    # Allow rotation to complete fully
                    time.sleep(0.5)

                    self.update_race_plot(current_pos[0], current_pos[1], flight_history, rows, cols,
                                          targets, race_start, self.current_facing)

                # Hover and detect object at this target (if enabled)
                if self.enable_detection.get():
                    self.log(f"Hovering at Target {target_num} for object detection...")
                    detected_obj = self.detect_object_at_target(target_num)
                    # Allow detection to complete fully
                    time.sleep(0.5)
                else:
                    self.log(f"Object detection disabled - skipping")

                # Additional wait at target if configured
                if timing["target"] > 0:
                    time.sleep(timing["target"])

            race_time = time.time() - race_start_time

            self.log("Landing...")
            api.single_fly_touchdown()
            self.log("[OK] Landed!")

            self.log(f"=== RACE COMPLETE: {race_time:.1f} seconds ===")

            # Log detected objects
            if detected_objects:
                self.log("=== DETECTED OBJECTS ===")
                for item in detected_objects:
                    if len(item) == 3:
                        target_num, obj_name, confidence = item
                        self.log(f"Target {target_num}: {obj_name} (confidence: {confidence:.2%})")
                    else:
                        # Backward compatibility
                        target_num, obj_name = item
                        self.log(f"Target {target_num}: {obj_name}")

            # Save race results with detected objects
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            # Format detected objects for JSON
            detected_objects_json = []
            for item in detected_objects:
                if len(item) == 3:
                    target_num, obj_name, confidence = item
                    detected_objects_json.append({
                        "target": target_num,
                        "object": obj_name,
                        "confidence": float(confidence)
                    })
                else:
                    # Backward compatibility
                    target_num, obj_name = item
                    detected_objects_json.append({
                        "target": target_num,
                        "object": obj_name
                    })

            race_results = {
                "race_date": time.strftime("%Y-%m-%d %H:%M:%S"),
                "race_time_seconds": race_time,
                "race_start": list(race_start),
                "targets": [[t[0], t[1], t[2]] for t in targets],
                "aggressiveness_level": level,
                "detection_enabled": self.enable_detection.get(),
                "detection_hover_time": self.detection_hover_time.get(),
                "detected_objects": detected_objects_json,
                "maze_file": "challenge2_complete_map.json"
            }

            results_filename = f"challenge2_race_results_{timestamp}.json"
            with open(results_filename, 'w') as f:
                json.dump(race_results, f, indent=2)

            self.log(f"[OK] Race results saved to {results_filename}")
            self.set_status(f"FINISHED - {race_time:.1f}s")

        except Exception as e:
            import traceback
            self.log(f"Error: {e}")
            self.log(f"Full traceback: {traceback.format_exc()}")
            self.set_status("ERROR")

            # Cleanup video
            try:
                if vid is not None:
                    vid.stoprecording()
                    vid.close()
                    cv2.destroyAllWindows()
            except:
                pass

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
                self.ax.text(tx * BLOCK + BLOCK / 2, ty * BLOCK + BLOCK - 0.1, f"T{i + 1}",
                             ha='center', va='top', fontsize=10, fontweight='bold')
                # Draw target facing arrow
                self.draw_facing_arrow_on_ax(self.ax, tx, ty, facing, 'black')

            # Draw flight path
            if flight_history:
                px = [p[0] * BLOCK + BLOCK / 2 for p in flight_history]
                py = [p[1] * BLOCK + BLOCK / 2 for p in flight_history]
                self.ax.plot(px, py, 'b-', linewidth=3, alpha=0.7)

            # Draw drone with facing arrow
            self.ax.plot(cx * BLOCK + BLOCK / 2, cy * BLOCK + BLOCK / 2, 'bo', markersize=20)
            self.draw_facing_arrow_on_ax(self.ax, cx, cy, current_facing, 'blue')

            self.canvas.draw()

        self.root.after(0, update)

    def draw_facing_arrow_on_ax(self, ax, x, y, facing, color):
        """Draw an arrow showing facing direction on given axes"""
        BLOCK = 0.6
        cx = x * BLOCK + BLOCK / 2
        cy = y * BLOCK + BLOCK / 2
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
        global api, vid
        if api and self.connected:
            self.log("Emergency landing...")

            # Close video if active
            try:
                if vid is not None:
                    vid.stoprecording()
                    vid.close()
                    cv2.destroyAllWindows()
                    self.log("Video streaming stopped")
            except:
                pass

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
            prev, curr, next_pt = path[i - 1], path[i], path[i + 1]
            dir1 = (curr[0] - prev[0], curr[1] - prev[1])
            dir2 = (next_pt[0] - curr[0], next_pt[1] - curr[1])
            if dir1 != dir2:
                simplified.append(curr)
        simplified.append(path[-1])
        return simplified


# Main
if __name__ == "__main__":
    root = tk.Tk()
    app = Challenge2GUI(root)
    root.mainloop()
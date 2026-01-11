"""
Mission Control GUI - Maze Simulation
Simple GUI to configure and run maze exploration simulations
"""
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import sys
from io import StringIO
import time
from collections import deque

# Import matplotlib for embedding plots
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

# Import the maze simulation code
import discover11

class OutputRedirector:
    """Redirect stdout to the GUI text widget"""
    def __init__(self, text_widget):
        self.text_widget = text_widget
        
    def write(self, string):
        self.text_widget.insert(tk.END, string)
        self.text_widget.see(tk.END)
        self.text_widget.update_idletasks()
        
    def flush(self):
        pass

class MissionControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Simulation Control")
        self.root.geometry("1200x700")
        
        # Mission running state
        self.mission_running = False
        self.race_running = False
        self.current_maze = None
        self.current_robot = None
        self.maze_data = None  # Store maze_data.json content
        
        # Visualization delay (seconds between updates)
        self.visualization_delay = 0.20  # Default 50ms
        
        # Configure grid for layout
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        
        # Create main frames
        self.create_input_frame()
        self.create_control_frame()
        self.create_output_frame()
        self.create_plot_frame()
        
    def create_input_frame(self):
        """Create input parameters frame"""
        input_frame = ttk.LabelFrame(self.root, text="Simulation Parameters", padding=10)
        input_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=5)
        
        # Maze size
        ttk.Label(input_frame, text="Maze Size (NxN):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.maze_size_var = tk.StringVar(value="5")
        maze_size_entry = ttk.Entry(input_frame, textvariable=self.maze_size_var, width=10)
        maze_size_entry.grid(row=0, column=1, sticky=tk.W, pady=5, padx=5)
        ttk.Label(input_frame, text="cells").grid(row=0, column=2, sticky=tk.W)
        
        # Start position
        ttk.Label(input_frame, text="Start Position (row, col):").grid(row=1, column=0, sticky=tk.W, pady=5)
        start_frame = ttk.Frame(input_frame)
        start_frame.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)
        self.start_row_var = tk.StringVar(value="0")
        self.start_col_var = tk.StringVar(value="0")
        ttk.Entry(start_frame, textvariable=self.start_row_var, width=5).pack(side=tk.LEFT)
        ttk.Label(start_frame, text=",").pack(side=tk.LEFT, padx=2)
        ttk.Entry(start_frame, textvariable=self.start_col_var, width=5).pack(side=tk.LEFT)
        
        # End position
        ttk.Label(input_frame, text="End Position (row, col):").grid(row=2, column=0, sticky=tk.W, pady=5)
        end_frame = ttk.Frame(input_frame)
        end_frame.grid(row=2, column=1, sticky=tk.W, pady=5, padx=5)
        self.end_row_var = tk.StringVar(value="4")
        self.end_col_var = tk.StringVar(value="4")
        ttk.Entry(end_frame, textvariable=self.end_row_var, width=5).pack(side=tk.LEFT)
        ttk.Label(end_frame, text=",").pack(side=tk.LEFT, padx=2)
        ttk.Entry(end_frame, textvariable=self.end_col_var, width=5).pack(side=tk.LEFT)
        
        # Visualization speed
        ttk.Label(input_frame, text="Visualization Delay (ms):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.delay_var = tk.StringVar(value="50")
        delay_entry = ttk.Entry(input_frame, textvariable=self.delay_var, width=10)
        delay_entry.grid(row=3, column=1, sticky=tk.W, pady=5, padx=5)
        ttk.Label(input_frame, text="(lower = faster)", 
                 font=("Arial", 8), foreground="gray").grid(row=3, column=2, sticky=tk.W)
        
    def create_control_frame(self):
        """Create control buttons frame"""
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=5)
        
        # Discovery button
        self.start_btn = ttk.Button(control_frame, text="▶ Start Discovery", 
                                    command=self.start_mission, style="Accent.TButton")
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        # Race button (disabled until discovery completes)
        self.race_btn = ttk.Button(control_frame, text="▶ Start Race", 
                                   command=self.start_race, state=tk.DISABLED)
        self.race_btn.pack(side=tk.LEFT, padx=5)
        
        # Stop button (disabled initially)
        self.stop_btn = ttk.Button(control_frame, text="⬛ Emergency Stop", 
                                   command=self.emergency_stop, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Clear output button
        clear_btn = ttk.Button(control_frame, text="Clear Output", 
                              command=self.clear_output)
        clear_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.status_var, 
                                font=("Arial", 10, "bold"))
        status_label.pack(side=tk.RIGHT, padx=10)
        
    def create_output_frame(self):
        """Create output console frame"""
        output_frame = ttk.LabelFrame(self.root, text="Simulation Output", padding=10)
        output_frame.grid(row=2, column=0, sticky="nsew", padx=(10, 5), pady=5)
        
        # Create scrolled text widget
        self.output_text = scrolledtext.ScrolledText(output_frame, 
                                                     wrap=tk.WORD,
                                                     width=50, 
                                                     height=25,
                                                     font=("Consolas", 9))
        self.output_text.pack(fill=tk.BOTH, expand=True)
    
    def create_plot_frame(self):
        """Create matplotlib plot frame"""
        plot_frame = ttk.LabelFrame(self.root, text="Maze Visualization", padding=10)
        plot_frame.grid(row=2, column=1, sticky="nsew", padx=(5, 10), pady=5)
        
        # Create matplotlib figure and canvas
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initialize empty plot
        self.ax.set_xlim(0, 150)
        self.ax.set_ylim(0, 150)
        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.ax.text(75, 75, 'Waiting for simulation...', 
                    ha='center', va='center', fontsize=12, color='gray')
        self.canvas.draw()
        
    def clear_output(self):
        """Clear the output text widget"""
        self.output_text.delete(1.0, tk.END)
    
    def draw_maze_state(self, maze, robot, frame_idx):
        """Draw the current state of the maze exploration"""
        if frame_idx >= len(robot.path):
            return
            
        self.ax.clear()
        
        # Adjust limits based on maze size
        limit = maze.size * 30
        self.ax.set_xlim(0, limit)
        self.ax.set_ylim(0, limit)
        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        
        # Draw cells
        for r in range(maze.size):
            for c in range(maze.size):
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        linewidth=0.5, edgecolor='lightgray', 
                                        facecolor='white')
                self.ax.add_patch(rect)
        
        # Get current frame data
        current_pos, current_count, current_barriers = robot.path[frame_idx]
        
        # Collect all sensed barriers up to this frame
        sensed_so_far = set()
        for i in range(frame_idx + 1):
            _, _, barriers = robot.path[i]
            sensed_so_far.update(barriers)
        
        # Draw walls
        for r in range(maze.size + 1):
            for c in range(maze.size):
                if r < len(maze.h_walls) and c < len(maze.h_walls[r]) and maze.h_walls[r][c]:
                    wall = ('h', r, c)
                    if wall in current_barriers:
                        color = 'yellow'
                        width = 4
                    elif wall in sensed_so_far:
                        color = 'blue'
                        width = 3
                    else:
                        color = 'black'
                        width = 3
                    self.ax.plot([c*30, (c+1)*30], [r*30, r*30], color=color, linewidth=width)
        
        for r in range(maze.size):
            for c in range(maze.size + 1):
                if r < len(maze.v_walls) and c < len(maze.v_walls[r]) and maze.v_walls[r][c]:
                    wall = ('v', r, c)
                    if wall in current_barriers:
                        color = 'yellow'
                        width = 4
                    elif wall in sensed_so_far:
                        color = 'blue'
                        width = 3
                    else:
                        color = 'black'
                        width = 3
                    self.ax.plot([c*30, c*30], [r*30, (r+1)*30], color=color, linewidth=width)
        
        # Draw visited trail
        for i in range(frame_idx):
            p, _, _ = robot.path[i]
            if p != current_pos and p != maze.start and p != maze.target:
                r, c = p
                circle = patches.Circle((c*30+15, r*30+15), 4, facecolor='lightblue')
                self.ax.add_patch(circle)
        
        # Draw shortest path if available
        if robot.shortest_path and frame_idx >= len(robot.path) - len(robot.shortest_path):
            for p in robot.shortest_path:
                if p != maze.start and p != maze.target:
                    r, c = p
                    circle = patches.Circle((c*30+15, r*30+15), 6,
                                           facecolor='lime', edgecolor='darkgreen', linewidth=1)
                    self.ax.add_patch(circle)
        
        # Draw start
        r, c = maze.start
        rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightgreen', alpha=0.6)
        self.ax.add_patch(rect)
        self.ax.text(c*30+15, r*30+15, 'S', ha='center', va='center', fontsize=12, weight='bold')
        
        # Draw target
        r, c = maze.target
        rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightcoral', alpha=0.6)
        self.ax.add_patch(rect)
        self.ax.text(c*30+15, r*30+15, 'T', ha='center', va='center', fontsize=12, weight='bold')
        
        # Draw robot
        r, c = current_pos
        circle = patches.Circle((c*30+15, r*30+15), 10,
                               facecolor='orange', edgecolor='darkorange', linewidth=2)
        self.ax.add_patch(circle)
        self.ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                   fontsize=12, weight='bold', color='white')
        
        # Title
        path_len = len(robot.shortest_path) if robot.shortest_path else 0
        title = f'Visited: {current_count}/{maze.size*maze.size} | Shortest Path: {path_len} cells\n'
        title += 'Yellow=Sensing | Blue=Sensed | Green=Shortest Path'
        self.ax.set_title(title, fontsize=10, weight='bold', pad=10)
        
        # Update canvas
        self.canvas.draw()
        
    def start_mission(self):
        """Start the simulation in a separate thread"""
        if self.mission_running:
            self.output_text.insert(tk.END, "Simulation already running!\n")
            return
            
        # Validate inputs
        try:
            maze_size = int(self.maze_size_var.get())
            if maze_size < 2 or maze_size > 20:
                self.output_text.insert(tk.END, "ERROR: Maze size must be between 2 and 20.\n")
                return
            
            # Validate start position
            start_row = int(self.start_row_var.get())
            start_col = int(self.start_col_var.get())
            if start_row < 0 or start_row >= maze_size or start_col < 0 or start_col >= maze_size:
                self.output_text.insert(tk.END, f"ERROR: Start position must be within 0-{maze_size-1}.\n")
                return
            
            # Validate end position
            end_row = int(self.end_row_var.get())
            end_col = int(self.end_col_var.get())
            if end_row < 0 or end_row >= maze_size or end_col < 0 or end_col >= maze_size:
                self.output_text.insert(tk.END, f"ERROR: End position must be within 0-{maze_size-1}.\n")
                return
            
            # Check that start and end are different
            if (start_row, start_col) == (end_row, end_col):
                self.output_text.insert(tk.END, "ERROR: Start and end positions must be different.\n")
                return
            
            # Update visualization delay
            delay_ms = int(self.delay_var.get())
            if delay_ms < 0:
                self.output_text.insert(tk.END, "ERROR: Delay must be non-negative.\n")
                return
            self.visualization_delay = delay_ms / 1000.0  # Convert to seconds
        except ValueError:
            self.output_text.insert(tk.END, "ERROR: Invalid input values. Please enter valid numbers.\n")
            return
            
        # Update UI state
        self.mission_running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("Simulation Running...")
        
        # Redirect stdout to output text widget
        sys.stdout = OutputRedirector(self.output_text)
        
        # Run simulation in separate thread to keep GUI responsive
        mission_thread = threading.Thread(
            target=self.run_mission,
            args=(maze_size, (start_row, start_col), (end_row, end_col)),
            daemon=True
        )
        mission_thread.start()
        
    def run_mission(self, maze_size, start_pos, end_pos):
        """Run the actual simulation (called in separate thread)"""
        try:
            print(f"\n{'='*60}")
            print(f"Starting Maze Simulation")
            print(f"Maze Size: {maze_size}x{maze_size}")
            print(f"Start: {start_pos}, End: {end_pos}")
            print(f"{'='*60}\n")
            
            # Create maze with specified size
            maze = discover11.Maze(size=maze_size)
            maze.generate()
            
            # Override start and target positions BEFORE creating robot
            maze.start = start_pos
            maze.target = end_pos
            
            self.current_maze = maze
            
            print(f"Maze generated with custom positions")
            
            # Create robot - it will automatically use maze.start as initial position
            robot = discover11.Robot(maze)
            # Robot is already at correct position from maze.start
            self.current_robot = robot
            
            # Run exploration with live updates
            self.explore_with_visualization(robot)
            
            # Save maze data
            discover11.save_maze_data(maze, robot)
            
            # Load and store maze data for race phase
            import json
            try:
                with open('maze_data.json', 'r') as f:
                    self.maze_data = json.load(f)
            except:
                print("Warning: Could not load maze_data.json")
            
            print(f"\n{'='*60}")
            print("Discovery Complete!")
            print(f"{'='*60}\n")
            print("Maze discovered and mapped. Data saved to 'maze_data.json'.")
            print("Ready to start race phase!\n")
            
            self.mission_complete(success=True)
            
        except Exception as e:
            import traceback
            print(f"\nERROR: Simulation failed with exception:\n")
            traceback.print_exc()
            self.mission_complete(success=False)
    
    def explore_with_visualization(self, robot):
        """Run robot exploration with live GUI updates"""
        # Copy the exploration logic from discover11.Robot.explore()
        # but with GUI updates after each move
        maze = robot.maze
        robot.visited.add(robot.pos)
        robot.path.append((robot.pos, len(robot.visited), robot.sense()))
        
        # Update GUI with initial state
        self.root.after(0, lambda: self.draw_maze_state(maze, robot, 0))
        time.sleep(0.1)
        
        stack = [robot.pos]
        frame_idx = 0
        
        while stack and len(robot.visited) < maze.size * maze.size:
            moves = []
            for direction in ['up', 'down', 'left', 'right']:
                if maze.can_move(robot.pos, direction):
                    r, c = robot.pos
                    next_pos = {
                        'up': (r-1, c), 'down': (r+1, c),
                        'left': (r, c-1), 'right': (r, c+1)
                    }[direction]
                    if next_pos not in robot.visited:
                        moves.append((next_pos, direction))
            
            if moves:
                next_pos, direction = moves[0]
                robot.visited.add(next_pos)
                robot.move(direction)
                stack.append(robot.pos)
                current_barriers = robot.sense()
                robot.path.append((robot.pos, len(robot.visited), current_barriers))
                frame_idx += 1
                
                # Update GUI
                self.root.after(0, lambda idx=frame_idx: self.draw_maze_state(maze, robot, idx))
                time.sleep(self.visualization_delay)
            else:
                stack.pop()
                if stack:
                    target = stack[-1]
                    r1, c1 = robot.pos
                    r2, c2 = target
                    if r2 < r1: robot.move('up')
                    elif r2 > r1: robot.move('down')
                    elif c2 < c1: robot.move('left')
                    else: robot.move('right')
                    current_barriers = robot.sense()
                    robot.path.append((robot.pos, len(robot.visited), current_barriers))
                    frame_idx += 1
                    
                    # Update GUI
                    self.root.after(0, lambda idx=frame_idx: self.draw_maze_state(maze, robot, idx))
                    time.sleep(self.visualization_delay)
        
        print(f"\n✓ Exploration complete! All {len(robot.visited)} cells visited.")
        print(f"  Current position: {robot.pos}")
        print(f"  Discovery phase finished. Ready to start race!")
            
    def start_race(self):
        """Start the race phase using discovered maze data"""
        if self.race_running:
            self.output_text.insert(tk.END, "Race already running!\n")
            return
        
        if not self.maze_data:
            self.output_text.insert(tk.END, "ERROR: No maze data available. Run discovery first!\n")
            return
        
        # Update UI state
        self.race_running = True
        self.race_btn.config(state=tk.DISABLED)
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("Race Running...")
        
        # Run race in separate thread
        race_thread = threading.Thread(
            target=self.run_race,
            daemon=True
        )
        race_thread.start()
    
    def run_race(self):
        """Run the race using maze_data.json"""
        try:
            import random
            from collections import deque
            
            print(f"\n{'='*60}")
            print(f"Starting Race Phase")
            print(f"{'='*60}\n")
            
            # Check if maze data exists
            if not self.maze_data:
                print("ERROR: No maze data available!")
                self.race_complete(success=False)
                return
            
            # Load maze data
            maze_map = self.maze_data.get('maze_map')
            sensed_barriers = self.maze_data.get('sensed_barriers')
            
            if not maze_map or not sensed_barriers:
                print("ERROR: Invalid maze data!")
                self.race_complete(success=False)
                return
                
            size = maze_map['size']
            
            print(f"Loaded maze data: {size}x{size}, {len(sensed_barriers)} barriers")
            
            # Use the start and end positions from GUI inputs
            try:
                start_row = int(self.start_row_var.get())
                start_col = int(self.start_col_var.get())
                end_row = int(self.end_row_var.get())
                end_col = int(self.end_col_var.get())
                start_position = (start_row, start_col)
                target_position = (end_row, end_col)
            except:
                # Fallback to random positions
                all_cells = [(r, c) for r in range(size) for c in range(size)]
                start_position = random.choice(all_cells)
                all_cells.remove(start_position)
                target_position = random.choice(all_cells)
            
            print(f"Race Start: {start_position}")
            print(f"Race Target: {target_position}")
            
            # Find shortest path using sensed barriers
            shortest_path = self.find_shortest_path_race(
                start_position, target_position, sensed_barriers, size)
            
            if not shortest_path:
                print("ERROR: No path found!")
                self.race_complete(success=False)
                return
            
            print(f"\nShortest path found: {len(shortest_path)} steps")
            print(f"Starting navigation...\n")
            
            # Animate the race
            self.animate_race(shortest_path, start_position, target_position, sensed_barriers, size)
            
            print(f"\n{'='*60}")
            print("Race Complete!")
            print(f"{'='*60}\n")
            print(f"Total steps: {len(shortest_path)}")
            print(f"Total distance: {(len(shortest_path) - 1) * 30} cm\n")
            
            self.race_complete(success=True)
            
        except Exception as e:
            import traceback
            print(f"\nERROR: Race failed with exception:\n")
            traceback.print_exc()
            self.race_complete(success=False)
    
    def find_shortest_path_race(self, start, target, sensed_barriers, size):
        """Find shortest path using BFS based on sensed barriers"""
        from collections import deque
        
        if start == target:
            return [start]
        
        # Convert sensed_barriers to set for faster lookup
        barrier_set = set()
        for barrier in sensed_barriers:
            barrier_set.add((barrier['type'], barrier['row'], barrier['col']))
        
        queue = deque([start])
        parent = {start: None}
        
        while queue:
            current = queue.popleft()
            if current == target:
                break
            
            for direction in ['up', 'down', 'left', 'right']:
                if self.can_move_race(current, direction, barrier_set, size):
                    r, c = current
                    next_pos = {
                        'up': (r-1, c), 'down': (r+1, c),
                        'left': (r, c-1), 'right': (r, c+1)
                    }[direction]
                    if next_pos not in parent:
                        parent[next_pos] = current
                        queue.append(next_pos)
        
        if target not in parent:
            return []
        
        # Reconstruct path
        path = []
        current = target
        while current:
            path.append(current)
            current = parent.get(current)
        path.reverse()
        return path
    
    def can_move_race(self, pos, direction, barrier_set, size):
        """Check if can move based on sensed barriers"""
        r, c = pos
        
        # Check boundaries
        if direction == 'up' and r == 0: return False
        if direction == 'down' and r == size - 1: return False
        if direction == 'left' and c == 0: return False
        if direction == 'right' and c == size - 1: return False
        
        # Check sensed barriers
        barrier_check = {
            'up': ('h', r, c),
            'down': ('h', r + 1, c),
            'left': ('v', r, c),
            'right': ('v', r, c + 1)
        }
        
        if barrier_check[direction] in barrier_set:
            return False
        
        return True
    
    def animate_race(self, path, start, target, sensed_barriers, size):
        """Animate the race path on the GUI"""
        # Clear and redraw maze for race
        print(f"Animating {len(path)} steps...")
        for i, pos in enumerate(path):
            print(f"  Step {i+1}/{len(path)}: {pos}")
            # Schedule GUI update on main thread
            self.root.after(0, lambda idx=i: self.draw_race_state(path, idx, start, target, sensed_barriers, size))
            time.sleep(self.visualization_delay * 2)
    
    def draw_race_state(self, path, current_idx, start, target, sensed_barriers, size):
        """Draw the race state"""
        self.ax.clear()
        
        # Adjust limits based on maze size
        limit = size * 30
        self.ax.set_xlim(0, limit)
        self.ax.set_ylim(0, limit)
        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        
        # Draw cells
        for r in range(size):
            for c in range(size):
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        linewidth=0.5, edgecolor='lightgray', 
                                        facecolor='white')
                self.ax.add_patch(rect)
        
        # Draw sensed barriers
        barrier_set = set()
        for barrier in sensed_barriers:
            barrier_set.add((barrier['type'], barrier['row'], barrier['col']))
        
        for barrier_type, r, c in barrier_set:
            if barrier_type == 'h':
                self.ax.plot([c*30, (c+1)*30], [r*30, r*30], color='blue', linewidth=3)
            else:  # 'v'
                self.ax.plot([c*30, c*30], [r*30, (r+1)*30], color='blue', linewidth=3)
        
        # Draw path traveled
        for i in range(current_idx):
            r, c = path[i]
            if (r, c) != start and (r, c) != target:
                circle = patches.Circle((c*30+15, r*30+15), 6,
                                       facecolor='lime', edgecolor='darkgreen', linewidth=2)
                self.ax.add_patch(circle)
        
        # Draw full shortest path as dashed line
        for i in range(len(path) - 1):
            r1, c1 = path[i]
            r2, c2 = path[i + 1]
            self.ax.plot([c1*30+15, c2*30+15], [r1*30+15, r2*30+15], 
                        'lime', linewidth=2, linestyle='--', alpha=0.5)
        
        # Draw start
        sr, sc = start
        rect = patches.Rectangle((sc*30, sr*30), 30, 30, facecolor='lightgreen', alpha=0.6)
        self.ax.add_patch(rect)
        self.ax.text(sc*30+15, sr*30+15, 'START', ha='center', va='center', 
                    fontsize=10, weight='bold', color='darkgreen')
        
        # Draw target
        tr, tc = target
        rect = patches.Rectangle((tc*30, tr*30), 30, 30, facecolor='lightcoral', alpha=0.6)
        self.ax.add_patch(rect)
        self.ax.text(tc*30+15, tr*30+15, 'TARGET', ha='center', va='center', 
                    fontsize=10, weight='bold', color='darkred')
        
        # Draw robot at current position
        if current_idx < len(path):
            r, c = path[current_idx]
            circle = patches.Circle((c*30+15, r*30+15), 13,
                                   facecolor='orange', edgecolor='darkorange', linewidth=3)
            self.ax.add_patch(circle)
            self.ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                       fontsize=16, weight='bold', color='white')
        
        # Title
        progress = (current_idx / (len(path) - 1) * 100) if len(path) > 1 else 100
        steps_remaining = len(path) - 1 - current_idx
        title = f'RACE - Optimal Path Navigation\n'
        title += f'Step {current_idx + 1}/{len(path)} | Progress: {progress:.0f}% | Remaining: {steps_remaining}'
        self.ax.set_title(title, fontsize=10, weight='bold', pad=10)
        
        self.canvas.draw()
    
    def race_complete(self, success):
        """Update UI after race completes"""
        self.race_running = False
        self.root.after(0, lambda: self.race_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.start_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.stop_btn.config(state=tk.DISABLED))
        if success:
            self.root.after(0, lambda: self.status_var.set("Race Complete ✓"))
        else:
            self.root.after(0, lambda: self.status_var.set("Race Failed ✗"))
    
    def mission_complete(self, success):
        """Update UI after simulation completes"""
        self.mission_running = False
        self.root.after(0, lambda: self.start_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.stop_btn.config(state=tk.DISABLED))
        if success:
            self.root.after(0, lambda: self.status_var.set("Discovery Complete ✓ - Ready to Race"))
            # Enable race button after successful discovery
            self.root.after(0, lambda: self.race_btn.config(state=tk.NORMAL))
        else:
            self.root.after(0, lambda: self.status_var.set("Discovery Failed ✗"))
            
    def emergency_stop(self):
        """Emergency stop - note: cannot force-stop running thread"""
        print("\n!!! EMERGENCY STOP ACTIVATED !!!")
        print("Note: Cannot forcibly stop simulation thread.")
        print("      Waiting for current operation to complete...\n")
        self.mission_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("Stop Requested")

def main():
    root = tk.Tk()
    app = MissionControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

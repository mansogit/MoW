# MissionGUI2.py - Maze Simulation Control Center

A comprehensive graphical user interface for simulating and visualizing maze exploration and navigation tasks. This application provides an interactive control panel for running maze discovery and race missions with real-time visualization.

## 🎯 Overview

MissionGUI2.py is an advanced GUI application built with Tkinter and Matplotlib that allows users to:
- Configure custom maze parameters (size, start/end positions)
- Run maze exploration (Discovery Phase)
- Execute optimal path navigation (Race Phase)
- Visualize maze exploration and navigation in real-time
- Control simulation speed and monitor progress

## ✨ Features

### Two-Phase Operation

#### 1. Discovery Phase
- **Purpose**: Explore the entire maze and map all barriers
- **Algorithm**: Depth-First Search (DFS) based exploration
- **Output**: Generates `maze_data.json` containing discovered maze structure
- **Visualization**: Shows real-time exploration with:
  - Yellow walls: Currently being sensed
  - Blue walls: Previously discovered barriers
  - Orange robot: Current position
  - Light blue trail: Visited cells

#### 2. Race Phase
- **Purpose**: Navigate optimal path from start to target
- **Algorithm**: Breadth-First Search (BFS) for shortest path
- **Optimization**: Groups consecutive moves in the same direction into "leaps"
- **Visualization**: Shows optimized navigation with:
  - Green markers: Path waypoints
  - Arrows: Multi-step leaps with step counts
  - Real-time progress tracking

### Configurable Parameters

- **Maze Size**: NxN grid (2-20 cells)
- **Start Position**: Custom starting coordinates (row, col)
- **End Position**: Custom target coordinates (row, col)
- **Visualization Delay**: Animation speed in milliseconds (lower = faster)

### Real-Time Visualization

- **Live Maze Display**: Interactive matplotlib canvas showing:
  - Cell grid with walls
  - Robot position and movement
  - Discovery progress counter
  - Path optimization visualization
  
- **Console Output**: Scrolled text widget displaying:
  - Simulation status messages
  - Progress updates
  - Performance metrics
  - Error messages

## 🚀 Getting Started

### Prerequisites

```python
# Required Python packages
tkinter          # GUI framework (usually included with Python)
matplotlib       # For maze visualization
threading        # For non-blocking simulation execution
json            # For maze data storage
```

### Dependencies

The application requires the `discover11.py` module, which should contain:
- `Maze` class: Maze generation and structure
- `Robot` class: Robot navigation logic
- `save_maze_data()`: Function to save discovered maze data

### Running the Application

```bash
python MissionGUI2.py
```

## 📖 Usage Guide

### Step 1: Configure Parameters

1. **Set Maze Size**: Enter grid dimensions (e.g., 5 for a 5x5 maze)
2. **Set Start Position**: Enter starting coordinates (row, col)
   - Example: `0, 0` for top-left corner
3. **Set End Position**: Enter target coordinates (row, col)
   - Example: `4, 4` for bottom-right corner
4. **Set Visualization Delay**: Adjust animation speed (default: 50ms)
   - Lower values = faster animation
   - Higher values = slower, easier to follow

### Step 2: Run Discovery Phase

1. Click **▶ Start Discovery** button
2. Watch the robot explore the maze using DFS
3. Monitor progress in the console output
4. Wait for "Discovery Complete!" message
5. The race button will become enabled

### Step 3: Run Race Phase

1. Click **▶ Start Race** button
2. The robot navigates the optimal path
3. Observe the optimized leaps (consecutive moves grouped)
4. View completion statistics

### Additional Controls

- **⬛ Emergency Stop**: Request simulation stop (completes current operation)
- **Clear Output**: Clear the console output window

## 🎨 Visualization Legend

### Discovery Phase
- **Green Cell**: Start position (S)
- **Red Cell**: Target position (T)
- **Orange Circle (R)**: Robot current position
- **Yellow Walls**: Currently being sensed
- **Blue Walls**: Previously discovered barriers
- **Light Blue Dots**: Visited cells
- **Lime Green Path**: Shortest path (when calculated)

### Race Phase
- **Green Cell**: START position
- **Red Cell**: TARGET position
- **Orange Circle (R)**: Robot current position
- **Green Markers**: Path waypoints
- **Green Arrows**: Multi-step leaps
- **"xN" Labels**: Number of steps in leap
- **Dashed Lime Line**: Full optimal path

## 📊 Output Files

### maze_data.json
Generated after successful discovery phase:
```json
{
  "maze_map": {
    "size": 5,
    "start": [0, 0],
    "target": [4, 4]
  },
  "sensed_barriers": [
    {"type": "h", "row": 0, "col": 1},
    {"type": "v", "row": 2, "col": 3}
  ]
}
```

- **maze_map**: Maze structure information
- **sensed_barriers**: List of discovered walls
  - `type`: 'h' (horizontal) or 'v' (vertical)
  - `row`, `col`: Wall position coordinates

## 🔧 Technical Details

### Architecture

- **Multi-threaded Design**: Simulations run in separate threads to keep GUI responsive
- **Thread-safe Updates**: Uses `root.after()` for GUI updates from worker threads
- **Output Redirection**: Captures stdout to display in GUI console

### Key Classes

#### `MissionControlGUI`
Main application class managing:
- GUI layout and widgets
- Simulation orchestration
- State management
- Visualization updates

#### `OutputRedirector`
Redirects console output to GUI text widget

### Algorithms

#### Discovery (DFS)
```python
1. Start at initial position
2. Mark current cell as visited
3. For each unvisited neighbor:
   - Move to neighbor
   - Recursively explore
4. Backtrack when no unvisited neighbors
5. Continue until all cells visited
```

#### Race (BFS Shortest Path)
```python
1. Load discovered barriers from maze_data.json
2. Use BFS to find shortest path
3. Optimize path by grouping consecutive moves
4. Navigate optimized path
```

## ⚠️ Error Handling

The application validates:
- Maze size (2-20)
- Start/end positions within bounds
- Start and end positions are different
- Maze data availability for race phase
- Valid numeric inputs

Error messages appear in the console output.

## 🎮 Example Scenarios

### Small Maze (5x5)
```
Maze Size: 5
Start: (0, 0)
End: (4, 4)
Delay: 50ms
```
**Result**: Quick exploration, easy to follow visualization

### Large Maze (15x15)
```
Maze Size: 15
Start: (0, 0)
End: (14, 14)
Delay: 10ms
```
**Result**: Comprehensive exploration, faster animation

### Custom Route
```
Maze Size: 8
Start: (2, 3)
End: (6, 5)
Delay: 100ms
```
**Result**: Explores specific route, slow motion visualization

## 🐛 Known Limitations

1. **Emergency Stop**: Cannot forcibly terminate running thread; waits for current operation completion
2. **Thread Safety**: Simulation must complete before starting new mission
3. **Memory**: Large mazes (>15x15) may use significant memory for path storage
4. **File Dependency**: Requires `discover11.py` module

## 📝 Notes

- The visualization delay affects both discovery and race phases
- Lower delays may cause flickering on slower systems
- The race phase requires successful completion of discovery phase
- All coordinates are 0-indexed (row, col format)

## 🔄 Version History

**Version 2.0**
- Added race phase with path optimization
- Enhanced visualization with leap animations
- Improved GUI layout and controls
- Added progress tracking and statistics

## 👥 Related Files

- **discover11.py**: Core maze and robot logic
- **maze_data.json**: Generated maze structure data
- **MissionGUI.py**: Previous version (without race optimization)
- **README_GUI.md**: General GUI documentation

## 📧 Support

For issues or questions, refer to the main project documentation or check the related challenge1 files.

---

**Happy Maze Exploring! 🤖🔍**

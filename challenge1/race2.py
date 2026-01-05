"""
Visualize Shortest Path - Standalone Script
This script loads maze data from 'maze_data.json' and animates the robot
moving along the shortest path without needing to re-run the exploration.
"""

import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# Load maze data from JSON file
try:
    with open('maze_data.json', 'r') as f:
        data = json.load(f)
    print("\n✓ Maze data loaded from 'maze_data.json'")
except FileNotFoundError:
    print("\n" + "="*70)
    print("ERROR: maze_data.json not found!")
    print("="*70)
    print("Please run 'maze_explorer.py' first to generate the maze data.")
    print("="*70 + "\n")
    exit()

# Extract data
maze_map = data['maze_map']
start_position = (data['start_position']['row'], data['start_position']['col'])
target_position = (data['target_position']['row'], data['target_position']['col'])
shortest_path = data['shortest_path']
sensed_barriers = data['sensed_barriers']

# Display loaded data
print("\n" + "="*70)
print("SHORTEST PATH VISUALIZATION")
print("="*70)
print(f"Maze Size: {maze_map['size']}x{maze_map['size']}")
print(f"Start Position: {start_position}")
print(f"Target Position: {target_position}")
print(f"Shortest Path: {len(shortest_path)} steps")
print(f"Sensed Barriers: {len(sensed_barriers)} barriers")
print("\nPath Route:")
for cell in shortest_path:
    print(f"  Step {cell['step']}: ({cell['row']}, {cell['col']}) -> {cell['direction']}")
print("="*70 + "\n")

# Create animation
fig, ax = plt.subplots(figsize=(10, 10))

def animate_path(frame):
    """Draw each frame of shortest path animation"""
    ax.clear()
    ax.set_xlim(-10, 160)
    ax.set_ylim(-30, 160)
    ax.invert_yaxis()
    ax.set_aspect('equal')
    
    # Draw cells
    for r in range(5):
        for c in range(5):
            rect = patches.Rectangle((c*30, r*30), 30, 30,
                                    linewidth=0.8, edgecolor='lightgray', 
                                    facecolor='white')
            ax.add_patch(rect)
    
    # Draw barriers
    for barrier in sensed_barriers:
        wall_type = barrier['type']
        r = barrier['row']
        c = barrier['col']
        if wall_type == 'h':
            ax.plot([c*30, (c+1)*30], [r*30, r*30], color='#2E4057', linewidth=4)
        else:
            ax.plot([c*30, c*30], [r*30, (r+1)*30], color='#2E4057', linewidth=4)
    
    # Draw complete path as dotted line
    if len(shortest_path) > 1:
        path_x = [cell['x_cm'] for cell in shortest_path]
        path_y = [cell['y_cm'] for cell in shortest_path]
        ax.plot(path_x, path_y, color='lime', linewidth=3, 
               linestyle='--', alpha=0.4)
    
    if frame < len(shortest_path):
        current_cell = shortest_path[frame]
        current_row = current_cell['row']
        current_col = current_cell['col']
        
        # Draw start
        sr, sc = start_position
        start_rect = patches.Rectangle((sc*30, sr*30), 30, 30,
                                      facecolor='lightgreen', alpha=0.6)
        ax.add_patch(start_rect)
        ax.text(sc*30+15, sr*30+15, 'START', ha='center', va='center',
               fontsize=10, weight='bold', color='darkgreen')
        
        # Draw target
        tr, tc = target_position
        target_rect = patches.Rectangle((tc*30, tr*30), 30, 30,
                                       facecolor='lightcoral', alpha=0.6)
        ax.add_patch(target_rect)
        ax.text(tc*30+15, tr*30+15, 'TARGET', ha='center', va='center',
               fontsize=10, weight='bold', color='darkred')
        
        # Draw traveled path
        for i in range(frame):
            cell = shortest_path[i]
            r, c = cell['row'], cell['col']
            if (r, c) != start_position and (r, c) != target_position:
                circle = patches.Circle((c*30+15, r*30+15), 6,
                                       facecolor='lime', edgecolor='darkgreen', 
                                       linewidth=2)
                ax.add_patch(circle)
        
        # Draw robot
        robot_circle = patches.Circle((current_col*30+15, current_row*30+15), 13,
                                     facecolor='orange', edgecolor='darkorange', 
                                     linewidth=3, zorder=10)
        ax.add_patch(robot_circle)
        ax.text(current_col*30+15, current_row*30+15, 'R', 
               ha='center', va='center', fontsize=16, 
               weight='bold', color='white', zorder=11)
        
        # Draw direction arrow
        if current_cell['direction'] != 'start':
            arrow_len = 12
            direction_map = {
                'up': (0, -arrow_len), 'down': (0, arrow_len),
                'left': (-arrow_len, 0), 'right': (arrow_len, 0)
            }
            if current_cell['direction'] in direction_map:
                dx, dy = direction_map[current_cell['direction']]
                ax.arrow(current_col*30+15, current_row*30+15, dx, dy,
                        head_width=6, head_length=5, 
                        fc='red', ec='red', linewidth=2, alpha=0.8, zorder=9)
        
        # Title
        steps_remaining = len(shortest_path) - 1 - current_cell['step']
        progress = (current_cell['step'] / (len(shortest_path) - 1) * 100) if len(shortest_path) > 1 else 100
        
        title = f"SHORTEST PATH - Step {current_cell['step'] + 1}/{len(shortest_path)}\n"
        title += f"Position: ({current_row}, {current_col}) | "
        title += f"Direction: {current_cell['direction'].upper()} | "
        title += f"Progress: {progress:.0f}% | "
        title += f"Steps Remaining: {steps_remaining}"
        ax.set_title(title, fontsize=12, weight='bold', pad=15)
        
        # Legend
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='orange', markersize=12, 
                      label='Robot', markeredgecolor='darkorange', markeredgewidth=2),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='lime', markersize=10, 
                      label='Path Traveled', markeredgecolor='darkgreen'),
            plt.Line2D([0], [0], color='lime', linewidth=3, 
                      linestyle='--', label='Shortest Route'),
            plt.Line2D([0], [0], color='#2E4057', linewidth=4, 
                      label='Maze Walls')
        ]
        ax.legend(handles=legend_elements, loc='upper left', 
                 bbox_to_anchor=(0, -0.05), ncol=2, frameon=True,
                 fontsize=9, fancybox=True, shadow=True)
        
        # Completion message
        if frame == len(shortest_path) - 1:
            ax.text(75, -20, '🎯 TARGET REACHED! 🎯\nOptimal Path Complete', 
                   ha='center', fontsize=14, weight='bold', color='green',
                   bbox=dict(boxstyle='round,pad=0.8', 
                            facecolor='yellow', edgecolor='green', 
                            linewidth=3, alpha=0.9))
    
    ax.set_xlabel('Distance (cm)', fontsize=11, weight='bold')
    ax.set_ylabel('Distance (cm)', fontsize=11, weight='bold')
    ax.grid(False)

print("Starting animation...")
anim = FuncAnimation(fig, animate_path, 
                    frames=len(shortest_path),
                    interval=500,
                    repeat=True, 
                    repeat_delay=2000)

plt.tight_layout()
plt.show()

print("\n✓ Animation complete!")
print(f"Robot navigated from {start_position} to {target_position}")
print(f"Total steps: {len(shortest_path)}")
print(f"Total distance: {(len(shortest_path) - 1) * 30} cm\n")
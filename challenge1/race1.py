from discover10_2 import maze_map, start_position, target_position, shortest_path, sensed_barriers
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(figsize=(8, 8))

def animate_robot(frame):
    ax.clear()
    ax.set_xlim(0, 150)
    ax.set_ylim(0, 150)
    ax.invert_yaxis()
    ax.axis('off')
    
    # Draw maze walls
    for barrier in sensed_barriers:
        wall_type, r, c = barrier
        if wall_type == 'h':
            ax.plot([c*30, (c+1)*30], [r*30, r*30], 'b-', linewidth=3)
        else:
            ax.plot([c*30, c*30], [r*30, (r+1)*30], 'b-', linewidth=3)
    
    # Draw path up to current frame
    for i in range(min(frame + 1, len(shortest_path))):
        cell = shortest_path[i]
        r, c = cell['row'], cell['col']
        
        if i == frame:  # Current robot position
            circle = patches.Circle((c*30+15, r*30+15), 10, 
                                   facecolor='orange', edgecolor='darkorange', linewidth=2)
            ax.add_patch(circle)
            ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                   fontsize=12, weight='bold', color='white')
        else:  # Previous positions
            circle = patches.Circle((c*30+15, r*30+15), 5, facecolor='lime')
            ax.add_patch(circle)
    
    # Draw start and target
    r, c = start_position
    ax.text(c*30+15, r*30+15, 'S', ha='center', fontsize=12, weight='bold', color='green')
    
    r, c = target_position
    ax.text(c*30+15, r*30+15, 'T', ha='center', fontsize=12, weight='bold', color='red')
    
    if frame < len(shortest_path):
        cell = shortest_path[frame]
        title = f"Step {cell['step']}/{len(shortest_path)-1} | Position: ({cell['row']}, {cell['col']}) | Direction: {cell['direction']}"
        ax.set_title(title, fontsize=11, weight='bold')

anim = FuncAnimation(fig, animate_robot, frames=len(shortest_path), interval=300, repeat=True)
plt.show()
# In your new script:
from discover11 import shortest_path, sensed_barriers, start_position, target_position, maze_map
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Now you can use the variables
print(f"Shortest path: {shortest_path}")
print(f"Start: {start_position}, Target: {target_position}")
print(f"Total barriers sensed: {len(sensed_barriers)}")

# Visualize
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 150)
ax.set_ylim(0, 150)
ax.invert_yaxis()

# Draw barriers in red
for barrier in sensed_barriers:
    wall_type, r, c = barrier
    if wall_type == 'h':
        ax.plot([c*30, (c+1)*30], [r*30, r*30], 'r-', linewidth=3)
    else:
        ax.plot([c*30, c*30], [r*30, (r+1)*30], 'r-', linewidth=3)

# Draw shortest path in green
for r, c in shortest_path:
    circle = patches.Circle((c*30+15, r*30+15), 8, facecolor='lime')
    ax.add_patch(circle)

plt.show()
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import deque
import json 
import pyhula
import time
api = pyhula.UserApi()

class Maze:
    def __init__(self, size=5):
        self.size = size
        self.h_walls = [[True] * size for _ in range(size + 1)]
        self.v_walls = [[True] * (size + 1) for _ in range(size)]
        self.start = None
        self.target = None
        
    def generate(self):
        """Generate random maze"""
        visited = set([(0, 0)])
        stack = [(0, 0)]
        
        while stack:
            r, c = stack[-1]
            neighbors = []
            for d, (dr, dc) in enumerate([(-1,0), (1,0), (0,-1), (0,1)]):
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size and 0 <= nc < self.size and (nr, nc) not in visited:
                    neighbors.append((nr, nc, d))
            
            if neighbors:
                nr, nc, direction = random.choice(neighbors)
                if direction == 0: self.h_walls[r][c] = False
                elif direction == 1: self.h_walls[r+1][c] = False
                elif direction == 2: self.v_walls[r][c] = False
                else: self.v_walls[r][c+1] = False
                visited.add((nr, nc))
                stack.append((nr, nc))
            else:
                stack.pop()
        
        all_cells = [(r, c) for r in range(self.size) for c in range(self.size)]
        self.start = random.choice(all_cells)
        all_cells.remove(self.start)
        self.target = random.choice(all_cells)
    
    def can_move(self, pos, direction):
        """Check if can move in direction"""
        r, c = pos
        if direction == 'up': return r > 0 and not self.h_walls[r][c]
        if direction == 'down': return r < self.size-1 and not self.h_walls[r+1][c]
        if direction == 'left': return c > 0 and not self.v_walls[r][c]
        if direction == 'right': return c < self.size-1 and not self.v_walls[r][c+1]


class Robot:
    def __init__(self, maze):
        self.maze = maze
        self.pos = maze.start
        self.visited = set([self.pos])
        self.path = []
        self.sensed_barriers = set()
        self.shortest_path = []
        
    def sense(self):
        """Sense barriers and record them"""
        r, c = self.pos
        obstacles = api.Plane_getBarrier()
        barriers = set()

        if obstacles["forward"]:
            barriers.add(('h', r, c))
        if obstacles["backward"]:
            barriers.add(('h', r+1, c))
        if obstacles["left"]:
            barriers.add(('v', r, c))
        if obstacles["right"]:
            barriers.add(('v', r, c+1)) 
        # if not self.maze.can_move(self.pos, 'up'):
        #     barriers.add(('h', r, c))
        # if not self.maze.can_move(self.pos, 'down'):
        #     barriers.add(('h', r+1, c))
        # if not self.maze.can_move(self.pos, 'left'):
        #     barriers.add(('v', r, c))
        # if not self.maze.can_move(self.pos, 'right'):
        #     barriers.add(('v', r, c+1))
        
        self.sensed_barriers.update(barriers)
        return barriers
    
    def move(self, direction):
        """Move in direction"""
        r, c = self.pos
        if direction == 'up': 
            self.pos = (r-1, c)
            api.single_fly_forward(30)
        elif direction == 'down': 
            self.pos = (r+1, c)
            api.single_fly_backward(30)
        elif direction == 'left': 
            self.pos = (r, c-1)
            api.single_fly_left(30)
        elif direction == 'right': 
            self.pos = (r, c+1)
            api.single_fly_right(30)
    
    def explore(self):
        """Explore all cells first, then find shortest path"""
        stack = [self.pos]
        current_barriers = self.sense()
        self.path.append((self.pos, len(self.visited), current_barriers))
        
        # Phase 1: Explore ALL cells
        while stack:
            self.pos = stack[-1]
            
            # Get valid moves
            moves = []
            for direction in ['up', 'down', 'left', 'right']:
                if self.maze.can_move(self.pos, direction):
                    r, c = self.pos
                    next_pos = {
                        'up': (r-1, c), 'down': (r+1, c),
                        'left': (r, c-1), 'right': (r, c+1)
                    }[direction]
                    if next_pos not in self.visited:
                        moves.append((next_pos, direction))
            
            if moves:
                next_pos, direction = moves[0]
                self.visited.add(next_pos)
                self.move(direction)
                stack.append(self.pos)
                current_barriers = self.sense()
                self.path.append((self.pos, len(self.visited), current_barriers))
            else:
                stack.pop()
                if stack:
                    target = stack[-1]
                    r1, c1 = self.pos
                    r2, c2 = target
                    if r2 < r1: self.move('up')
                    elif r2 > r1: self.move('down')
                    elif c2 < c1: self.move('left')
                    else: self.move('right')
                    current_barriers = self.sense()
                    self.path.append((self.pos, len(self.visited), current_barriers))
        
        print(f"\n Exploration complete! All {len(self.visited)} cells visited.")
        print(f"  Current position: {self.pos}")
        print(f"  Target position: {self.maze.target}")
        
        # Phase 2: Find shortest path AFTER all cells explored
        print("\n Computing shortest path from current position to target...")
        self._find_shortest_path()
        print(f" Shortest path found: {len(self.shortest_path)} steps")
        
        # Phase 3: Navigate to target using shortest path
        print("\n Moving to target via shortest path...")
        for next_pos in self.shortest_path[1:]:
            r1, c1 = self.pos
            r2, c2 = next_pos
            if r2 < r1: self.move('up')
            elif r2 > r1: self.move('down')
            elif c2 < c1: self.move('left')
            else: self.move('right')
            current_barriers = self.sense()
            self.path.append((self.pos, len(self.visited), current_barriers))
    
    def _find_shortest_path(self):
        """Find shortest path from current pos to target"""
        if self.pos == self.maze.target:
            self.shortest_path = [self.pos]
            return
        
        queue = deque([self.pos])
        parent = {self.pos: None}
        
        while queue:
            current = queue.popleft()
            if current == self.maze.target:
                break
            
            for direction in ['up', 'down', 'left', 'right']:
                if self.maze.can_move(current, direction):
                    r, c = current
                    next_pos = {
                        'up': (r-1, c), 'down': (r+1, c),
                        'left': (r, c-1), 'right': (r, c+1)
                    }[direction]
                    if next_pos not in parent:
                        parent[next_pos] = current
                        queue.append(next_pos)
        
        path = []
        current = self.maze.target
        while current:
            path.append(current)
            current = parent.get(current)
        path.reverse()
        self.shortest_path = path


def save_maze_data(maze, robot):
    """Save maze data to JSON file"""
    
    # Build shortest path with details
    shortest_path_data = []
    for i, (row, col) in enumerate(robot.shortest_path):
        cell_info = {
            'step': i,
            'row': row,
            'col': col,
            'x_cm': col * 30 + 15,
            'y_cm': row * 30 + 15,
        }
        
        if i > 0:
            prev_row, prev_col = robot.shortest_path[i-1]
            if row < prev_row: cell_info['direction'] = 'up'
            elif row > prev_row: cell_info['direction'] = 'down'
            elif col < prev_col: cell_info['direction'] = 'left'
            else: cell_info['direction'] = 'right'
        else:
            cell_info['direction'] = 'start'
        
        shortest_path_data.append(cell_info)
    
    # Convert sensed barriers to list format
    barriers_list = []
    for barrier in robot.sensed_barriers:
        wall_type, r, c = barrier
        barriers_list.append({'type': wall_type, 'row': r, 'col': c})
    
    # Create data structure
    maze_data = {
        'maze_map': {
            'h_walls': maze.h_walls,
            'v_walls': maze.v_walls,
            'size': maze.size
        },
        'start_position': {'row': maze.start[0], 'col': maze.start[1]},
        'target_position': {'row': maze.target[0], 'col': maze.target[1]},
        'shortest_path': shortest_path_data,
        'sensed_barriers': barriers_list
    }
    
    # Save to JSON file
    with open('maze_data.json', 'w') as f:
        json.dump(maze_data, f, indent=2)
    
    print("\n" + "="*70)
    print("✓ Maze data saved to 'maze_data.json'")
    print("="*70)
    print(f"  Maze Size: {maze.size}x{maze.size}")
    print(f"  Start: {maze.start}")
    print(f"  Target: {maze.target}")
    print(f"  Shortest Path: {len(shortest_path_data)} steps")
    print(f"  Sensed Barriers: {len(barriers_list)} barriers")
    print("\nYou can now run 'visualize_shortest_path.py' to see the animation!")
    print("="*70 + "\n")


def visualize(maze, robot):
    """Animate the robot exploration"""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    def draw(frame):
        ax.clear()
        ax.set_xlim(0, 150)
        ax.set_ylim(0, 150)
        ax.invert_yaxis()
        ax.set_aspect('equal')
        ax.axis('off')
        
        # Draw cells
        for r in range(5):
            for c in range(5):
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        linewidth=0.5, edgecolor='lightgray', 
                                        facecolor='white')
                ax.add_patch(rect)
        
        # Get current frame data
        if frame < len(robot.path):
            current_pos, current_count, current_barriers = robot.path[frame]
            
            # Collect all sensed barriers up to this frame
            sensed_so_far = set()
            for i in range(frame + 1):
                _, _, barriers = robot.path[i]
                sensed_so_far.update(barriers)
            
            # Draw walls
            for r in range(6):
                for c in range(5):
                    if maze.h_walls[r][c]:
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
                        ax.plot([c*30, (c+1)*30], [r*30, r*30], color=color, linewidth=width)
            
            for r in range(5):
                for c in range(6):
                    if maze.v_walls[r][c]:
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
                        ax.plot([c*30, c*30], [r*30, (r+1)*30], color=color, linewidth=width)
            
            # Draw visited trail
            for i in range(frame):
                p, _, _ = robot.path[i]
                if p != current_pos and p != maze.start and p != maze.target:
                    r, c = p
                    circle = patches.Circle((c*30+15, r*30+15), 4, facecolor='lightblue')
                    ax.add_patch(circle)
            
            # Draw shortest path
            if robot.shortest_path:
                for p in robot.shortest_path:
                    if p != maze.start and p != maze.target:
                        r, c = p
                        circle = patches.Circle((c*30+15, r*30+15), 6,
                                               facecolor='lime', edgecolor='darkgreen', linewidth=1)
                        ax.add_patch(circle)
            
            # Draw start
            r, c = maze.start
            rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightgreen', alpha=0.6)
            ax.add_patch(rect)
            ax.text(c*30+15, r*30+15, 'S', ha='center', va='center', fontsize=12, weight='bold')
            
            # Draw target
            r, c = maze.target
            rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightcoral', alpha=0.6)
            ax.add_patch(rect)
            ax.text(c*30+15, r*30+15, 'T', ha='center', va='center', fontsize=12, weight='bold')
            
            # Draw robot
            r, c = current_pos
            circle = patches.Circle((c*30+15, r*30+15), 10,
                                   facecolor='orange', edgecolor='darkorange', linewidth=2)
            ax.add_patch(circle)
            ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                   fontsize=12, weight='bold', color='white')
            
            # Title
            path_len = len(robot.shortest_path) if robot.shortest_path else 0
            title = f'Visited: {current_count}/25 | Shortest Path: {path_len} cells\n'
            title += 'Yellow=Sensing | Blue=Sensed | Green=Shortest Path'
            ax.set_title(title, fontsize=11, weight='bold', pad=10)
    
    anim = FuncAnimation(fig, draw, frames=len(robot.path), interval=200, repeat=False)
    plt.tight_layout()
    plt.show()
    return anim


def main():
    maze = Maze(size=5)
    maze.generate()
    
    robot = Robot(maze)
    robot.explore()
    
    # Save maze data to JSON file
    save_maze_data(maze, robot)
    
    # Visualize exploration
    visualize(maze, robot)


if __name__ == "__main__":
    if not api.connect():
        print("connect error")
    else:
    api.single_fly_takeoff()
    time.sleep(5)
    main()
    api.singly_fly_touchown()

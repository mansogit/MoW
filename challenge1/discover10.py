import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import deque

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
        
    def sense_and_record(self):
        """Sense barriers and add to sensed list"""
        r, c = self.pos
        barriers = []
        
        if not self.maze.can_move(self.pos, 'up'):
            barriers.append(('h', r, c))
        if not self.maze.can_move(self.pos, 'down'):
            barriers.append(('h', r+1, c))
        if not self.maze.can_move(self.pos, 'left'):
            barriers.append(('v', r, c))
        if not self.maze.can_move(self.pos, 'right'):
            barriers.append(('v', r, c+1))
        
        return barriers
    
    def move(self, direction):
        """Move in direction"""
        r, c = self.pos
        if direction == 'up': self.pos = (r-1, c)
        elif direction == 'down': self.pos = (r+1, c)
        elif direction == 'left': self.pos = (r, c-1)
        elif direction == 'right': self.pos = (r, c+1)
    
    def explore(self):
        """Explore all cells"""
        stack = [self.pos]
        
        # Record initial position
        barriers = self.sense_and_record()
        self.path.append((self.pos, len(self.visited), barriers))
        
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
                barriers = self.sense_and_record()
                self.path.append((self.pos, len(self.visited), barriers))
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
                    barriers = self.sense_and_record()
                    self.path.append((self.pos, len(self.visited), barriers))
        
        # Find shortest path to target
        self._find_shortest_path()
        
        # Navigate to target
        for next_pos in self.shortest_path[1:]:
            r1, c1 = self.pos
            r2, c2 = next_pos
            if r2 < r1: self.move('up')
            elif r2 > r1: self.move('down')
            elif c2 < c1: self.move('left')
            else: self.move('right')
            barriers = self.sense_and_record()
            self.path.append((self.pos, len(self.visited), barriers))
    
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


def visualize(maze, robot):
    """Create animation"""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    def draw(frame):
        ax.clear()
        ax.set_xlim(0, 150)
        ax.set_ylim(0, 150)
        ax.invert_yaxis()
        ax.set_aspect('equal')
        
        # Draw cells
        for r in range(5):
            for c in range(5):
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        linewidth=0.5, edgecolor='gray', 
                                        facecolor='white')
                ax.add_patch(rect)
        
        # Collect sensed barriers up to this frame
        sensed_so_far = set()
        current_barriers = set()
        
        if frame < len(robot.path):
            for i in range(frame + 1):
                pos, count, barriers = robot.path[i]
                sensed_so_far.update(barriers)
                if i == frame:
                    current_barriers = set(barriers)
        
        # Draw walls with colors
        for r in range(6):
            for c in range(5):
                if maze.h_walls[r][c]:
                    wall = ('h', r, c)
                    if wall in current_barriers:
                        color = 'yellow'
                    elif wall in sensed_so_far:
                        color = 'blue'
                    else:
                        color = 'black'
                    ax.plot([c*30, (c+1)*30], [r*30, r*30], color=color, linewidth=3)
        
        for r in range(5):
            for c in range(6):
                if maze.v_walls[r][c]:
                    wall = ('v', r, c)
                    if wall in current_barriers:
                        color = 'yellow'
                    elif wall in sensed_so_far:
                        color = 'blue'
                    else:
                        color = 'black'
                    ax.plot([c*30, c*30], [r*30, (r+1)*30], color=color, linewidth=3)
        
        if frame < len(robot.path):
            pos, count, _ = robot.path[frame]
            
            # Draw visited trail
            for i in range(frame):
                p, _, _ = robot.path[i]
                if p != pos and p != maze.start and p != maze.target:
                    r, c = p
                    circle = patches.Circle((c*30+15, r*30+15), 5, facecolor='lightblue')
                    ax.add_patch(circle)
            
            # Draw shortest path
            for p in robot.shortest_path:
                if p != maze.start and p != maze.target:
                    r, c = p
                    circle = patches.Circle((c*30+15, r*30+15), 7,
                                           facecolor='lime', edgecolor='darkgreen', linewidth=2)
                    ax.add_patch(circle)
            
            # Draw start
            r, c = maze.start
            rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightgreen', alpha=0.5)
            ax.add_patch(rect)
            ax.text(c*30+15, r*30+15, 'S', ha='center', fontsize=10, weight='bold')
            
            # Draw target
            r, c = maze.target
            rect = patches.Rectangle((c*30, r*30), 30, 30, facecolor='lightcoral', alpha=0.5)
            ax.add_patch(rect)
            ax.text(c*30+15, r*30+15, 'T', ha='center', fontsize=10, weight='bold')
            
            # Draw robot
            r, c = pos
            circle = patches.Circle((c*30+15, r*30+15), 10,
                                   facecolor='orange', edgecolor='darkorange', linewidth=2)
            ax.add_patch(circle)
            ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                   fontsize=12, weight='bold', color='white')
            
            title = f'Visited: {count}/25 | Shortest Path: {len(robot.shortest_path)} cells\n'
            title += 'Yellow=Sensing Now | Blue=Sensed | Green=Shortest Path'
            ax.set_title(title, fontsize=10, weight='bold')
    
    anim = FuncAnimation(fig, draw, frames=len(robot.path), interval=200, repeat=False)
    plt.tight_layout()
    plt.show()


def main():
    maze = Maze(size=5)
    maze.generate()
    
    robot = Robot(maze)
    robot.explore()
    
    visualize(maze, robot)

if __name__ == "__main__":
    main()
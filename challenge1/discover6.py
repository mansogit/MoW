import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import deque

class Maze:
    def __init__(self, size=5, cell_size=30):
        self.size = size
        self.cell_size = cell_size
        self.h_walls = [[True] * size for _ in range(size + 1)]
        self.v_walls = [[True] * (size + 1) for _ in range(size)]
        self.start = None
        self.target = None
        
    def generate(self):
        """Generate maze using recursive backtracking"""
        visited = set()
        stack = [(0, 0)]
        visited.add((0, 0))
        
        while stack:
            r, c = stack[-1]
            neighbors = self._get_unvisited_neighbors(r, c, visited)
            
            if neighbors:
                nr, nc, direction = random.choice(neighbors)
                self._remove_wall(r, c, direction)
                visited.add((nr, nc))
                stack.append((nr, nc))
            else:
                stack.pop()
        
        self._set_start_and_target()
    
    def _get_unvisited_neighbors(self, r, c, visited):
        """Get neighboring cells that haven't been visited"""
        neighbors = []
        directions = [(-1, 0, 0), (1, 0, 1), (0, -1, 2), (0, 1, 3)]  # up, down, left, right
        
        for dr, dc, direction in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.size and 0 <= nc < self.size and (nr, nc) not in visited:
                neighbors.append((nr, nc, direction))
        return neighbors
    
    def _remove_wall(self, r, c, direction):
        """Remove wall in specified direction"""
        if direction == 0:    # up
            self.h_walls[r][c] = False
        elif direction == 1:  # down
            self.h_walls[r + 1][c] = False
        elif direction == 2:  # left
            self.v_walls[r][c] = False
        else:                 # right
            self.v_walls[r][c + 1] = False
    
    def _set_start_and_target(self):
        """Randomly place start and target"""
        cells = [(r, c) for r in range(self.size) for c in range(self.size)]
        self.start = random.choice(cells)
        cells.remove(self.start)
        self.target = random.choice(cells)
    
    def get_neighbors(self, pos):
        """Get accessible neighbors from a position"""
        r, c = pos
        neighbors = []
        
        if r > 0 and not self.h_walls[r][c]:
            neighbors.append((r - 1, c))
        if r < self.size - 1 and not self.h_walls[r + 1][c]:
            neighbors.append((r + 1, c))
        if c > 0 and not self.v_walls[r][c]:
            neighbors.append((r, c - 1))
        if c < self.size - 1 and not self.v_walls[r][c + 1]:
            neighbors.append((r, c + 1))
        
        return neighbors


class Robot:
    def __init__(self, maze):
        self.maze = maze
        self.position = maze.start
        self.visited = set([self.position])
        self.path = [(self.position, 1)]
    
    def explore_all(self):
        """Explore all reachable cells using DFS"""
        stack = [self.position]
        
        while stack:
            current = stack[-1]
            self.position = current
            
            unvisited = [n for n in self.maze.get_neighbors(current) 
                        if n not in self.visited]
            
            if unvisited:
                next_cell = unvisited[0]
                self.visited.add(next_cell)
                stack.append(next_cell)
                self.position = next_cell
                self.path.append((self.position, len(self.visited)))
            else:
                stack.pop()
                if stack:
                    self.position = stack[-1]
                    self.path.append((self.position, len(self.visited)))
    
    def go_to_target(self):
        """Navigate to target using BFS"""
        if self.position == self.maze.target:
            return
        
        queue = deque([self.position])
        parent = {self.position: None}
        
        while queue:
            current = queue.popleft()
            if current == self.maze.target:
                break
            
            for neighbor in self.maze.get_neighbors(current):
                if neighbor not in parent:
                    parent[neighbor] = current
                    queue.append(neighbor)
        
        # Reconstruct path
        path_to_target = []
        current = self.maze.target
        while current:
            path_to_target.append(current)
            current = parent[current]
        path_to_target.reverse()
        
        # Add to exploration path
        for pos in path_to_target[1:]:
            self.position = pos
            self.path.append((self.position, len(self.visited)))


class Visualizer:
    def __init__(self, maze, robot):
        self.maze = maze
        self.robot = robot
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
    
    def animate(self):
        """Create and show animation"""
        anim = FuncAnimation(self.fig, self._draw_frame, 
                           frames=len(self.robot.path),
                           interval=200, repeat=False)
        plt.tight_layout()
        plt.show()
    
    def _draw_frame(self, frame_num):
        """Draw a single frame"""
        self.ax.clear()
        self._setup_axes()
        self._draw_cells()
        self._draw_walls()
        
        if frame_num < len(self.robot.path):
            current_pos, cells_visited = self.robot.path[frame_num]
            self._draw_visited_trail(frame_num)
            self._draw_start_and_target()
            self._draw_robot(current_pos)
            self._draw_title(cells_visited, frame_num)
    
    def _setup_axes(self):
        """Configure axes"""
        limit = self.maze.size * self.maze.cell_size
        self.ax.set_xlim(0, limit)
        self.ax.set_ylim(0, limit)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        self.ax.set_xlabel('Distance (cm)', fontsize=10)
        self.ax.set_ylabel('Distance (cm)', fontsize=10)
        self.ax.grid(False)
    
    def _draw_cells(self):
        """Draw all cells"""
        for r in range(self.maze.size):
            for c in range(self.maze.size):
                rect = patches.Rectangle(
                    (c * self.maze.cell_size, r * self.maze.cell_size),
                    self.maze.cell_size, self.maze.cell_size,
                    linewidth=0.5, edgecolor='lightgray', facecolor='white')
                self.ax.add_patch(rect)
    
    def _draw_walls(self):
        """Draw maze walls"""
        cs = self.maze.cell_size
        
        # Horizontal walls
        for r in range(self.maze.size + 1):
            for c in range(self.maze.size):
                if self.maze.h_walls[r][c]:
                    self.ax.plot([c * cs, (c + 1) * cs], [r * cs, r * cs], 
                                'k-', linewidth=2)
        
        # Vertical walls
        for r in range(self.maze.size):
            for c in range(self.maze.size + 1):
                if self.maze.v_walls[r][c]:
                    self.ax.plot([c * cs, c * cs], [r * cs, (r + 1) * cs], 
                                'k-', linewidth=2)
    
    def _draw_visited_trail(self, frame_num):
        """Draw visited cells as dots"""
        for i in range(frame_num + 1):
            pos, _ = self.robot.path[i]
            if pos != self.robot.path[frame_num][0] and pos != self.maze.start and pos != self.maze.target:
                r, c = pos
                x = (c + 0.5) * self.maze.cell_size
                y = (r + 0.5) * self.maze.cell_size
                circle = patches.Circle((x, y), self.maze.cell_size / 6,
                                       facecolor='lightblue', edgecolor='blue')
                self.ax.add_patch(circle)
    
    def _draw_start_and_target(self):
        """Draw start and target cells"""
        cs = self.maze.cell_size
        
        # Start
        r, c = self.maze.start
        rect = patches.Rectangle((c * cs, r * cs), cs, cs,
                                facecolor='lightgreen', alpha=0.5)
        self.ax.add_patch(rect)
        self.ax.text((c + 0.5) * cs, (r + 0.5) * cs, 'START',
                    ha='center', va='center', fontsize=9, weight='bold')
        
        # Target
        r, c = self.maze.target
        rect = patches.Rectangle((c * cs, r * cs), cs, cs,
                                facecolor='lightcoral', alpha=0.5)
        self.ax.add_patch(rect)
        self.ax.text((c + 0.5) * cs, (r + 0.5) * cs, 'TARGET',
                    ha='center', va='center', fontsize=9, weight='bold')
    
    def _draw_robot(self, pos):
        """Draw robot at current position"""
        r, c = pos
        x = (c + 0.5) * self.maze.cell_size
        y = (r + 0.5) * self.maze.cell_size
        circle = patches.Circle((x, y), self.maze.cell_size / 3,
                               facecolor='orange', edgecolor='darkorange', linewidth=3)
        self.ax.add_patch(circle)
        self.ax.text(x, y, 'R', ha='center', va='center',
                    fontsize=14, weight='bold', color='white')
    
    def _draw_title(self, cells_visited, frame_num):
        """Draw title and completion message"""
        total = self.maze.size * self.maze.size
        self.ax.set_title(
            f'Maze Explorer | Cells Visited: {cells_visited}/{total} | Cell Size: {self.maze.cell_size}cm x {self.maze.cell_size}cm',
            fontsize=12, weight='bold')
        
        if frame_num == len(self.robot.path) - 1:
            self.ax.text(self.maze.size * self.maze.cell_size / 2, -10,
                        'EXPLORATION COMPLETE!',
                        ha='center', fontsize=14, weight='bold', color='green')


def main():
    # Create and generate maze
    maze = Maze(size=5, cell_size=30)
    maze.generate()
    
    # Create robot and explore
    robot = Robot(maze)
    robot.explore_all()
    robot.go_to_target()
    
    # Visualize
    viz = Visualizer(maze, robot)
    viz.animate()


if __name__ == "__main__":
    main()
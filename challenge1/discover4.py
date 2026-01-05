import random
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class Maze:
    def __init__(self, size=5):
        self.size = size
        self.grid = [[1 for _ in range(size)] for _ in range(size)]
        self.start = None
        self.target = None
        self.robot_pos = None
        self.visited = set()
        self.exploration_path = []
        
    def generate_maze(self):
        """Generate a simple random maze using recursive backtracking"""
        stack = []
        current = (0, 0)
        self.grid[0][0] = 0
        stack.append(current)
        
        while stack:
            row, col = current
            neighbors = []
            
            # Check all four directions
            for dr, dc in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
                new_row, new_col = row + dr, col + dc
                if (0 <= new_row < self.size and 0 <= new_col < self.size and 
                    self.grid[new_row][new_col] == 1):
                    neighbors.append((new_row, new_col, dr//2, dc//2))
            
            if neighbors:
                new_row, new_col, wall_row, wall_col = random.choice(neighbors)
                self.grid[new_row][new_col] = 0
                self.grid[row + wall_row][col + wall_col] = 0
                stack.append((new_row, new_col))
                current = (new_row, new_col)
            else:
                if stack:
                    current = stack.pop()
        
        # Set random start and target positions on open cells
        open_cells = [(r, c) for r in range(self.size) for c in range(self.size) 
                      if self.grid[r][c] == 0]
        self.start = random.choice(open_cells)
        open_cells.remove(self.start)
        self.target = random.choice(open_cells)
        self.robot_pos = self.start
        
    def get_neighbors(self, pos):
        """Get valid neighboring cells"""
        row, col = pos
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_row, new_col = row + dr, col + dc
            if (0 <= new_row < self.size and 0 <= new_col < self.size and 
                self.grid[new_row][new_col] == 0):
                neighbors.append((new_row, new_col))
        return neighbors
    
    def explore_all_then_target(self):
        """DFS to explore all cells before reaching target"""
        stack = [self.robot_pos]
        self.visited.add(self.robot_pos)
        self.exploration_path.append((self.robot_pos, len(self.visited)))
        parent = {self.robot_pos: None}
        
        # First, explore all reachable cells
        while stack:
            current = stack[-1]
            self.robot_pos = current
            
            # Find unvisited neighbors
            unvisited = [n for n in self.get_neighbors(current) if n not in self.visited]
            
            if unvisited:
                next_cell = unvisited[0]
                self.visited.add(next_cell)
                parent[next_cell] = current
                stack.append(next_cell)
                self.robot_pos = next_cell
                self.exploration_path.append((self.robot_pos, len(self.visited)))
            else:
                stack.pop()
                if stack:
                    self.robot_pos = stack[-1]
                    self.exploration_path.append((self.robot_pos, len(self.visited)))
        
        # Now navigate to target if not already there
        if self.robot_pos != self.target:
            # Build path to target using BFS
            from collections import deque
            queue = deque([self.robot_pos])
            parent_to_target = {self.robot_pos: None}
            found = False
            
            while queue and not found:
                current = queue.popleft()
                if current == self.target:
                    found = True
                    break
                    
                for neighbor in self.get_neighbors(current):
                    if neighbor not in parent_to_target:
                        parent_to_target[neighbor] = current
                        queue.append(neighbor)
            
            # Reconstruct and follow path to target
            if found:
                path = []
                current = self.target
                while current is not None:
                    path.append(current)
                    current = parent_to_target[current]
                path.reverse()
                
                for pos in path[1:]:
                    self.robot_pos = pos
                    self.exploration_path.append((self.robot_pos, len(self.visited)))
    
    def visualize_animated(self):
        """Create animated visualization using matplotlib"""
        fig, ax = plt.subplots(figsize=(8, 8))
        cell_size = 30  # 30cm as specified
        
        def draw_frame(frame_num):
            ax.clear()
            ax.set_xlim(0, self.size * cell_size)
            ax.set_ylim(0, self.size * cell_size)
            ax.set_aspect('equal')
            ax.invert_yaxis()
            
            # Draw grid and walls
            for r in range(self.size):
                for c in range(self.size):
                    x = c * cell_size
                    y = r * cell_size
                    
                    if self.grid[r][c] == 1:  # Wall
                        rect = patches.Rectangle((x, y), cell_size, cell_size, 
                                                linewidth=1, edgecolor='black', 
                                                facecolor='#333333')
                        ax.add_patch(rect)
                    else:  # Open cell
                        rect = patches.Rectangle((x, y), cell_size, cell_size, 
                                                linewidth=1, edgecolor='gray', 
                                                facecolor='white')
                        ax.add_patch(rect)
            
            # Draw visited cells up to current frame
            if frame_num < len(self.exploration_path):
                current_pos, cells_visited = self.exploration_path[frame_num]
                
                for i in range(min(frame_num + 1, len(self.exploration_path))):
                    pos, _ = self.exploration_path[i]
                    r, c = pos
                    x = c * cell_size + cell_size / 2
                    y = r * cell_size + cell_size / 2
                    
                    if pos != current_pos and pos != self.start and pos != self.target:
                        circle = patches.Circle((x, y), cell_size / 6, 
                                               facecolor='lightblue', edgecolor='blue')
                        ax.add_patch(circle)
                
                # Draw start cell
                r, c = self.start
                rect = patches.Rectangle((c * cell_size, r * cell_size), 
                                        cell_size, cell_size, 
                                        linewidth=2, edgecolor='green', 
                                        facecolor='lightgreen', alpha=0.5)
                ax.add_patch(rect)
                ax.text((c + 0.5) * cell_size, (r + 0.5) * cell_size, 'START',
                       ha='center', va='center', fontsize=10, weight='bold')
                
                # Draw target cell
                r, c = self.target
                rect = patches.Rectangle((c * cell_size, r * cell_size), 
                                        cell_size, cell_size, 
                                        linewidth=2, edgecolor='red', 
                                        facecolor='lightcoral', alpha=0.5)
                ax.add_patch(rect)
                ax.text((c + 0.5) * cell_size, (r + 0.5) * cell_size, 'TARGET',
                       ha='center', va='center', fontsize=10, weight='bold')
                
                # Draw robot
                r, c = current_pos
                x = c * cell_size + cell_size / 2
                y = r * cell_size + cell_size / 2
                circle = patches.Circle((x, y), cell_size / 3, 
                                       facecolor='orange', edgecolor='darkorange', 
                                       linewidth=3)
                ax.add_patch(circle)
                ax.text(x, y, 'R', ha='center', va='center', 
                       fontsize=14, weight='bold', color='white')
                
                # Title with stats
                ax.set_title(f'Maze Explorer | Cells Visited: {cells_visited}/{self.size * self.size} | Cell Size: 30cm x 30cm',
                           fontsize=12, weight='bold')
                
                if frame_num == len(self.exploration_path) - 1:
                    ax.text(self.size * cell_size / 2, -10, 
                           'EXPLORATION COMPLETE!', 
                           ha='center', fontsize=14, weight='bold', color='green')
            
            ax.set_xlabel('Distance (cm)', fontsize=10)
            ax.set_ylabel('Distance (cm)', fontsize=10)
            ax.grid(True, alpha=0.3)
        
        anim = FuncAnimation(fig, draw_frame, frames=len(self.exploration_path),
                           interval=200, repeat=False)
        plt.tight_layout()
        plt.show()

def main():
    maze = Maze(5)
    maze.generate_maze()
    maze.explore_all_then_target()
    maze.visualize_animated()

if __name__ == "__main__":
    main()
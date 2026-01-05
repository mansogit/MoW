import random
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class Maze:
    def __init__(self, size=5):
        self.size = size
        # Create walls between cells (horizontal and vertical)
        self.h_walls = [[True for _ in range(size)] for _ in range(size + 1)]
        self.v_walls = [[True for _ in range(size + 1)] for _ in range(size)]
        self.start = None
        self.target = None
        self.robot_pos = None
        self.visited = set()
        self.exploration_path = []
        
    def generate_maze(self):
        """Generate maze by removing walls using recursive backtracking"""
        visited_cells = set()
        stack = []
        current = (0, 0)
        visited_cells.add(current)
        stack.append(current)
        
        while stack:
            row, col = current
            neighbors = []
            
            # Check all four directions
            for direction, (dr, dc) in enumerate([(-1, 0), (1, 0), (0, -1), (0, 1)]):
                new_row, new_col = row + dr, col + dc
                if (0 <= new_row < self.size and 0 <= new_col < self.size and 
                    (new_row, new_col) not in visited_cells):
                    neighbors.append((new_row, new_col, direction))
            
            if neighbors:
                new_row, new_col, direction = random.choice(neighbors)
                
                # Remove wall between current and new cell
                if direction == 0:  # Up
                    self.h_walls[row][col] = False
                elif direction == 1:  # Down
                    self.h_walls[row + 1][col] = False
                elif direction == 2:  # Left
                    self.v_walls[row][col] = False
                else:  # Right
                    self.v_walls[row][col + 1] = False
                
                visited_cells.add((new_row, new_col))
                stack.append((new_row, new_col))
                current = (new_row, new_col)
            else:
                if stack:
                    stack.pop()
                    if stack:
                        current = stack[-1]
        
        # Set random start and target positions
        all_cells = [(r, c) for r in range(self.size) for c in range(self.size)]
        self.start = random.choice(all_cells)
        all_cells.remove(self.start)
        self.target = random.choice(all_cells)
        self.robot_pos = self.start
        
    def get_neighbors(self, pos):
        """Get valid neighboring cells (not blocked by walls)"""
        row, col = pos
        neighbors = []
        
        # Check up
        if row > 0 and not self.h_walls[row][col]:
            neighbors.append((row - 1, col))
        # Check down
        if row < self.size - 1 and not self.h_walls[row + 1][col]:
            neighbors.append((row + 1, col))
        # Check left
        if col > 0 and not self.v_walls[row][col]:
            neighbors.append((row, col - 1))
        # Check right
        if col < self.size - 1 and not self.v_walls[row][col + 1]:
            neighbors.append((row, col + 1))
        
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
        wall_thickness = 2
        
        def draw_frame(frame_num):
            ax.clear()
            ax.set_xlim(0, self.size * cell_size)
            ax.set_ylim(0, self.size * cell_size)
            ax.set_aspect('equal')
            ax.invert_yaxis()
            
            # Draw all cells as white
            for r in range(self.size):
                for c in range(self.size):
                    x = c * cell_size
                    y = r * cell_size
                    rect = patches.Rectangle((x, y), cell_size, cell_size, 
                                            linewidth=0.5, edgecolor='lightgray', 
                                            facecolor='white')
                    ax.add_patch(rect)
            
            # Draw walls between cells
            # Horizontal walls
            for r in range(self.size + 1):
                for c in range(self.size):
                    if self.h_walls[r][c]:
                        x1 = c * cell_size
                        x2 = (c + 1) * cell_size
                        y = r * cell_size
                        ax.plot([x1, x2], [y, y], 'k-', linewidth=wall_thickness)
            
            # Vertical walls
            for r in range(self.size):
                for c in range(self.size + 1):
                    if self.v_walls[r][c]:
                        x = c * cell_size
                        y1 = r * cell_size
                        y2 = (r + 1) * cell_size
                        ax.plot([x, x], [y1, y2], 'k-', linewidth=wall_thickness)
            
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
                                        linewidth=0, edgecolor='none', 
                                        facecolor='lightgreen', alpha=0.5)
                ax.add_patch(rect)
                ax.text((c + 0.5) * cell_size, (r + 0.5) * cell_size, 'START',
                       ha='center', va='center', fontsize=9, weight='bold')
                
                # Draw target cell
                r, c = self.target
                rect = patches.Rectangle((c * cell_size, r * cell_size), 
                                        cell_size, cell_size, 
                                        linewidth=0, edgecolor='none', 
                                        facecolor='lightcoral', alpha=0.5)
                ax.add_patch(rect)
                ax.text((c + 0.5) * cell_size, (r + 0.5) * cell_size, 'TARGET',
                       ha='center', va='center', fontsize=9, weight='bold')
                
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
            ax.grid(False)
        
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
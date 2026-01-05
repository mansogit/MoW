import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

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
            neighbors = [(r+dr, c+dc, d) for d, (dr, dc) in 
                        enumerate([(-1,0), (1,0), (0,-1), (0,1)])
                        if 0 <= r+dr < self.size and 0 <= c+dc < self.size 
                        and (r+dr, c+dc) not in visited]
            
            if neighbors:
                nr, nc, direction = random.choice(neighbors)
                self._remove_wall(r, c, direction)
                visited.add((nr, nc))
                stack.append((nr, nc))
            else:
                stack.pop()
        
        all_cells = [(r, c) for r in range(self.size) for c in range(self.size)]
        self.start = random.choice(all_cells)
        all_cells.remove(self.start)
        self.target = random.choice(all_cells)
    
    def _remove_wall(self, r, c, direction):
        if direction == 0: self.h_walls[r][c] = False
        elif direction == 1: self.h_walls[r+1][c] = False
        elif direction == 2: self.v_walls[r][c] = False
        else: self.v_walls[r][c+1] = False
    
    def can_move(self, pos, direction):
        """Check if robot can move in direction"""
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
        self.path = [(self.pos, 1)]
        self.sensed_walls = set()  # Track which walls have been sensed
        self.current_sensed = set()  # Walls currently being sensed
    
    def sense(self):
        """Sense walls around current position and record them"""
        r, c = self.pos
        sensors = {}
        self.current_sensed = set()  # Clear current sensing
        
        # Check each direction and record sensed walls
        if not self.maze.can_move(self.pos, 'up'):
            sensors['up'] = False
            wall = ('h', r, c)
            self.sensed_walls.add(wall)
            self.current_sensed.add(wall)
        else:
            sensors['up'] = True
            
        if not self.maze.can_move(self.pos, 'down'):
            sensors['down'] = False
            wall = ('h', r+1, c)
            self.sensed_walls.add(wall)
            self.current_sensed.add(wall)
        else:
            sensors['down'] = True
            
        if not self.maze.can_move(self.pos, 'left'):
            sensors['left'] = False
            wall = ('v', r, c)
            self.sensed_walls.add(wall)
            self.current_sensed.add(wall)
        else:
            sensors['left'] = True
            
        if not self.maze.can_move(self.pos, 'right'):
            sensors['right'] = False
            wall = ('v', r, c+1)
            self.sensed_walls.add(wall)
            self.current_sensed.add(wall)
        else:
            sensors['right'] = True
        
        return sensors
    
    def move(self, direction):
        """Move in direction"""
        r, c = self.pos
        if direction == 'up': self.pos = (r-1, c)
        elif direction == 'down': self.pos = (r+1, c)
        elif direction == 'left': self.pos = (r, c-1)
        elif direction == 'right': self.pos = (r, c+1)
    
    def explore(self):
        """Explore all cells using sensors, then go to target"""
        stack = [self.pos]
        
        # Explore all cells
        while stack:
            self.pos = stack[-1]
            sensors = self.sense()
            
            # Find unvisited neighbors
            moves = []
            for direction, can_go in sensors.items():
                if can_go:
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
                self.path.append((self.pos, len(self.visited)))
            else:
                stack.pop()
                if stack:
                    # Move back
                    target = stack[-1]
                    r1, c1 = self.pos
                    r2, c2 = target
                    if r2 < r1: self.move('up')
                    elif r2 > r1: self.move('down')
                    elif c2 < c1: self.move('left')
                    else: self.move('right')
                    self.path.append((self.pos, len(self.visited)))
        
        # Now navigate to target
        self._go_to_target()
    
    def _go_to_target(self):
        """Navigate to target cell using BFS"""
        if self.pos == self.maze.target:
            return
        
        from collections import deque
        queue = deque([self.pos])
        parent = {self.pos: None}
        
        # BFS to find path
        while queue:
            current = queue.popleft()
            if current == self.maze.target:
                break
            
            sensors = {d: self.maze.can_move(current, d) 
                      for d in ['up', 'down', 'left', 'right']}
            
            for direction, can_go in sensors.items():
                if can_go:
                    r, c = current
                    next_pos = {
                        'up': (r-1, c), 'down': (r+1, c),
                        'left': (r, c-1), 'right': (r, c+1)
                    }[direction]
                    if next_pos not in parent:
                        parent[next_pos] = current
                        queue.append(next_pos)
        
        # Build path to target
        path_to_target = []
        current = self.maze.target
        while current:
            path_to_target.append(current)
            current = parent.get(current)
        path_to_target.reverse()
        
        # Move along path
        for next_pos in path_to_target[1:]:
            r1, c1 = self.pos
            r2, c2 = next_pos
            if r2 < r1: self.move('up')
            elif r2 > r1: self.move('down')
            elif c2 < c1: self.move('left')
            else: self.move('right')
            self.path.append((self.pos, len(self.visited)))


class Visualizer:
    def __init__(self, maze, robot):
        self.maze = maze
        self.robot = robot
        
    def show(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        
        def draw(frame):
            ax.clear()
            ax.set_xlim(0, 150)
            ax.set_ylim(0, 150)
            ax.invert_yaxis()
            ax.set_aspect('equal')
            
            # Draw cells
            for r in range(self.maze.size):
                for c in range(self.maze.size):
                    rect = patches.Rectangle((c*30, r*30), 30, 30,
                                            linewidth=0.5, edgecolor='gray', 
                                            facecolor='white')
                    ax.add_patch(rect)
            
            # Draw walls
            for r in range(6):
                for c in range(5):
                    if self.maze.h_walls[r][c]:
                        wall = ('h', r, c)
                        # Yellow if currently sensing, red if previously sensed, black otherwise
                        if wall in self.robot.current_sensed:
                            color = 'yellow'
                        elif wall in self.robot.sensed_walls:
                            color = 'red'
                        else:
                            color = 'black'
                        ax.plot([c*30, c*30+30], [r*30, r*30], color=color, linewidth=3)
            for r in range(5):
                for c in range(6):
                    if self.maze.v_walls[r][c]:
                        wall = ('v', r, c)
                        # Yellow if currently sensing, red if previously sensed, black otherwise
                        if wall in self.robot.current_sensed:
                            color = 'yellow'
                        elif wall in self.robot.sensed_walls:
                            color = 'red'
                        else:
                            color = 'black'
                        ax.plot([c*30, c*30], [r*30, r*30+30], color=color, linewidth=3)
            
            if frame < len(self.robot.path):
                pos, count = self.robot.path[frame]
                
                # Update current sensed walls for this frame
                temp_pos = self.robot.pos
                self.robot.pos = pos
                self.robot.sense()  # Update current_sensed
                self.robot.pos = temp_pos
                
                # Draw visited
                for i in range(frame):
                    p, _ = self.robot.path[i]
                    if p != pos and p != self.maze.start and p != self.maze.target:
                        r, c = p
                        circle = patches.Circle((c*30+15, r*30+15), 5,
                                               facecolor='lightblue')
                        ax.add_patch(circle)
                
                # Draw start/target
                r, c = self.maze.start
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        facecolor='lightgreen', alpha=0.5)
                ax.add_patch(rect)
                ax.text(c*30+15, r*30+15, 'START', ha='center', fontsize=8, weight='bold')
                
                r, c = self.maze.target
                rect = patches.Rectangle((c*30, r*30), 30, 30,
                                        facecolor='lightcoral', alpha=0.5)
                ax.add_patch(rect)
                ax.text(c*30+15, r*30+15, 'TARGET', ha='center', fontsize=8, weight='bold')
                
                # Draw robot
                r, c = pos
                circle = patches.Circle((c*30+15, r*30+15), 10,
                                       facecolor='orange', edgecolor='darkorange', 
                                       linewidth=2)
                ax.add_patch(circle)
                ax.text(c*30+15, r*30+15, 'R', ha='center', va='center',
                       fontsize=12, weight='bold', color='white')
                
                ax.set_title(f'Cells Visited: {count}/25 | Cell Size: 30cm x 30cm\nYellow=Currently Sensing | Red=Previously Sensed | Black=Not Sensed',
                           fontsize=11, weight='bold')
        
        anim = FuncAnimation(fig, draw, frames=len(self.robot.path),
                           interval=200, repeat=False)
        plt.tight_layout()
        plt.show()


def main():
    maze = Maze(size=5)
    maze.generate()
    
    robot = Robot(maze)
    robot.explore()
    
    viz = Visualizer(maze, robot)
    viz.show()

if __name__ == "__main__":
    main()
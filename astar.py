import numpy as np
from utils import dist, CellType

class AStar:
    def __init__(self, map, start, goal):
        self.start = tuple(start)
        self.goal = tuple(goal)
        
        self.map = map
        self.height, self.width = self.map.shape
        
        self.g_scores = np.full((self.height, self.width), np.inf)
        self.g_scores[self.start] = 0
        
        y_coords, x_coords = np.indices((self.height, self.width))
        self.h_scores = np.abs(y_coords - goal[0]) + np.abs(x_coords - goal[1])
        
        self.f_scores = self.g_scores + self.h_scores        
        
        self.parents = np.full((self.height, self.width, 2), -1, dtype=int)

        self.open_set = [self.start]
        self.closed_set = np.zeros((self.height, self.width), dtype=bool)

    def h(self, p):
        return dist(p, self.goal, 'manhattam')
    
    def g(self, p1, p2):
        return dist(p1, p2, 'manhattam')
    
    def get_valid_neighbors(self, p):
        x, y = p
        # Generate potential 4-connectivity neighbors
        candidates = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        
        # Filter using a list comprehension and convert back to a list or array
        valid_neighbors = [
            neighbor for neighbor in candidates 
            if self.is_cell_valid(neighbor)
        ]
        
        return valid_neighbors

    def is_cell_valid(self, p) -> bool:
        x, y = p
        # Corrected boundary checks (using >= width/height is out of bounds)
        if x >= self.width or x < 0:
            return False
        
        if y >= self.height or y < 0:
            return False
        
        # Accessing map with (y, x) or (x, y) depends on your map's orientation
        if self.map[y, x] == CellType.OBSTACLE:
            return False
        
        return True
        
    def reconstruct_path(self, current_pos):
        path = [current_pos]
        while self.start not in path:
            previous_pos = self.parents[current_pos]
            current_pos = tuple(previous_pos)
            path.append(current_pos)
            
        return path
        
    def step(self):
        open_set_array = np.array(self.open_set)
        current_f_values = self.f_scores[open_set_array[:, 0], open_set_array[:, 1]]
        best_idx = np.argmin(current_f_values)
        current_pos = self.open_set[best_idx]

        if np.array_equal(current_pos, self.goal):
            return True, current_pos
        
        self.open_set.pop(best_idx)
        self.closed_set[tuple(current_pos)] = True
        
        for neighbor in self.get_valid_neighbors(current_pos):
            try:
                if self.closed_set[neighbor]:
                    continue
                
                g_score = self.g_scores[current_pos] + self.g(neighbor, current_pos)
                if g_score < self.g_scores[neighbor]:
                    self.parents[neighbor] = current_pos
                    self.g_scores[neighbor] = g_score
                    self.f_scores[neighbor] = g_score + self.h_scores[neighbor]
                    
                    if neighbor not in self.open_set:
                        self.open_set.append(neighbor)
            except:
                breakpoint()
        
        return False, current_pos
    
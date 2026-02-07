import cv2
import numpy as np
import random
import json
from pathlib import Path
from utils import CellType
class GridMap:
    def __init__(self, cols, rows, cell_size, line_thickness=1):
        self.cols = cols
        self.rows = rows
        self.cell_size = cell_size
        self.line_thickness = line_thickness
        
        self.width = cols * cell_size
        self.height = rows * cell_size
        
        # --- Data Storage (The "State") ---
        self.grid_data = np.full((rows, cols), CellType.FREE, dtype=CellType)
        self.start = None
        self.goal = None
        self.circles = []  # List of tuples: (col, row, radius)
        self.path = []
        self.search = []

    def add_start(self, start_column, start_row) -> bool:        
        if start_column > self.cols or start_row > self.rows:
            print("Failed adding start point")
            return False
        
        # Check if start overlaps with an obstacle
        if self.grid_data[start_row, start_column] == CellType.OBSTACLE:
            print("Failed adding start point")
            return False
                
        # Update grid data matrix
        self.start = (start_column, start_row)
        self.grid_data[start_row, start_column] = CellType.START
        return True

    def add_goal(self, goal_column, goal_row) -> bool:
        if goal_column > self.cols or goal_row > self.rows:
            print("Failed adding goal point")
            return False
        
        # Check if goal overlaps with an obstacle
        if self.grid_data[goal_row, goal_column] == CellType.OBSTACLE:
            print("Failed adding goal point")
            return False
                
        # Update grid data matrix
        self.goal = (goal_column, goal_row)
        self.grid_data[goal_row, goal_column] = CellType.GOAL
        return True

    def add_start_and_goal(self, start_coords, goal_coords) -> bool:
        """Stores start and goal coordinates."""
        start_column, start_row = start_coords
        goal_column, goal_row = goal_coords
        
        success = self.add_start(start_column, start_row)
        success = success and self.add_goal(goal_column, goal_row)
        return success
    
    def add_path_point(self, point_coords) -> bool:
        point_column, point_row = point_coords
        if self.grid_data[point_row, point_column] == CellType.GOAL:
            return False
        if self.grid_data[point_row, point_column] == CellType.START:
            return False
        
        self.path.append(np.array([point_column, point_row]))
        self.grid_data[point_row, point_column] = CellType.PATH
        return True

    def add_search_point(self, point_coords) -> bool:
        point_column, point_row = point_coords
        if self.grid_data[point_row, point_column] == CellType.GOAL:
            return False
        if self.grid_data[point_row, point_column] == CellType.START:
            return False
        
        self.search.append(np.array([point_column, point_row]))
        self.grid_data[point_row, point_column] = CellType.SEARCH
        return True

    def add_circular_obstacle(self, center_col, center_row, radius_cells) -> bool:
        """Adds obstacle metadata to the list and updates the logic grid."""
        if self.has_circle_overlap(center_col, center_row, radius_cells):
            return False
        
        self.circles.append((center_col, center_row, radius_cells))
        
        # Pre-calculate squared radius to avoid sqrt() in the loop
        radius_sq = radius_cells ** 2

        # Define the bounding box of the circle
        row_start = max(0, center_row - radius_cells)
        row_end = min(self.rows, center_row + radius_cells + 1)
        col_start = max(0, center_col - radius_cells)
        col_end = min(self.cols, center_col + radius_cells + 1)

        for r in range(row_start, row_end):
            for c in range(col_start, col_end):
                # Calculate squared Euclidean distance from cell center (c, r) to circle center
                dist_sq = (c - center_col)**2 + (r - center_row)**2
                
                if dist_sq <= radius_sq:
                    # Update cell if it's currently free
                    if self.grid_data[r, c] == CellType.FREE:
                        self.grid_data[r, c] = CellType.OBSTACLE
                        
        return True

    def has_circle_overlap(self, center_col, center_row, radius_cells):
        # Pre-calculate squared radius to avoid sqrt() in the loop
        radius_sq = radius_cells ** 2

        # Define the bounding box of the circle
        row_start = max(0, center_row - radius_cells)
        row_end = min(self.rows, center_row + radius_cells + 1)
        col_start = max(0, center_col - radius_cells)
        col_end = min(self.cols, center_col + radius_cells + 1)

        for r in range(row_start, row_end):
            for c in range(col_start, col_end):
                # Calculate squared Euclidean distance from cell center (c, r) to circle center
                dist_sq = (c - center_col)**2 + (r - center_row)**2
                
                if dist_sq <= radius_sq:
                    # Fail circle adding if it has overlap with start or goal points
                    if self.grid_data[r, c] == CellType.START or \
                        self.grid_data[r, c] == CellType.GOAL:
                            return True
                        
        return False

    def make_random_circles(self, count, min_radius=1, max_radius=3):
        placed = 0
        attempts = 0
        #TODO: improve behavior for max number of attempts
        while placed < count and attempts < 200:
            attempts += 1
            c = random.randint(0, self.cols - 1)
            r = random.randint(0, self.rows - 1)
            rad = random.randint(min_radius, max_radius)
            if self.add_circular_obstacle(c, r, rad):
                placed += 1
    
    def make_random_start_and_goal(self):
        #TODO: improve behavior for max number of attempts
        attempts = 0
        while attempts < 200:
            attempts += 1
            c_start = random.randint(0, self.cols - 1)
            r_start = random.randint(0, self.rows - 1)
            if self.add_start(c_start, r_start):
                break
        
        #TODO: improve behavior for max number of attempts
        attempts = 0
        while attempts < 200:
            attempts += 1
            c_goal = random.randint(0, self.cols - 1)
            r_goal = random.randint(0, self.rows - 1)
            if self.add_goal(c_goal, r_goal):
                break
    
    def make_random_scenario(self):
        self.make_random_start_and_goal()
        self.make_random_circles(10, 1, 4)        
    
    def _get_cell_center(self, col, row):
        x = (col * self.cell_size) + (self.cell_size // 2)
        y = (row * self.cell_size) + (self.cell_size // 2)
        return (x, y)

    def _draw_cell_rect(self, img, col, row, color):
        p1 = (col * self.cell_size + self.line_thickness, row * self.cell_size + self.line_thickness)
        p2 = ((col+1) * self.cell_size - self.line_thickness, (row+1) * self.cell_size - self.line_thickness)
        cv2.rectangle(img, p1, p2, color, -1)

    def _draw_grid_cells(self, canvas):
        """Iterates through the grid and draws non-free cells onto the canvas."""
        # Define a mapping of CellType to RGB colors
        color_map = {
            CellType.OBSTACLE: (0, 0, 0),
            CellType.START: (0, 255, 0),
            CellType.GOAL: (0, 0, 255),
            CellType.SEARCH: (0, 200, 255),
            CellType.PATH: (0, 130, 255)
        }

        # np.ndenumerate returns ((row, col), value)
        for (r, c), cell_type in np.ndenumerate(self.grid_data):
            if cell_type in color_map:
                self._draw_cell_rect(canvas, c, r, color_map[cell_type])

    def render(self):
        """The single source of truth for drawing the current state."""
        # Create blank canvas
        canvas = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

        # Draw cells
        self._draw_grid_cells(canvas)
        
        # Draw Obstacles
        for c, r, rad in self.circles:
            center_px = self._get_cell_center(c, r)
            radius_px = int(rad * self.cell_size)
            cv2.circle(canvas, center_px, radius_px, (80, 80, 80), -1)

        # Draw Grid Lines
        for x in range(0, self.width + 1, self.cell_size):
            cv2.line(canvas, (x, 0), (x, self.height), (200, 200, 200), self.line_thickness)
        for y in range(0, self.height + 1, self.cell_size):
            cv2.line(canvas, (0, y), (self.width, y), (200, 200, 200), self.line_thickness)
            
        return canvas

    def show(self, wait_ms = 0):
        img = self.render()
        cv2.imshow("Grid Map", img)
        key = cv2.waitKey(wait_ms)
        if key == ord('d'):
            breakpoint()
        if key == ord('q'):
            cv2.destroyAllWindows()
            quit()
     
    def save_to_json(self, file_path: str):
        """Saves the map configuration and state to a JSON file."""
        data = {
            "config": {
                "cols": self.cols,
                "rows": self.rows,
                "cell_size": self.cell_size,
                "line_thickness": self.line_thickness
            },
            "state": {
                "start": self.start,    # [col, row]
                "goal": self.goal,      # [col, row]
                "circles": self.circles  # List of [col, row, rad]
            }
        }
        
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Map saved to {file_path}")

    @classmethod
    def load_from_json(cls, file_path: str):
        """Creates a new GridMap instance from a JSON file."""
        if not Path(file_path).exists():
            print(f"File {file_path} not found.")
            return None

        with open(file_path, 'r') as f:
            data = json.load(f)

        # 1. Reconstruct the object with original dimensions
        config = data["config"]
        new_map = cls(
            cols=config["cols"], 
            rows=config["rows"], 
            cell_size=config["cell_size"], 
            line_thickness=config["line_thickness"]
        )

        # 2. Re-apply the state
        state = data["state"]
        
        # Load Start and Goal
        if state["start"] and state["goal"]:
            # add_start_and_goal expects (col, row)
            new_map.add_start_and_goal(state["start"], state["goal"])

        # Load Obstacles
        for circle in state["circles"]:
            # circle expects [col, row, rad]
            new_map.add_circular_obstacle(circle[0], circle[1], circle[2])

        return new_map

if __name__ == "__main__":
    # 1. Create and save a random map
    filename = "maps/map2.json"
    
    #original_grid = GridMap(cols=30, rows=20, cell_size=30)
    #original_grid.make_random_circles(15, 1, 3)
    #original_grid.make_random_start_and_goal()
    #original_grid.save_to_json(filename)
    #original_grid.show()
    
    # 2. Load the map from the file
    loaded_grid = GridMap.load_from_json(filename)
    
    if loaded_grid:
        loaded_grid.show()
        
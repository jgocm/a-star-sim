from map import GridMap
from astar import AStar
import utils
import cv2

if __name__ == "__main__":
    path_to_map = 'maps/map3.json'
    map = GridMap.load_from_json(path_to_map)
    
    astar = AStar(map.grid_data, map.start, map.goal)
    finished = False
    i = 0
    while not finished:
        finished, current_pos = astar.step()
        i += 1
        print(i, finished, current_pos)
        map.add_search_point(current_pos)
        map.show(1)
    
    path = astar.reconstruct_path(current_pos)
    for point in reversed(path):
        map.add_path_point(point)
        map.show(0)
    
    cv2.destroyAllWindows()
    
        
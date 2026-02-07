from map import GridMap
from astar import AStar
import utils

if __name__ == "__main__":
    path_to_map = 'maps/map2.json'
    map = GridMap.load_from_json(path_to_map)
    
    print(f'start: {map.start}')
    print(f'goal: {map.goal}')
    print(f'distance: {utils.dist(map.start, map.goal)}')
    
    astar = AStar(map.grid_data, map.start, map.goal)
    finished = False
    i = 0
    while not finished:
        finished, current_pos = astar.step()
        i += 1
        print(i, finished, current_pos)
        map.add_path_point(current_pos)
        map.show(1)
        # TODO: debug x, y coordinates, there might be something inverted
        
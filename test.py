from map import GridMap
from astar import AStar
import cv2

if __name__ == "__main__":
    #path_to_map = 'maps/map3.json'
    map = GridMap(cols=40,
                  rows=10,
                  cell_size=20)
    map.make_random_scenario()
    
    astar = AStar(map.grid_data, map.start, map.goal)
    success = False
    i = 0
    while not success:
        success, current_pos = astar.step()
        if current_pos == None:
            print('There is no path to the goal')
            break
        i += 1
        print(i, success, current_pos)
        map.add_search_point(current_pos)
        map.show(1)
    
    map.show(0)
    if success:
        path = astar.reconstruct_path(current_pos)
        for point in reversed(path):
            map.add_path_point(point)
            map.show(0)
    
    cv2.destroyAllWindows()
    
        
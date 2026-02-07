from map import GridMap
from astar import AStar
import cv2

if __name__ == "__main__":
    map = GridMap(cols=40,
                  rows=10,
                  cell_size=20)
    map.make_random_scenario()
    
    #path_to_map = 'maps/map4.json'
    #map = GridMap.load_from_json(path_to_map)
    
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
        checkpoints = list(reversed(astar.reconstruct_path(current_pos)))
        path = [map.get_cell_center(x, y) for x, y in checkpoints]
        for center_px, point in zip(path, checkpoints):
            # testing drawings
            map.add_path_point(point)
            img = map.render()
            cv2.drawMarker(img, center_px, (0, 0, 0), markerType=cv2.MARKER_CROSS)
            cv2.imshow("Grid Map", img)
            cv2.waitKey(0)
            
            
            #map.show(0)
            
        
    
    cv2.destroyAllWindows()
    
        
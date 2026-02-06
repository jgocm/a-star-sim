from map import GridMap
import utils

if __name__ == "__main__":
    path_to_map = 'maps/map2.json'
    map = GridMap.load_from_json(path_to_map)
    
    print(f'start: {map.start}')
    print(f'goal: {map.goal}')
    print(f'distance: {utils.dist(map.start, map.goal)}')
    map.show()
    
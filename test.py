from map import GridMap
from astar import AStar
import cv2
import numpy as np
from scipy.interpolate import splprep, splev

def smooth_path(astar_path, num_samples=100):
    """
    Returns a smoothed path as a list of (x, y) integer tuples.
    """
    if len(astar_path) < 3:
        return astar_path
    
    path = np.array(astar_path)
    x, y = path[:, 0], path[:, 1]

    # Calculate spline
    tck, u = splprep([x, y], s=0.5, k=3)
    u_fine = np.linspace(0, 1, num_samples)
    x_smooth, y_smooth = splev(u_fine, tck)
    
    # Convert to rounded integers
    x_int = np.round(x_smooth).astype(int)
    y_int = np.round(y_smooth).astype(int)
                
    return list(zip(x_int, y_int))

if __name__ == "__main__":
    
    # Generate random map
    map = GridMap(cols=40,
                  rows=10,
                  cell_size=20)
    map.make_random_scenario()
    
    # Load map from previously saved json
    #path_to_map = 'maps/map4.json'
    #map = GridMap.load_from_json(path_to_map)
    
    # Instantiate A*
    astar = AStar(map.grid_data, map.start, map.goal)
    
    # Run A* search step-by-step and show on screen
    success = False
    while not success:
        success, current_pos = astar.step()
        if current_pos == None:
            print('There is no path to the goal')
            break
        #print(success, current_pos)
        map.add_search_point(current_pos)
        map.show(1)
    
    # Show result after search
    map.show(1)
    
    if not success:
        cv2.destroyAllWindows()
        quit()
        
    # Reconstruct path (if found)
    checkpoints = astar.reconstruct_path(current_pos)
    path = [map.get_cell_center(x, y) for x, y in checkpoints]
    poses = astar.get_poses_from_path(path)
    for pose, point in zip(poses, checkpoints):
        center_px = pose[0], pose[1]
        theta = pose[2]
        map.add_path_point(point)
        map.add_pose(pose)
        map.show(1)

    # Show path
    map.show(0)
    
    # Optimize path with interpolation
    map.reset_poses()
    new_path = smooth_path(path)
    new_poses = astar.get_poses_from_path(new_path)
    step = 10
    last_pose_idx = len(new_poses) - 1
    for i, (pose, point) in enumerate(zip(new_poses, new_path)):
        center_px = pose[0], pose[1]
        theta = pose[2]
        map.add_trajectory_point(point)
        if i % step == 0 or i == last_pose_idx:
            map.add_pose(pose)
        map.show(1)
    
    # Show new path
    map.show(0)
    
    cv2.destroyAllWindows()
        
    
    
    
        
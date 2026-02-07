import numpy as np
import cv2
from enum import Enum
class CellType(Enum):
    FREE = 0
    OBSTACLE = 1
    START = 2
    GOAL = 3
    PATH = 4

def manhattan_distance(p1, p2):
    return np.abs(p1[0] - p2[0]) + np.abs(p1[1] - p2[1])

def euclidean_dist(p1, p2):
    return np.linalg.norm(p1 - p2)

def dist(p1, p2, type='euclidean'):
    if type == 'euclidean':
        return euclidean_dist(p1, p2)
    elif type == 'manhattam':
        return manhattan_distance(p1, p2)
        
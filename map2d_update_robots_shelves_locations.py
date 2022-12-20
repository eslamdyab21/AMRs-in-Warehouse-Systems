import numpy as np
import string


class Map2D():
    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y
        self.map = np.zeros((size_x, size_y))
        self.map = self.map.astype('int')
        self.map = self.map.astype('object')
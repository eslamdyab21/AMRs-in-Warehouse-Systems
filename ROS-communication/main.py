from Map2D import Map2D
from Robot import Robot
from Shelf import Shelf
from Control import Control
from Logger import Logger

from random import randrange





map_size_x = 15
map_size_y = 15

# Create a 2d map
map = Map2D(size_x=map_size_x, size_y=map_size_y)

logger = Logger()


x_y = [randrange(map_size_y), randrange(map_size_y)]
R = Robot(logger, id = 'R1', intial_location = x_y, intial_orientation = 'right', speed=1, map_size = [map_size_x, map_size_y])
S = Shelf(logger, id = 'S1', intial_location = x_y, map_size = [map_size_x, map_size_y])

control = Control(logger, robot = R, shelf = S, map = map, robot_ip = '127.0.0.1', roscore_ip = '127.0.0.1')
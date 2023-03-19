from Map2D import Map2D
from Robot import Robot
from Shelf import Shelf
from Control import Control
from Database import Database
from Logger import Logger
import math
from random import gauss
from random import randrange


logger = Logger()

map_size_x = 15
map_size_y = 15

write_or_update = 'write'

# Create a 2d map
map = Map2D(size_x=map_size_x, size_y=map_size_y)
# map.show_map()

# Create a database object to commnicate with database
database = Database(logger)


if write_or_update == 'write':
    # Creating shelfs and robots and inserting them into the database
    my_mean = map_size_y/2
    my_variance = 7
    x_y = [int(gauss(my_mean, math.sqrt(my_variance))),int(gauss(my_mean, math.sqrt(my_variance)))]
    S1 = Shelf(logger, id = 'S1', intial_location = x_y, map_size = [map_size_x, map_size_y])
    # database.write_to_db(S1.id, S1)

    
    x_y = [int(gauss(my_mean, math.sqrt(my_variance))),int(gauss(my_mean, math.sqrt(my_variance)))]
    S2 = Shelf(logger, id = 'S2', intial_location = x_y, map_size = [map_size_x, map_size_y])
    # database.write_to_db(S2.id, S2)

    x_y = [int(gauss(my_mean, math.sqrt(my_variance))),int(gauss(my_mean, math.sqrt(my_variance)))]
    S3 = Shelf(logger, id = 'S3', intial_location = x_y, map_size = [map_size_x, map_size_y])

    x_y = [int(gauss(my_mean, math.sqrt(my_variance))),int(gauss(my_mean, math.sqrt(my_variance)))]
    S4 = Shelf(logger, id = 'S4', intial_location = x_y, map_size = [map_size_x, map_size_y])

    x_y = [int(gauss(my_mean, math.sqrt(my_variance))),int(gauss(my_mean, math.sqrt(my_variance)))]
    S5 = Shelf(logger, id = 'S5', intial_location = x_y, map_size = [map_size_x, map_size_y])

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R1 = Robot(logger, id = 'R1', intial_location = x_y, intial_orientation = 'right', speed=1, map_size = [map_size_x, map_size_y])
    # database.write_to_db(R1.id, R1)

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R2 = Robot(logger, id = 'R2', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
    # database.write_to_db(R2.id, R2)

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R3 = Robot(logger, id = 'R3', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
    # database.write_to_db(R3.id, R3)

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R4 = Robot(logger, id = 'R4', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R5 = Robot(logger, id = 'R5', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R6 = Robot(logger, id = 'R6', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])

    x_y = [randrange(map_size_y), randrange(map_size_y)]
    R6 = Robot(logger, id = 'R7', intial_location = x_y, intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])



if write_or_update == 'update':
    database.update_db(table="Shelves", id=S1.id, parameters={"LocationX":S1.current_location[0], "LocationY":S1.current_location[1], "ProductID":"S1"})





control = Control(logger, database, robots = [R1, R2, R3, R4, R5, R6], 
                  shelvs = [S1, S2, S3, S4, S5], map = map)
# database.query_shelf_id()

# update S1, S2 recived order for testing
database.update_db(table="Shelves", id=S1.id, parameters={"HavingOrder":1})
database.update_db(table="Shelves", id=S2.id, parameters={"HavingOrder":1})
database.update_db(table="Shelves", id=S3.id, parameters={"HavingOrder":1})

map.update_objects_locations({S1.id:S1.locations, S2.id:S2.locations,S3.id:S3.locations, 
                              S4.id:S4.locations, S5.id:S5.locations ,R1.id:R1.locations, 
                              R2.id:R2.locations, R3.id:R3.locations, R4.id:R4.locations,
                              R5.id:R5.locations, R6.id:R6.locations})
map.show_map()

for i in range(10):
    control.steps_map()


logger.log("---------------------------------------------- Line breaker ----------------------------------------------\n")

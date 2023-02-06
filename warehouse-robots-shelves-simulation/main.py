from Map2D import Map2D
from Robot import Robot
from Shelf import Shelf
from Control import Control
from Database import Database
from Logger import Logger


logger = Logger()

map_size_x = 8
map_size_y = 8

write_or_update = 'write'

# Create a 2d map
map = Map2D(size_x=map_size_x, size_y=map_size_y)
# map.show_map()

# Create a database object to commnicate with database
database = Database(logger)


if write_or_update == 'write':
    # Creating shelfs and robots and inserting them into the database
    S1 = Shelf(logger, id = 'S1', intial_location = [1,6], map_size = [map_size_x, map_size_y])
    database.write_to_db(S1.id, S1)

    S2 = Shelf(logger, id = 'S2', intial_location = [2,2], map_size = [map_size_x, map_size_y])
    database.write_to_db(S2.id, S2)

    # S3 = Shelf(logger, id = 'S2', intial_location = [2,2], map_size = [map_size_x, map_size_y])

    # S4 = Shelf(logger, id = 'S2', intial_location = [2,2], map_size = [map_size_x, map_size_y])


    R1 = Robot(logger, id = 'R1', intial_location = [5,3], intial_orientation = 'right', speed=10, map_size = [map_size_x, map_size_y])
    database.write_to_db(R1.id, R1)

    R2 = Robot(logger, id = 'R2', intial_location = [0,3], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
    database.write_to_db(R2.id, R2)

    R3 = Robot(logger, id = 'R3', intial_location = [0,0], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
    database.write_to_db(R3.id, R3)


if write_or_update == 'update':
    database.update_db(table="Shelves", id=S1.id, parameters={"LocationX":S1.current_location[0], "LocationY":S1.current_location[1], "ProductID":"S1"})





control = Control(logger, database, robots = [R1, R2, R3], shelvs = [S2, S1], map_size = [map_size_x, map_size_y])
# database.query_shelf_id()

# update S1, S2 recived order fore testing
database.update_db(table="Shelves", id=S1.id, parameters={"HavingOrder":1})
database.update_db(table="Shelves", id=S2.id, parameters={"HavingOrder":1})

for i in range(5):
    control.steps_map(map)


logger.log("---------------------------------------------- Line breaker ----------------------------------------------\n")

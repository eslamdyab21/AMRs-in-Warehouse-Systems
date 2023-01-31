from Map2D import Map2D
from Robot import Robot
from Shelf import Shelf
from Control import Control
from Database import Database


map_size_x = 8
map_size_y = 8

map = Map2D(size_x=map_size_x, size_y=map_size_y)
# map.show_map()

# Note 1 --> Once a shelf object is created, it must be added to the database

# Creating a shelf and inserting it into the database
database = Database()
S1 = Shelf(id = "'S1'", intial_location = [10,11], map_size = [map_size_x, map_size_y])
database.update_db(table="Shelves", id=S1.id, parameters={"LocationX":S1.intial_location[0], "LocationY":S1.intial_location[1], "ProductID":"'P1'"})


# Another shelf
print("Another shelf")
S2 = Shelf(id = 'S2', intial_location = [2,2], map_size = [map_size_x, map_size_y])
database.update_db(S2.id, S2)

'''
databse = Database(shelvs = S2)
database.write_to_db(S2.id, [S2.intial_location[0], S2.intial_location[1], 'P1'])

map.update_objects_locations({S1.id:S1.locations, S2.id:S2.locations})
map.show_map()
'''

# Note 2 --> Once a robot object is created, it must be added to the database

R1 = Robot(id = "'R1'", intial_location = [5,11], intial_orientation = 'right', speed=10, map_size = [map_size_x, map_size_y])
#database.update_db(R1.id, [R1.speed, R1.prev_location[0], R1.prev_location[1], R1.prev_location[0], R1.prev_location[1], 'S1', 15, R1.battery_precentage])
database.update_db(table="Robots", id=R1.id, parameters={'Speed':R1.speed, 'CurrentLocationX':R1.prev_location[0]})
database.update_db(table="RobotHealth", id=R1.id, parameters={'BatteryLife':R1.battery_precentage})

#R2 = Robot(id = 'R2', intial_location = [0,3], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
#R3 = Robot(id = 'R3', intial_location = [0,0], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])

'''
map.update_objects_locations({R1.id:R1.locations, R2.id:R2.locations, R3.id:R3.locations})
map.show_map()


database = Database(robots = [R1, R2, R3], shelvs = [S1, S2], map_size = [map_size_x, map_size_y])
# update sheves and robots status from db
database.query_from_db()

control = Control(robots = [R1, R2, R3], shelvs = [S2, S1], map_size = [map_size_x, map_size_y])

for i in range(5):
	control.steps_map(map)
	

# Note 3 --> Once any object is updated, it must be updated in the database
'''
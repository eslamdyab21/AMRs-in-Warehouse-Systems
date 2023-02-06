>This is a documentation to issue #3, the `warehouse-robots-shelves-simulation` folder contains a number of classes developed to tackle this issue

We have 5 classes `Control, Database, Map2D, Robot and Shelf` and a `main.py`.


## TO-DO
- `Database` class, querying and writing.
- Handle collision
- Moving the robot with the shelf to the destination after pairing.
- Moving the shelf back to its original location or other location (needs discussing with team)
- Editing `move_forward` method in `Shelves` class (shelves doesn't have `orientation` attribute)
</br>
</br>

## `main` 
```python
from Map2D import Map2D
from Robot import Robot
from Shelf import Shelf
from Control import Control
from Database import Database

map_size_x = 8
map_size_y = 8


map = Map2D(size_x=map_size_x, size_y=map_size_y)
# map.show_map()

  
S1 = Shelf(id = 'S1', intial_location = [5,4], map_size = [map_size_x, map_size_y])
S2 = Shelf(id = 'S2', intial_location = [2,2], map_size = [map_size_x, map_size_y])

map.update_objects_locations({S1.id:S1.locations, S2.id:S2.locations})
map.show_map()

R1 = Robot(id = 'R1', intial_location = [3,3], intial_orientation = 'right', speed=1, map_size = [map_size_x, map_size_y])
R2 = Robot(id = 'R2', intial_location = [0,3], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])
R3 = Robot(id = 'R3', intial_location = [0,0], intial_orientation = 'left', speed=1, map_size = [map_size_x, map_size_y])

map.update_objects_locations({R1.id:R1.locations, R2.id:R2.locations, R3.id:R3.locations})

map.show_map()
```
We first import our classes, create our `2D-map`, create 2 shelves and 3 robots and then update the map with new created shelves and robots and then showing it.
</br>
</br>

```python
database = Database(robots = [R1, R2, R3], shelvs = [S1, S2], map_size = [map_size_x, map_size_y])

# update sheves and robots status from db
database.query_from_db()

control = Control(robots = [R1, R2, R3], shelvs = [S2, S1], map_size = [map_size_x, map_size_y])

for i in range(5):
	control.steps_map(map)
```
Then we create an imaginary database instant to query the updated status of the shelves and robots to use this updated info on the control class.

Then for 5 simulation time instances we call the `control.steps_map` to run the simulation 5 time instances and observing the output. 

#### Sample output:
```python
---------------------------------------
    A  B   C   D   E  F  G  H
A  R3  0   0  R2   0  0  0  0
B   0  0   0   0   0  0  0  0
C   0  0  S2   0   0  0  0  0
D   0  0   0  R1   0  0  0  0
E   0  0   0   0   0  0  0  0
F   0  0   0   0  S1  0  0  0
G   0  0   0   0   0  0  0  0
H   0  0   0   0   0  0  0  0
---------------------------------------
S1 ----> R1 (Min cost = 3) (Costs: [['R1', 3], ['R2', 6], ['R3', 9]])
S2 ----> R2 (Min cost = 3) (Costs: [['R1', 2], ['R2', 3], ['R3', 4]])
---------------------------------------
    A  B   C   D   E  F  G  H
A  R3  0   0   0   0  0  0  0
B   0  0   0  R2   0  0  0  0
C   0  0  S2   0   0  0  0  0
D   0  0   0   0   0  0  0  0
E   0  0   0  R1   0  0  0  0
F   0  0   0   0  S1  0  0  0
G   0  0   0   0   0  0  0  0
H   0  0   0   0   0  0  0  0
---------------------------------------
    A  B   C   D   E  F  G  H
A  R3  0   0   0   0  0  0  0
B   0  0   0   0   0  0  0  0
C   0  0  S2  R2   0  0  0  0
D   0  0   0   0   0  0  0  0
E   0  0   0   0   0  0  0  0
F   0  0   0  R1  S1  0  0  0
G   0  0   0   0   0  0  0  0
H   0  0   0   0   0  0  0  0
---------------------------------------
    A  B     C  D     E  F  G  H
A  R3  0     0  0     0  0  0  0
B   0  0     0  0     0  0  0  0
C   0  0  R2S2  0     0  0  0  0
D   0  0     0  0     0  0  0  0
E   0  0     0  0     0  0  0  0
F   0  0     0  0  R1S1  0  0  0
G   0  0     0  0     0  0  0  0
H   0  0     0  0     0  0  0  0
---------------------------------------

```
</br>
</br>
</br>
</br>
</br>

## `Map2D` class
`Map2D` class is the same as in issue #1 [[Create a 2d map to update robots and shelves locations @1]] 


</br>
</br>
</br>
</br>
</br>

## `Robot` class
`Robot` class creates robots for the warehouse and control its movement.

First we initialize the class with robot's parameters 
```python
class Robot():
	"""
	Robot class creates robots for the warehouse.
	
	:param id: (string) id for the robot to distinguish between robots
	:param intial_location: (list) [x, y]
	:param intial_orientation: (string) robot's intial_orientation (up, down, 
	right, left)
	:param speed: (float) robot speed
	:param map_size: (list) [map_size_x, map_size_y]
	"""
	
	def __init__(self, id, intial_location, intial_orientation, speed, map_size):
		self.id = id
		self.speed = speed
		
		# active_order_status: (boolean) if robot is delivering an order (paired 
		# with a shelf) it's true
		self.active_order_status = False
		
		# paired_with_shelf_status: (boolean) if robot is paired with a shelf 
		# it's true
		self.paired_with_shelf_status = False
		
		self.paired_with_shelf = None
		self.battery_precentage = 100
		
		# cost: (float) robot's path cost from its current location to the shelf 
		# location
		self.cost = None
		
		self.orientation = intial_orientation
		self.map_size = map_size
		
		self.prev_location = intial_location
		self.current_location = intial_location
		self.locations = [self.prev_location, self.current_location]
```

Then we define 3 functions which will govern the robot motion in the map:
1. `move_forward`
2. `rotate_90_degree_clock_wise`
3. `rotate_90_degree_anti_clock_wise`

>The robot can only move forward, rotate 90 degree eathier clockwise or anticlockwise. It can move in the 4 directions `up, down, left abd right` with the help of the rotation methods.

</br>
</br>

#### `move_forward` method
The `move_forward` method moves the robot in the forward direction based on its
current location and orientation.
```python
def move_forward(self):
	"""
	move_forward function moves the robot in the forward direction based on its
	current location and orientation
	"""
	
	self.prev_location = self.current_location.copy()
	
	if (self.orientation == 'down') and (self.current_location[0] != 
	self.map_size[1]):
		self.current_location[0] = self.current_location[0] + 1
	
	elif (self.orientation == 'up') and (self.current_location[0] != 0):
		self.current_location[0] = self.current_location[0] - 1
	
	elif (self.orientation == 'right') and (self.current_location[1] != 
	self.map_size[1]):
		self.current_location[1] = self.current_location[1] + 1
	
	elif (self.orientation == 'left') and (self.current_location[1] != 0):
		self.current_location[1] = self.current_location[1] - 1
	
	self.locations = [self.prev_location, self.current_location]
```

</br>
</br>

#### `rotate_90_degree_clock_wise` method
The `rotate_90_degree_clock_wise` method rotates the robot's orientation based
on the robot current orientation.
```python
def rotate_90_degree_clock_wise(self):
	"""
	rotate_90_degree_clock_wise function rotates the robot's orientation based
	on the robot current orientation.
	"""
	
	if (self.orientation == 'down'):
		self.orientation = 'left'
	
	elif (self.orientation == 'up'):
		self.orientation = 'right'
	
	elif (self.orientation == 'right'):
		self.orientation = 'down'
	
	elif (self.orientation == 'left'):
		self.orientation = 'up'
```

</br>
</br>

#### `rotate_90_degree_anti_clock_wise` method
And similarly the `rotate_90_degree_anti_clock_wise` 
```python
def rotate_90_degree_anti_clock_wise(self):
	"""
	rotate_90_degree_clock_wise function rotates the robot's orientation based
	on the robot current orientation.
	"""
	
	if (self.orientation == 'down'):
		self.orientation = 'right'
	
	elif (self.orientation == 'up'):
		self.orientation = 'left'
	
	elif (self.orientation == 'right'):
		self.orientation = 'up'
	
	elif (self.orientation == 'left'):
		self.orientation = 'down'
```

</br>
</br>
</br>
</br>
</br>

## `Shelf` class
The `Shelf` class is pretty much like the `Robot` class, only difference is that the `Shelf` class doesn't have a rotation method.


</br>
</br>
</br>
</br>
</br>

## `Database` class
`Database` class will be responsible of writing and querying updated data from the database, not complete yet.

</br>
</br>
</br>
</br>
</br>


## `Control` class
- `Control` class controls the robots movement, finds the cost of each robot path to the shelf and decides which robot will move (lowest cost) and gives the steps needed to get to the shelf.
- We have 4 methods `min_cost_robots, steps_map and move`

1. `min_cost_robots` gets robots with minimum cost to shelves, for each shelf it iterate over all robots that are not paired with other shelves and compute the corresponding cost until the robot with minimum cost is found.
   
2. `steps_map` work on the min cost robots to get their steps/movement instructions that they will take to reach the shelf.
   
3. `move` moves the min cost robots based on the instructions obtained in steps_map function.

</br>

First we initialize the class with control parameters 
```python
class Control():
	"""
	Control class controls the robots movement, find the cost of each robot path 
	to the shelf and decides which robot will move (lowest cost) and gives the 
	steps needed to get to the shelf to the chosen robot.
	
	:param robots: (list) list of robots objects in the warehouse [robot1, 
	robot2,....., robotn]
	
	:param shelves: (list) list of shelves objects in the warehouse [shelf1, 
	shelf2,....., shelfn]
	
	:param map_size: (list) [map_size_x, map_size_y]
	"""
	
	def __init__(self, robots, shelvs, map_size):
		self.robots = robots
		self.shelvs = shelvs
		self.map_size = map_size
		# self.min_cost = map_size[0]
		self.robots_with_min_cost_list = []
```

</br>
</br>

####  `min_cost_robots` method
First method `min_cost_robots`, we will break it to two parts
```python
def min_cost_robots(self):
	"""
	min_cost_robots function gets robots with minimum cost to shelves, for each 
	shelf we iterate over all robots that are not paired with other shelves and 
	compute the corresponding cost until the robot with minimum cost is found.
	"""
	
	shelf_cost_vector = []
	shelf_costs_vector = []
	shelvs_recived_order = []
	
	for shelf in self.shelvs:
		if shelf.recived_order_status and shelf.paired_with_robot_status==False:
			for robot in self.robots:
				if robot.paired_with_shelf_status == False:
					robot.active_order_status = False
					cost_vector = abs(np.subtract(robot.current_location, 
									  shelf.current_location))
					cost = cost_vector[0] + cost_vector[1]
					
					shelf_cost_vector.append(cost)
					shelf_costs_vector.append([robot, robot.id, cost])

			# sort robots based on their costs
			shelf_costs_vector = sorted(shelf_costs_vector, key=lambda x: x[2])
			shelvs_recived_order.append([shelf, sum(shelf_cost_vector), 
		                             	 shelf_costs_vector])
		
			# self.min_cost = self.map_size[0]
			shelf_cost_vector = []
			shelf_costs_vector = []
```
In this part we loop over each `ready-to-be-paired shelf` and for each one of those shelves we loop over each `ready-to-be-paired robot`. For each of those robots we get its cost and append this cost in two lists `shelf_cost_vector` and `shelf_costs_vector`.

Then after looping over all `ready-to-be-paired robots` for a given  `ready-to-be-paired shelf`, we sort those appended robots based on their costs (low cost first then higher). And then for each of the `ready-to-be-paired shelves` we append the shelf along with its costs sum of its `ready-to-be-paired robots` and its `ready-to-be-paired robots` too.

</br>
</br>

Then the second part after finishing those two loops, we select the robots which will give min cost for all the system.
```python
# sort shelves desendingly based on higher costs value to give it the robot with
# min path cost
shelvs_recived_order = sorted(shelvs_recived_order, key=lambda x: x[1], 
							  reverse=True)

# print(shelvs_recived_order)
for i in range(len(shelvs_recived_order)):
	shelf = shelvs_recived_order[i][0]
	robot = shelvs_recived_order[i][2][i][0]
	
	shelf_costs_vector = shelvs_recived_order[i][2]
	min_cost = shelf_costs_vector[i][2]
	
	robot.active_order_status = True
	robot.paired_with_shelf_status = True
	shelf.paired_with_robot_status = True
	
	robot.paired_with_shelf = shelf
	shelf.paired_with_robot = robot
	
	self.robots_with_min_cost_list.append(robot)
	
	if shelf.paired_with_robot == None:
		print(str(shelf.id) + " ----> " + "None" + " (Min cost)")
	else:
		print(str(shelf.id) + " ----> " + str(shelf.paired_with_robot.id) + 
		      " (Min cost = " + str(min_cost) + ")" + 
		      " (Costs: " + str([i[1:] for i in shelf_costs_vector]) + ")")
```

</br>
</br>

It's worth giving example on the `shelvs_recived_order` list variable to understand it better, after the two loops of the first part, an example of the `shelvs_recived_order` list variable would be:

- before `sorted(shelvs_recived_order, key=lambda x: x[1], reverse=True)`
```python
[[<Shelf object> , 9, [[<Robot object>, 'R1', 2], [<Robot object>, 'R2', 3],   
					   [<Robot object>, 'R3', 4]]], 
					   
[<Shelf object>, 18, [[<Robot object>, 'R1', 3], [<Robot object>, 'R2', 6], 
					  [<Robot object>, 'R3', 9]]]]
```

</br>

- after `sorted(shelvs_recived_order, key=lambda x: x[1], reverse=True)`
```python
[[<Shelf object>, 18, [[<Robot object>, 'R1', 3], [<Robot object>, 'R2', 6], 
					   [<Robot object>, 'R3', 9]]], 
					   
[<Shelf object>, 9, [[<Robot object>, 'R1', 2], [<Robot object>, 'R2', 3], 
					 [<Robot object>, 'R3', 4]]]]
```

</br>
</br>

Shelves with higher cost sum will be given the robots nearest to it first before the shelves with lower cost sum, that to get the overall minimum cost of the system.

Then the status of those chosen shelves and robots is updated, and the robots are appended to `robots_with_min_cost_list` to later get their steps/movement instructions needed to reach their shelves.

</br>
</br>
</br>

####  `steps_map` method
`steps_map` work on the robots in the `robots_with_min_cost_list` to get their steps/movement instructions that they will take to reach the shelf.

```python
def steps_map(self, map):
	"""
	steps_map function work on the robots in the robots_with_min_cost_list to get 
	their steps/movement instructions that they will take to reach the shelf.
	
	:param map: (2d array) object of the warehouse
	"""
	
	self.min_cost_robots()
	
	for i in range(len(self.robots_with_min_cost_list)):
		robot = self.robots_with_min_cost_list[i]
		horizontal_steps = robot.paired_with_shelf.current_location[1] - 
		                                     robot.current_location[1]
		
		vertical_steps = robot.paired_with_shelf.current_location[0] - 
		                                   robot.current_location[0]
```
for each robot in the `robots_with_min_cost_list` we calculate `horizontal_steps and vertical_steps`, those are the number of steps in the vertical and horizontal direction needed to reach the shelf. 

</br>

Then each simulation time we move one step in the vertical direction until we finish all the vertical then we move the horizontal_steps. this is done in this part down here:
```python
def steps_map(self, map):
	
	for i in range(len(self.robots_with_min_cost_list)):
		...
		...
		...
		...
		if vertical_steps > 0:
			# want to move down
			self.move(robot, 'down')
		
		elif vertical_steps < 0:
			# want to move up
			self.move(robot, 'up')
	
		elif vertical_steps == 0:
			if horizontal_steps > 0:
				# want to move right
				self.move(robot, 'right')
		
			elif horizontal_steps < 0:
				# want to move left
				self.move(robot, 'left')
	
		if (abs(horizontal_steps) == 1 or horizontal_steps == 0) and 
										        vertical_steps == 0:
			map.update_objects_locations({robot.id+robot.paired_with_shelf.id:
			                                                  robot.locations})
		else:
			map.update_objects_locations({robot.id:robot.locations})
		
		  
	map.show_map()
```

And then we update the map and show it at the end.

</br>
</br>
</br>

####  `move` method
`move` moves the min cost robots based on the instructions obtained in steps_map function, and move method is very simple and self explanatory.

>This is a documentation to issue #3, the `warehouse-robots-shelves-simulation` folder contains a number of classes developed to tackle this issue

We have 5 classes `Control, Database, Map2D, Robot and Shelf` and a `main.py`.


## TO-DO
- Optimize A*
- Moving the robot with the shelf to the destination after pairing.
- Moving the shelf back to its original location or other location 
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

# Create a database object to commnicate with database
database = Database(logger)


S1 = Shelf(logger, id = 'S1', intial_location = [3,0], map_size = 
		   [map_size_x, map_size_y])
# database.write_to_db(S1.id, S1)

S2 = Shelf(logger, id = 'S2', intial_location = [2,2], map_size = 
		   [map_size_x, map_size_y])
# database.write_to_db(S2.id, S2)



R1 = Robot(logger, id = 'R1', intial_location = [5,3], intial_orientation = 
		   'right', speed=1, map_size = [map_size_x, map_size_y])
# database.write_to_db(R1.id, R1)

R2 = Robot(logger, id = 'R2', intial_location = [0,3], intial_orientation = 
		   'left', speed=1, map_size = [map_size_x, map_size_y])
# database.write_to_db(R2.id, R2)

R3 = Robot(logger, id = 'R3', intial_location = [0,0], intial_orientation = 
		   'left', speed=1, map_size = [map_size_x, map_size_y])
# database.write_to_db(R3.id, R3)
```
We first import our classes, create our `2D-map`, and a database object instant, create a number of shelves and robots.
</br>
</br>

```python
control = Control(logger, database, robots = [R1, R2], shelvs = [S2, S1], map = map)


# update S1, S2 recived order fore testing
database.update_db(table="Shelves", id=S1.id, parameters={"HavingOrder":1})
database.update_db(table="Shelves", id=S2.id, parameters={"HavingOrder":1})


map.update_objects_locations({S1.id:S1.locations, S2.id:S2.locations, 
							  R1.id:R1.locations, R2.id:R2.locations})

map.show_map()

for i in range(5):
	control.steps_map()
```
Then we make a control object instant to interact with system with, update the shelves status for simulation.

Then for 5 simulation time instances we call the `control.steps_map` to run the simulation 5 time instances and observing the output. 

#### Sample output:
![](images/robots_sim.gif)

The above yellow circle is R2 and the bottom one is R1, the red stars are the shelves and the black lies are the A* path plan. We can see that when a robot moves the other one has a gray box, this gray box indicates an obstacle that the moving robot sees and takes it into consideration when planning the path.
```python
    A  B   C   D  E  F  G  H
A   0  0   0  R2  0  0  0  0
B   0  0   0   0  0  0  0  0
C   0  0  S2   0  0  0  0  0
D  S1  0   0   0  0  0  0  0
E   0  0   0   0  0  0  0  0
F   0  0   0  R1  0  0  0  0
G   0  0   0   0  0  0  0  0
H   0  0   0   0  0  0  0  0
---------------------------------------
S1 ----> R1 (Min cost = 5) (Costs: [['R1', 5], ['R2', 6]])
S2 ----> R2 (Min cost = 3) (Costs: [['R2', 3]])
[(5, 3), (5, 2), (4, 2), (4, 1), (3, 1), (3, 0)] (R1 Path-Plan)
[(0, 3), (0, 2), (1, 2), (2, 2)] (R2 Path-Plan)
    A  B   C  D  E  F  G  H
A   0  0  R2  0  0  0  0  0
B   0  0   0  0  0  0  0  0
C   0  0  S2  0  0  0  0  0
D  S1  0   0  0  0  0  0  0
E   0  0   0  0  0  0  0  0
F   0  0  R1  0  0  0  0  0
G   0  0   0  0  0  0  0  0
H   0  0   0  0  0  0  0  0
---------------------------------------
[(5, 2), (4, 2), (4, 1), (3, 1), (3, 0)] (R1 Path-Plan)
[(0, 2), (1, 2), (2, 2)] (R2 Path-Plan)
    A  B   C  D  E  F  G  H
A   0  0   0  0  0  0  0  0
B   0  0  R2  0  0  0  0  0
C   0  0  S2  0  0  0  0  0
D  S1  0   0  0  0  0  0  0
E   0  0  R1  0  0  0  0  0
F   0  0   0  0  0  0  0  0
G   0  0   0  0  0  0  0  0
H   0  0   0  0  0  0  0  0
---------------------------------------
[(4, 2), (4, 1), (3, 1), (3, 0)] (R1 Path-Plan)
[(1, 2), (2, 2)] (R2 Path-Plan)
    A   B     C  D  E  F  G  H
A   0   0     0  0  0  0  0  0
B   0   0     0  0  0  0  0  0
C   0   0  R2S2  0  0  0  0  0
D  S1   0     0  0  0  0  0  0
E   0  R1     0  0  0  0  0  0
F   0   0     0  0  0  0  0  0
G   0   0     0  0  0  0  0  0
H   0   0     0  0  0  0  0  0
---------------------------------------
[(4, 1), (3, 1), (3, 0)] (R1 Path-Plan)
    A   B     C  D  E  F  G  H
A   0   0     0  0  0  0  0  0
B   0   0     0  0  0  0  0  0
C   0   0  R2S2  0  0  0  0  0
D  S1  R1     0  0  0  0  0  0
E   0   0     0  0  0  0  0  0
F   0   0     0  0  0  0  0  0
G   0   0     0  0  0  0  0  0
H   0   0     0  0  0  0  0  0
---------------------------------------
[(3, 1), (3, 0)] (R1 Path-Plan)
      A  B     C  D  E  F  G  H
A     0  0     0  0  0  0  0  0
B     0  0     0  0  0  0  0  0
C     0  0  R2S2  0  0  0  0  0
D  R1S1  0     0  0  0  0  0  0
E     0  0     0  0  0  0  0  0
F     0  0     0  0  0  0  0  0
G     0  0     0  0  0  0  0  0
H     0  0     0  0  0  0  0  0
---------------------------------------


```
</br>
</br>
</br>
</br>
</br>

## `Map2D` class
`Map2D` class is the same as in issue #1 [[Create a 2d map to update robots and shelves locations @1]] 
but with a one more function `show_astar_map(astart_map, start, goal, route)` which show the matplotlib map of robots and the obstacles they see and the path they plan with A*.

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
	:param intial_orientation: (string) robot's intial_orientation (up, 
	                            down, right, left)
	:param speed: (float) robot speed
	:param map_size: (list) [map_size_x, map_size_y]
	"""
	
	def __init__(self, id, intial_location, intial_orientation, speed, 
	             map_size):
		......
		......

    def move_forward(self):
		"""
		move_forward function moves the robot in the forward direction based 
		on its current location and orientation
		"""
		......
		......


	def rotate_90_degree_clock_wise(self):
		"""
		rotate_90_degree_clock_wise function rotates the robot's orientation 
		based on the robot current orientation.
		"""
		......
		......

	def rotate_90_degree_anti_clock_wise(self):
		"""
		rotate_90_degree_clock_wise function rotates the robot's orientation 
		based on the robot current orientation.
		"""
		......
		......
```

We define 3 functions which will govern the robot motion in the map:
1. `move_forward`
2. `rotate_90_degree_clock_wise`
3. `rotate_90_degree_anti_clock_wise`

>The robot can only move forward, rotate 90 degree eathier clockwise or anticlockwise. It can move in the 4 directions `up, down, left abd right` with the help of the rotation methods.

</br>
</br>

#### `move_forward` method
The `move_forward` method moves the robot in the forward direction based on its
current location and orientation.

</br>
</br>

#### `rotate_90_degree_clock_wise` method
The `rotate_90_degree_clock_wise` method rotates the robot's orientation based
on the robot current orientation.

</br>
</br>

#### `rotate_90_degree_anti_clock_wise` method
And similarly the `rotate_90_degree_anti_clock_wise` 

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
- We have 5 methods `min_cost_robots, query_recived_order_shelfs, steps_map, steps_map_to_shelf, steps_map_to_packaging and move`

1. `min_cost_robots` gets robots with minimum cost to shelves, for each shelf it iterate over all robots that are not paired with other shelves and compute the corresponding cost until the robot with minimum cost is found.
   
2. `steps_map` work on the min cost robots to get their steps/movement instructions that they will take to reach the shelf.
   
3. `move` moves the min cost robots based on the instructions obtained in steps_map function.

</br>

```python

from Path_Planning_Algorithms import Algorithms
import utils

class Control():
	"""
	Control class controls the robots movement, find the cost of each robot 
	path to the shelf and decides which robot will move (lowest cost) and 
	gives the steps needed to get to the shelf to the chosen robot.
	
	:param robots: (list) list of robots objects in the warehouse [robot1, 
	               robot2,....., robotn]
	
	:param shelves: (list) list of shelves objects in the warehouse [shelf1, 
	                shelf2,....., shelfn]
	
	:param map_size: (list) [map_size_x, map_size_y]
	"""
	
	def __init__(self, robots, shelvs, map_size):
		......
		......

	def min_cost_robots(self):
		"""
		min_cost_robots function gets robots with minimum cost to shelves, 
		for each shelf we iterate over all robots that are not paired with 
		other shelves and compute the corresponding cost until the robot 
		with minimum cost is found.
		"""
		......
		......


	def query_recived_order_shelfs(self):
		"""
		query_recived_order_shelfs function queres shelves order status
		
		:return shelves_recived_order_list: a list of shelves ids who 
											recived orders
		"""
		......
		......


	def steps_map(self):
		"""
		steps_map function gets the steps needed for each rootto reach its 
		goal	
		"""
		......
		......


	def steps_map_to_shelf(self):
		"""
		steps_map_to_shelf function uses A* algrothim to plan the path to  
		the shelf and moves the robot to it.
		"""
		......
		......


	def steps_map_to_shelf(self):
		"""
		steps_map_to_packaging function uses A* algrothim to plan the path 
		tothe packaging location and moves the robot to it.
		"""
		......
		......


	def move(self, robot, direction):
		
		"""
		move function moves the min cost robots based on the instructions 
		obtained in steps_map function.
		
		:param robot: robot objects in the warehouse
		:param direction: desired direction to move the robot
		"""
		......
		......
```

</br>
</br>

####  `min_cost_robots` method
First method `min_cost_robots`, we will break it to two parts
```python
def min_cost_robots(self):

	shelf_cost_vector = []
	shelf_costs_vector = []
	shelvs_recived_order = []
	
	for shelf in shelves_recived_order_list:
		if shelf.paired_with_robot_status==False:
			for robot in self.robots:
				if robot.paired_with_shelf_status == False:
					......
					......
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
`steps_map` function gets the steps needed for each robot reach its goal

```python
def steps_map(self):
	"""
	steps_map function gets the steps needed for each robot reach its goal
	"""
	
	start_time = time.time()	
	self.query_recived_order_shelfs()
	self.min_cost_robots()
	self.steps_map_to_shelf()
	self.steps_map_to_packaging()	
	self.map.show_map()
	
	self.logger.log(f'Control : steps_map : {time.time()-start_time} -->')
```

Each simulation time we move one step based on the path from A*, and then we update the map and show it at the end.

</br>
</br>
</br>

####  `steps_map_to_shelf` method
`steps_map_to_shelf` function uses A* algorithm to plan the path to the shelf and moves the robot to it.

</br>
</br>
</br>

####  `steps_map_to_packaging` method
`steps_map_to_packaging`  function uses A* algorithm to plan the path to the packaging location and moves the robot to it.

</br>
</br>
</br>

####  `move` method
`move` moves the min cost robots based on the instructions obtained in steps_map function.

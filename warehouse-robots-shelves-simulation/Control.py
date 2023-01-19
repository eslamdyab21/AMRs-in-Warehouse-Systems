import numpy as np


class Control():
    """
    Control class controls the robots movement, find the cost of each robot path to the shelf
    and decides which robot will move (lowest cost) and gives the steps needed to
    get to the shelf to the chosen robot.

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, robots, shelvs, map_size):

        self.robots = robots
        self.shelvs = shelvs
        self.map_size = map_size
        self.min_cost = map_size[0]
        self.robots_with_min_cost_list = []



    def min_cost_robots(self):
        """
        min_cost_robots function gets robots with minimum cost to shelves, for each shelf
        we iterate over all robots that are not paired with other shelves and compute the
        corresponding cost until the robot with minimum cost is found, this robot is 
        then appended to a list.
        """
        shelf_cost_vector = []
        shelf_costs_vector = []
        shelvs_recived_order = []
        for shelf in self.shelvs:
            if shelf.recived_order_status:   
                for robot in self.robots:
                    if robot.paired_with_shelf_status == False:
                        robot.active_order_status = False  
                        cost_vector = abs(np.subtract(robot.current_location, shelf.current_location))
                        cost = cost_vector[0] + cost_vector[1]
                        
                        shelf_cost_vector.append(cost)
                        shelf_costs_vector.append([robot, robot.id, cost])


                # sort robots based on their costs
                shelf_costs_vector = sorted(shelf_costs_vector, key=lambda x: x[2])

                shelvs_recived_order.append([shelf, sum(shelf_cost_vector), shelf_costs_vector])

                # self.min_cost = self.map_size[0]
                shelf_cost_vector = []
                shelf_costs_vector = []



        # sort shelves desendingly based on higher costs value to give it the robot with
        # min path
        shelvs_recived_order = sorted(shelvs_recived_order, key=lambda x: x[1], reverse=True)
        # print(shelvs_recived_order)
        for i in range(len(shelvs_recived_order)):
            shelf = shelvs_recived_order[i][0]
            robot = shelvs_recived_order[i][2][i][0]
            
            shelf_costs_vector = shelvs_recived_order[i][2]
            min_cost = shelf_costs_vector[i][2]

            robot.active_order_status = True
            robot.paired_with_shelf_status = True
            robot.paired_with_shelf = shelf
            shelf.paired_with_robot = robot

            if shelf.paired_with_robot == None:
                print(str(shelf.id) + " ----> " + "None" + " (Min cost)")
            else:
                print(str(shelf.id) + " ----> " + str(shelf.paired_with_robot.id) + " (Min cost = " + str(min_cost) + ")" + " (Costs: " + str([i[1:] for i in shelf_costs_vector]) + ")")
        


    def steps_map(self):
        """
        steps_map function work on the robots in the robots_with_min_cost_list to get their
        steps/movement instructions that they will take to reach the shelf.
        """
        
        self.min_cost_robots()

        for i in range(len(self.robots_with_min_cost_list)):
            robot = self.robots_with_min_cost_list[i]
            vertical_steps = abs(robot.paired_with_shelf.current_location[1] - robot.current_location[1])
            horizontal_steps = abs(robot.paired_with_shelf.current_location[0] - robot.current_location[0])



"""
problem:
    A  B   C   D   E  F  G  H
A  R3  0   0  R2   0  0  0  0
B   0  0   0   0   0  0  0  0
C   0  0  S2   0   0  0  0  0
D   0  0   0  R1   0  0  0  0
E   0  0   0   0   0  0  0  0
F   0  0   0   0  S1  0  0  0
G   0  0   0   0   0  0  0  0
H   0  0   0   0   0  0  0  0
------------------------------
S2 ----> R1 (Min cost =2)
S1 ----> R2 (Min cost =6)




***************************************
solution:
implemented logic to get min path for each robot

    A  B   C   D   E  F  G  H
A  R3  0   0  R2   0  0  0  0
B   0  0   0   0   0  0  0  0
C   0  0  S2   0   0  0  0  0
D   0  0   0  R1   0  0  0  0
E   0  0   0   0   0  0  0  0
F   0  0   0   0  S1  0  0  0
G   0  0   0   0   0  0  0  0
H   0  0   0   0   0  0  0  0
-----------------------------
S1 ----> R1 (Min cost = 3) (Costs: [['R1', 3], ['R2', 6], ['R3', 9]])
S2 ----> R2 (Min cost = 3) (Costs: [['R1', 2], ['R2', 3], ['R3', 4]])
"""
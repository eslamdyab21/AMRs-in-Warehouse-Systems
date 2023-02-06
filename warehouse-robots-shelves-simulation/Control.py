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
    def __init__(self, logger, database, robots, shelvs, map_size):
        self.logger = logger
        self.database = database

        self.robots = robots
        self.shelvs = shelvs
        self.map_size = map_size
        # self.min_cost = map_size[0]
        self.robots_with_min_cost_list = []



    def min_cost_robots(self):
        """
        min_cost_robots function gets robots with minimum cost to shelves, for each shelf
        we iterate over all robots that are not paired with other shelves and compute the
        corresponding cost until the robot with minimum cost is found.
        """

        shelves_recived_order_list = self.query_recived_order_shelfs()

        shelf_cost_vector = []
        shelf_costs_vector = []
        shelvs_recived_order = []
        for shelf in shelves_recived_order_list:
            if shelf.paired_with_robot_status==False:   
                for robot in self.robots:
                    if robot.paired_with_shelf_status == False:
                        # robot.active_order_status = False  
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

            # robot.active_order_status = True
            robot.paired_with_shelf_status = True
            shelf.paired_with_robot_status = True

            self.database.update_db(table="Shelves", id=shelf.id, parameters={"HavingOrder":0})

            robot.paired_with_shelf = shelf
            shelf.paired_with_robot = robot

            self.robots_with_min_cost_list.append(robot)

            if shelf.paired_with_robot == None:
                info = str(shelf.id) + " ----> " + "None" + " (Min cost)"
                # print(info)
                self.logger.log('Control -->' + info)
            else:
                info = str(shelf.id) + " ----> " + str(shelf.paired_with_robot.id) + " (Min cost = " + str(min_cost) + ")" + " (Costs: " + str([i[1:] for i in shelf_costs_vector]) + ")"
                # print(info)
                self.logger.log('Control -->' + info)


    def query_recived_order_shelfs(self):
        shelves_id_recived_order_list = self.database.query_recived_order_shelfs_id()
        
        shelves_recived_order_list = []
        for shelf in self.shelvs:
            if shelf.id in shelves_id_recived_order_list:
                shelves_recived_order_list.append(shelf)

        return shelves_recived_order_list
        




    def steps_map(self, map):
        """
        steps_map function work on the robots in the robots_with_min_cost_list to get their
        steps/movement instructions that they will take to reach the shelf.

        :param map: (2d array) object of the warehouse
        """
        
        self.query_recived_order_shelfs()
        self.min_cost_robots()

        for i in range(len(self.robots_with_min_cost_list)):
            robot = self.robots_with_min_cost_list[i]
            horizontal_steps = robot.paired_with_shelf.current_location[1] - robot.current_location[1]
            vertical_steps = robot.paired_with_shelf.current_location[0] - robot.current_location[0]

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

            if (abs(horizontal_steps) == 1 or horizontal_steps == 0) and vertical_steps == 0:
                map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
            else:
                map.update_objects_locations({robot.id:robot.locations})

        map.show_map()


    def move(self, robot, direction):
        """
        move function moves the min cost robots based on the instructions obtained 
        in steps_map function.

        :param robot: robot objects in the warehouse
        :param direction: desired direction to move the robot
        """
        
        # print(robot.id, robot.current_location, direction)
        if direction == 'down':
            if robot.orientation == 'down':
                robot.move_forward()
            elif robot.orientation == 'up':
                robot.rotate_90_degree_clock_wise()
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'right':
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'left':
                robot.rotate_90_degree_anti_clock_wise()
                robot.move_forward()


        elif direction == 'up':
            if robot.orientation == 'up':
                robot.move_forward()
            elif robot.orientation == 'down':
                robot.rotate_90_degree_clock_wise()
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'right':
                robot.rotate_90_degree_anti_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'left':
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()


        elif direction == 'right':
            if robot.orientation == 'right':
                robot.move_forward()
            elif robot.orientation == 'left':
                robot.rotate_90_degree_clock_wise()
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'down':
                robot.rotate_90_degree_anti_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'up':
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()


        elif direction == 'left':
            if robot.orientation == 'left':
                robot.move_forward()
            elif robot.orientation == 'right':
                robot.rotate_90_degree_clock_wise()
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'up':
                robot.rotate_90_degree_anti_clock_wise()
                robot.move_forward()
            elif robot.orientation == 'down':
                robot.rotate_90_degree_clock_wise()
                robot.move_forward()


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




****************************************
solution::
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
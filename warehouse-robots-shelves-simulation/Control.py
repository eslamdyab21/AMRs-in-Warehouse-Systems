import numpy as np
import time
from Path_Planning_Algorithms import Algorithms
import utils


class Control():
    """
    Control class controls the robots movement, find the cost of each robot path to the shelf
    and decides which robot will move (lowest cost) and gives the steps needed to
    get to the shelf to the chosen robot.

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, logger, database, robots, shelvs, map):
        self.logger = logger
        self.database = database

        self.robots = robots
        self.shelvs = shelvs
        self.map = map
        # self.min_cost = map_size[0]
        self.robots_with_min_cost_list = []
        self.robots_physically_connected_to_shelves_list = []

        self.path_algorithms = Algorithms()



    def min_cost_robots(self):
        """
        min_cost_robots function gets robots with minimum cost to shelves, for each shelf
        we iterate over all robots that are not paired with other shelves and compute the
        corresponding cost until the robot with minimum cost is found.
        """
        start_time = time.time()


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
        for i in range(0,len(shelvs_recived_order)):
            shelf = shelvs_recived_order[i][0]
            robot = shelvs_recived_order[i][2][0][0]

            shelf_costs_vector = shelvs_recived_order[i][2]
            min_cost = shelf_costs_vector[0][2]

            # robot.active_order_status = True
            robot.paired_with_shelf_status = True
            shelf.paired_with_robot_status = True

            # self.database.update_db(table="Shelves", id=shelf.id, parameters={"HavingOrder":0})

            robot.paired_with_shelf = shelf
            shelf.paired_with_robot = robot

            self.robots_with_min_cost_list.append(robot)

            if shelf.paired_with_robot == None:
                info = str(shelf.id) + " ----> " + "None" + " (Min cost)"
                print(info)
                self.logger.log(f'Control : min_cost_robots : {time.time()-start_time} -->' + info)
            else:
                info = str(shelf.id) + " ----> " + str(shelf.paired_with_robot.id) + " (Min cost = " + str(min_cost) + ")" + " (Costs: " + str([i[1:] for i in shelf_costs_vector]) + ")"
                print(info)
                self.logger.log(f'Control : min_cost_robots : {time.time()-start_time} -->' + info)

            
            # remove current robot from other potintal shelves, so it's not taken twice
            for i in range(i,len(shelvs_recived_order)):
                print(shelf_costs_vector[0][1])
                print('======================')
                print(shelvs_recived_order[i][2][1])
                # if shelf_costs_vector[0][1] == shelvs_recived_order[i][2][1]:
                robot_index = shelvs_recived_order[i][2][1].index(shelf_costs_vector[0][1])                
                del shelvs_recived_order[i][2][robot_index]
                

    def query_recived_order_shelfs(self):
        """
        query_recived_order_shelfs function queres shelves order status

        :return shelves_recived_order_list: a list of shelves ids who recived orders
        """

        start_time = time.time()
        shelves_id_recived_order_list = self.database.query_recived_order_shelfs_id()
        
        shelves_recived_order_list = []
        for shelf in self.shelvs:
            if shelf.id in shelves_id_recived_order_list:
                shelves_recived_order_list.append(shelf)

        self.logger.log(f'Control : query_recived_order_shelfs : {time.time()-start_time} -->')

        return shelves_recived_order_list
        



    def steps_map_to_shelf(self):
        """
        steps_map_to_shelf function uses A* algrothim to plan the path to the shelf
        and moves the robot to it.
        """
        start_time = time.time()

        i=0
        # print(self.robots_with_min_cost_list)
        for robot in self.robots_with_min_cost_list:
            if robot != None:

                start = robot.current_location
                goal = robot.paired_with_shelf.current_location

                robot.astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

                route = self.path_algorithms.astar(robot.astart_map, start, goal)
                print(route)
                
                
                if len(route) == 2:
                    prev_location = robot.current_location.copy()
                    robot.current_location = route[-1]
                    robot.locations = [prev_location, robot.current_location]

                    horizontal_steps = 0
                    vertical_steps = 0
                elif len(route) > 2:
                    horizontal_steps = route[1][1] - robot.current_location[1] 
                    vertical_steps = route[1][0] - robot.current_location[0]
                
                if len(route) > 2:
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

                        else:
                            robot.paired_with_shelf.current_location = robot.current_location
                        
                            # robot is now physicly connected to shelf
                            robot.physically_connected_to_shelf = robot.paired_with_shelf
                            robot.paired_with_shelf.physically_connected_to_robot = robot

                            # appending that robot to a list
                            self.robots_physically_connected_to_shelves_list.append(robot)

                            # delete that robot from the robots_with_min_cost_list
                            # del self.robots_with_min_cost_list[i]
                            self.robots_with_min_cost_list[i] = None
                            # self.steps_map_to_shelf()

                if len(route) == 2:
                    # self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})

                    if horizontal_steps == 0 and vertical_steps == 0:
                        self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})

                else:
                    self.map.update_objects_locations({robot.id:robot.locations})
                
                print(robot.id)
                self.map.show_astar_map(robot.astart_map, robot.current_location, goal, route)
                i = i + 1
            
            
        
        self.logger.log(f'Control : steps_map_to_shelf : {time.time()-start_time} -->')


    
    def steps_map_to_packaging(self):
        """
        steps_map_to_packaging function uses A* algrothim to plan the path to 
        the packaging location and moves the robot to it.
        """
        start_time = time.time()


        i = 0
        for robot in self.robots_physically_connected_to_shelves_list:
            horizontal_steps = robot.current_location[1]
            vertical_steps = robot.current_location[0]

            if vertical_steps < horizontal_steps:
                # want to move up to packaging
                self.move(robot, 'up')

            elif horizontal_steps < vertical_steps:
                # want to left for packaging
                self.move(robot, 'left')

            elif vertical_steps == horizontal_steps:
                # check which region is less crowdy
                top_free_locations = np.count_nonzero(self.map.map[0:vertical_steps][:] == '0')
                left_free_locations = np.count_nonzero(self.map.map[:][0:horizontal_steps] == '0')

                if top_free_locations < left_free_locations:
                    self.move(robot, 'up')
                else:
                    self.move(robot, 'left')
                    

            robot.paired_with_shelf.locations = robot.locations

            
            self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})
            self.database.update_db(table="Shelves", id=robot.paired_with_shelf.id, parameters={"LocationX":robot.current_location[0], "LocationY":robot.current_location[1]})
            

            self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
            
            i = i + 1

        self.logger.log(f'Control : steps_map_to_packaging : {time.time()-start_time} -->')


    def steps_map(self):
        """
        steps_map function gets the steps needed for each robot reach its goal
        """
        start_time = time.time()

        
        self.query_recived_order_shelfs()
        self.min_cost_robots()
        self.steps_map_to_shelf()
        # self.steps_map_to_packaging()
        

        self.map.show_map()
        self.logger.log(f'Control : steps_map : {time.time()-start_time} -->')


    def move(self, robot, direction):
        """
        move function moves the min cost robots based on the instructions obtained 
        in steps_map function.

        :param robot: robot objects in the warehouse
        :param direction: desired direction to move the robot
        """
        
        start_time = time.time()

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
        
        self.logger.log(f'Control : move : {time.time()-start_time} -->')
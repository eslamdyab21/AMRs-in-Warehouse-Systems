#!/usr/bin/env python3

import numpy as np
import time
from Path_Planning_Algorithms import Algorithms
import utils
import rospy
from std_msgs.msg import String, Int16MultiArray, Int8
from std_msgs.msg import Float32
import os
import ast
import random


class Control():
    """
    Control class controls the robots movement, find the cost of each robot path to the shelf
    and decides which robot will move (lowest cost) and gives the steps needed to
    get to the shelf to the chosen robot.

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, logger, robot, shelf, map, robot_ip='127.0.0.1', roscore_ip='127.0.0.1'):
        self.logger = logger

        self.robot = robot
        self.shelf = shelf
        self.map = map

        self.ros_robots_status_dict = None
        self.ros_shelves_status_dict = None

        self.robot_feedback_stm_flag = 0
        self.counter = 0
        self.goal = None
        self.reached_packaging_spot_flag = False
        self.unpacking_flag = False
        self.robots_area_flag = False
        self.worker_counter = 0 
        self.worker_counter_timeout = 5

        self.path_algorithms = Algorithms()

        local_ip = robot_ip
        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node(robot.id, anonymous=True)

        # ros_Rx_topic topic: has robot movement related info 
        # robot_id:movement:speed:location-shelf_id:location
        self.ros_topic_robot = rospy.Publisher(f'ros_{robot.id}_topic', String, queue_size=10)

        # ros_2dmap topic: 2dmap is self.map.map
        self.ros_map = rospy.Publisher('ros_2dmap', String, queue_size=10)

        # ros_robots_status topic: dictionary of robots movement status 
        # ros_robots_status_dict = {'robot_id':{movement_status:'moving or charging or waiting for order', locations:[current, next], battery:x}, ....}
        self.ros_robots_status_topic = rospy.Publisher('ros_robots_status', String, queue_size=10)

        self.ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)


        self.ros_robot_move_stm_topic = rospy.Publisher(f'ros_robot_{robot.id}_move_stm', Int16MultiArray, queue_size=10)

        rospy.Subscriber("ros_assign_robot_2_shelf", String, self.ros_assign_robot_2_shelf_callback)
        rospy.Subscriber("ros_2dmap", String, self.ros_2dmap_callback)

        # subscripted only once at starting from master node to get initial robot position
        rospy.Subscriber("ros_robots_status", String, self.ros_robots_status_callback)

        # dictionary of shelves orders and their locations
        # {shelf_id:{shelf_id:Sx, movement_status:'moving or waiting', received_order_status:'True or False', 'paired_with_robot':R.id}, ....}
        rospy.Subscriber("ros_shelves_status", String, self.ros_shelves_status_callback)

        rospy.Subscriber(f"ros_robot_{robot.id}_moved_feedback_stm_callback", Int8, self.ros_robot_moved_feedback_stm_callback)



    def ros_move(self, movement):
        self.movement.publish(movement)
    

    def ros_set_speed(self, speed):
        speed = Float32(speed)
        self.speed.publish(speed)

  
    def ros_assign_robot_2_shelf_callback(self, data):
        start_time = time.time()

        order_at_shelf = str(data.data)
        robot_id, shelf_id = order_at_shelf.split('-')

        if robot_id == self.robot.id:
            self.robot.active_order_status = True
            self.shelf.id = shelf_id
            self.shelf.current_location = self.ros_shelves_status_dict[shelf_id]['locations'][1]
            self.robot.paired_with_shelf = self.shelf
        
        self.logger.log(f'Robot_Control : ros_assign_robot_2_shelf_callback : {time.time()-start_time} -->')


    def ros_2dmap_callback(self, data):
        start_time = time.time()

        mapp = data.data
        map_str = mapp.strip()
        self.map.map = np.asarray(np.matrix(map_str)) 
        self.map.map = self.map.map.astype('object')
        self.map.map = np.resize(self.map.map,(self.map.size_x,self.map.size_y))
        self.map.map = np.squeeze(self.map.map)

        self.logger.log(f'Control : ros_2dmap_callback : {time.time()-start_time} -->')


    def ros_robots_status_callback(self, data):
        start_time = time.time()
        
        ros_robots_status_dict = data.data
        ros_robots_status_dict = str(data.data)
        self.ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)
        
        self.robot.locations = self.ros_robots_status_dict[self.robot.id]['locations']
        self.robot.current_location = self.ros_robots_status_dict[self.robot.id]['locations'][1]

        self.logger.log(f'Robot_Control : ros_robots_movement_status_callback : {time.time()-start_time} -->')


    def ros_shelves_status_callback(self, data):
        start_time = time.time()

        ros_shelves_status_dict = str(data.data)
        self.ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)
    

    def ros_robot_moved_feedback_stm_callback(self, data):
        start_time = time.time()

        self.robot_feedback_stm_flag = int(data.data)


    def ros_close(self):
        rospy.signal_shutdown('break')




    def steps_map_to_shelf(self, robot):
        """
        steps_map_to_shelf function uses A* algrothim to plan the path to the shelf
        and moves the robot to it.
        """
        start_time = time.time()


        # start = robot.current_location
        # goal = robot.paired_with_shelf.current_location

        start = self.ros_robots_status_dict[robot.id]['locations'][1]
        goal = self.ros_shelves_status_dict[self.shelf.id]['locations'][1]

        astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

        route = self.path_algorithms.astar(astart_map, start, goal)
        
        
        if type(route) == bool:
            print(robot.id, 'No path found')
        else:
            print(robot.id, route)
            horizontal_steps = route[1][1] - robot.current_location[1] 
            vertical_steps = route[1][0] - robot.current_location[0]
        
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

            # skip if no path is found
            # if route == False:
            #     continue

            if len(route) == 2:
                prev_location = robot.current_location.copy()
                robot.current_location = list(route[-1])
                robot.locations = [prev_location, robot.current_location]
                self.shelf.locations = [prev_location, robot.current_location].copy()

                horizontal_steps = 0
                vertical_steps = 0

                self.goal = None

                # self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})
                robot.paired_with_shelf.current_location = robot.current_location
                
                # robot is now physically connected to shelf
                robot.physically_connected_to_shelf = robot.paired_with_shelf
                robot.paired_with_shelf.physically_connected_to_robot = robot
                
                self.map.map[route[0][0]][route[0][1]] = 0
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})


                self.ros_map.publish(str(self.map.map))
                self.ros_topic_robot.publish(f'{robot.id}:{"None"}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                
                self.ros_shelves_status_dict[self.shelf.id]['paired_with_robot_status'] = True
                self.ros_shelves_status_dict[self.shelf.id]['paired_with_robot'] = robot.id
                self.ros_shelves_status_dict[self.shelf.id]['locations'] = self.shelf.locations
                self.ros_shelves_status_topic.publish(str(self.ros_shelves_status_dict))


            elif len(route) > 2:
                self.map.update_objects_locations({robot.id:robot.locations})
                # self.map.update_objects_locations({self.shelf.id:self.shelf.locations})

                self.ros_map.publish(str(self.map.map))   
            
            self.ros_robots_status_dict[robot.id]['locations'] = robot.locations
            
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))

        # self.map.show_astar_map(robot.id, robot.astart_map, robot.current_location, goal, route)    
        self.logger.log(f'Control : steps_map_to_shelf : {time.time()-start_time} -->')


    def get_free_spot(self, mapp, spot_place):

        if spot_place == 'packaging_area':
            free_spot_col = 0
            area = mapp[:,free_spot_col]

        elif spot_place == 'unpacking_area':
            free_spot_col = mapp.shape[1] - 1
            area = mapp[:,free_spot_col]
        
        elif spot_place == 'robots_area':
            free_spot_row = 0
            area = mapp[free_spot_row,:]


        avalible_pos_row_lst = []
        for index in range(len(area)):
            if area[index] == '0':
                avalible_pos_row_lst.append(index)

        if spot_place == 'robots_area':
            free_spot_col = random.choice(avalible_pos_row_lst)
        else:
            free_spot_row = random.choice(avalible_pos_row_lst)
        
        free_spot = [free_spot_row, free_spot_col]

        return free_spot



    def steps_map_to_packaging(self, robot):
        """
        steps_map_to_packaging function uses A* algrothim to plan the path to 
        the packaging location and moves the robot to it.
        """
        start_time = time.time()


        start = self.ros_robots_status_dict[robot.id]['locations'][1]
        if self.goal is None and not self.reached_packaging_spot_flag:
            self.goal = self.get_free_spot(self.map.map.copy(), 'packaging_area')

        if not self.reached_packaging_spot_flag:
            astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

            route = self.path_algorithms.astar(astart_map, start, self.goal)
        else:
            route = [0]


        
        if type(route) == bool:
            print(robot.id, 'No path found')
            self.goal = self.get_free_spot(self.map.map.copy(), 'packaging_area')

        elif len(route) < 2:
            self.goal = None
            self.reached_packaging_spot_flag = True
            print(robot.id, 'Reached packaging area !')

        else:
            print(robot.id, route)

            if len(route) == 2:
                prev_location = robot.current_location.copy()
                robot.current_location = list(route[-1])
                robot.locations = [prev_location, robot.current_location]
                self.shelf.locations = [prev_location, robot.current_location].copy()

                horizontal_steps = 0
                vertical_steps = 0

                # self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})
                robot.paired_with_shelf.current_location = robot.current_location
                
                # robot is now physically connected to shelf
                # robot.physically_connected_to_shelf = robot.paired_with_shelf
                # robot.paired_with_shelf.physically_connected_to_robot = robot
                
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})


                self.ros_map.publish(str(self.map.map))
                self.ros_topic_robot.publish(f'{robot.id}:{"None"}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                

            elif len(route) > 2:
                horizontal_steps = route[1][1] - robot.current_location[1] 
                vertical_steps = route[1][0] - robot.current_location[0]

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

                
                # self.map.update_objects_locations({robot.id:robot.locations})
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
                # self.map.update_objects_locations({self.shelf.id:self.shelf.locations})

                self.ros_map.publish(str(self.map.map))   
            
            self.ros_robots_status_dict[robot.id]['locations'] = robot.locations
            self.ros_shelves_status_dict[self.shelf.id]['locations'] = robot.locations
            self.shelf.locations = robot.locations.copy()
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))
            self.ros_shelves_status_topic.publish(str(self.ros_shelves_status_dict))

        self.logger.log(f'Control : steps_map_to_packaging : {time.time()-start_time} -->')


    def steps_map_to_unpacking(self, robot):
        """
        steps_map_to_packaging function uses A* algrothim to plan the path to 
        the packaging location and moves the robot to it.
        """
        start_time = time.time()


        start = self.ros_robots_status_dict[robot.id]['locations'][1]
        if self.goal is None and not self.reached_unpacking_spot_flag:
            self.goal = self.get_free_spot(self.map.map.copy(), 'unpacking_area')

        if not self.reached_unpacking_spot_flag:
            astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

            route = self.path_algorithms.astar(astart_map, start, self.goal)
        else:
            route = [0]


        
        if type(route) == bool:
            print(robot.id, 'No path found')
            self.goal = self.get_free_spot(self.map.map.copy(), 'unpacking_area')

        elif len(route) < 2:
            self.goal = None
            self.reached_unpacking_spot_flag = True
            self.unpacking_flag = False
            self.robots_area_flag = True

            print(robot.id, 'Reached unpacking area !')

            self.ros_shelves_status_dict[self.shelf.id]['received_order_status'] = False
            self.ros_shelves_status_dict[self.shelf.id]['paired_with_robot_status'] = False
            self.ros_shelves_status_dict[self.shelf.id]['paired_with_robot'] = 'NULL'
            numoforders = self.ros_shelves_status_dict[self.shelf.id]['num_of_orders']
            
            if numoforders > 0:
                self.ros_shelves_status_dict[self.shelf.id]['num_of_orders'] = numoforders - 1
            print(numoforders)
            self.ros_shelves_status_topic.publish(str(self.ros_shelves_status_dict))

            self.ros_robots_status_dict[robot.id]['paired_with_shelf'] = 'NULL'
            self.ros_robots_status_dict[robot.id]['paired_with_shelf_status'] = False
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))


        else:
            print(robot.id, route)

            if len(route) == 2:
                prev_location = robot.current_location.copy()
                robot.current_location = list(route[-1])
                robot.locations = [prev_location, robot.current_location]
                self.shelf.locations = [prev_location, robot.current_location].copy()

                horizontal_steps = 0
                vertical_steps = 0

                robot.paired_with_shelf.current_location = robot.current_location
                
                
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})


                self.ros_map.publish(str(self.map.map))
                self.ros_topic_robot.publish(f'{robot.id}:{"None"}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                

            elif len(route) > 2:
                horizontal_steps = route[1][1] - robot.current_location[1] 
                vertical_steps = route[1][0] - robot.current_location[0]

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

                
                # self.map.update_objects_locations({robot.id:robot.locations})
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
                # self.map.update_objects_locations({self.shelf.id:self.shelf.locations})

                self.ros_map.publish(str(self.map.map))   
            
            self.ros_robots_status_dict[robot.id]['locations'] = robot.locations
            self.ros_shelves_status_dict[self.shelf.id]['locations'] = robot.locations
            self.shelf.locations = robot.locations.copy()
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))
            self.ros_shelves_status_topic.publish(str(self.ros_shelves_status_dict))

        self.logger.log(f'Control : steps_map_to_packaging : {time.time()-start_time} -->')



    def steps_map_to_robots_area(self, robot):
        """
        steps_map_to_packaging function uses A* algrothim to plan the path to 
        the packaging location and moves the robot to it.
        """
        start_time = time.time()


        start = self.ros_robots_status_dict[robot.id]['locations'][1]
        if self.goal is None and not self.reached_robots_area_spot_flag:
            self.goal = self.get_free_spot(self.map.map.copy(), 'robots_area')

        if not self.reached_robots_area_spot_flag:
            astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

            route = self.path_algorithms.astar(astart_map, start, self.goal)
        else:
            route = [0]


        
        if type(route) == bool:
            print(robot.id, 'No path found')
            self.goal = self.get_free_spot(self.map.map.copy(), 'robots_area')

        elif len(route) < 2:
            self.goal = None
            self.reached_robots_area_spot_flag = True
            self.robots_area_flag = False
            self.robot.active_order_status = False 
            self.robot.physically_connected_to_shelf = None

            print(robot.id, 'Reached robots area !')

        else:
            print(robot.id, route)

            if len(route) == 2:
                prev_location = robot.current_location.copy()
                robot.current_location = list(route[-1])
                robot.locations = [prev_location, robot.current_location]

                horizontal_steps = 0
                vertical_steps = 0
                
                self.map.update_objects_locations({robot.id:robot.locations})


                self.ros_map.publish(str(self.map.map))
                self.ros_topic_robot.publish(f'{robot.id}:{"None"}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                

            elif len(route) > 2:
                horizontal_steps = route[1][1] - robot.current_location[1] 
                vertical_steps = route[1][0] - robot.current_location[0]

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

                
                # self.map.update_objects_locations({robot.id:robot.locations})
                self.map.update_objects_locations({robot.id:robot.locations})

                self.ros_map.publish(str(self.map.map))   
            
            self.ros_robots_status_dict[robot.id]['locations'] = robot.locations
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))

        self.logger.log(f'Control : steps_map_to_packaging : {time.time()-start_time} -->')


    def steps_map(self):
        """
        steps_map function gets the steps needed for each robot reach its goal
        """
        start_time = time.time()

        if self.robot.active_order_status and self.robot.physically_connected_to_shelf == None:
            if self.ros_robots_status_dict is not None:
                self.ros_robots_status_dict[self.robot.id]['movement_status'] = 'moving'
                #self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))

                self.steps_map_to_shelf(self.robot)
                # print(self.robot.locations)
                # print(self.ros_robots_status_dict[self.robot.id]['locations'])
                self.map.show_map()
        # self.steps_map_to_packaging()

        

        elif self.robot.physically_connected_to_shelf != None and not self.unpacking_flag and not self.robots_area_flag:
            self.ros_robots_status_dict[self.robot.id]['movement_status'] = 'moving'

            self.steps_map_to_packaging(self.robot)

            if self.reached_packaging_spot_flag:
                if self.worker_counter >= self.worker_counter_timeout:
                    self.unpacking_flag = True
                    # self.robot.active_order_status = False
                    # self.robot.physically_connected_to_shelf = None

                self.worker_counter = self.worker_counter + 1

            # self.map.show_map()
        


        elif self.unpacking_flag:
            self.steps_map_to_unpacking(self.robot)


        elif self.robots_area_flag:
            self.steps_map_to_robots_area(self.robot)
            self.counter = 0


        elif self.robot.active_order_status == False:
            if self.ros_robots_status_dict is not None and self.counter < 10:
                self.reached_packaging_spot_flag = False
                self.reached_unpacking_spot_flag = False
                self.reached_robots_area_spot_flag = False

                self.ros_robots_status_dict[self.robot.id]['movement_status'] = 'waiting for order'
                self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))

                self.map.update_objects_locations({self.robot.id:self.robot.locations})
                self.ros_map.publish(str(self.map.map))
                self.counter = self.counter + 1

        self.logger.log(f'Control : steps_map : {time.time()-start_time} -->')




    def wait_for_stm_response(self):
        # while self.robot_feedback_stm_flag == 0:
        #     pass
        self.robot_feedback_stm_flag = 0



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
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()


            elif robot.orientation == 'up':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()
                
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()


                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'right':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()


                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()


            elif robot.orientation == 'left':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, 1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_anti_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_anti_clock_wise()


                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()


        elif direction == 'up':
            if robot.orientation == 'up':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'down':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'right':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, 1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_anti_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_anti_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'left':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()


        elif direction == 'right':
            if robot.orientation == 'right':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'left':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'down':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, 1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_anti_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_anti_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'up':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()


        elif direction == 'left':
            if robot.orientation == 'left':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'right':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'up':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, 1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_anti_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_anti_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()

            elif robot.orientation == 'down':
                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
                self.ros_topic_robot.publish(f'{robot.id}:"rotate_90_degree_clock_wise":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.rotate_90_degree_clock_wise()

                self.ros_robot_move_stm_topic.publish(Int16MultiArray(data=[robot.speed, 0]))
                self.ros_topic_robot.publish(f'{robot.id}:"move_forward":{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
                self.wait_for_stm_response()
                robot.move_forward()
        
        self.logger.log(f'Control : move : {time.time()-start_time} -->')
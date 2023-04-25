#!/usr/bin/env python3

import numpy as np
import time
from Path_Planning_Algorithms import Algorithms
import utils
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import os


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

        self.path_algorithms = Algorithms()

        local_ip = robot_ip
        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node(robot.id, anonymous=True)
        #robot_id:movement:speed:location-shelf_id:location
        self.ros_topic_robot = rospy.Publisher(f'{robot.id}_topic', String, queue_size=10)
        self.ros_map = rospy.Publisher('ros_2dmap', String, queue_size=10)

        rospy.Subscriber("order_at_shelf", String, self.ros_order_at_shelf_callback)


    def ros_move(self, movement):
        self.movement.publish(movement)
    

    def ros_set_speed(self, speed):
        speed = Float32(speed)
        self.speed.publish(speed)

  
    def ros_order_at_shelf_callback(self, data):
        order_at_shelf = str(data.data)
        robot_id, shelf_id, shelf_loc = order_at_shelf.split('_')
        if robot_id == self.robot.id:
            self.robot.active_order_status = True
            self.shelf.id = shelf_id
            self.shelf.current_location = [int(shelf_loc.split(',')[0]), int(shelf_loc.split(',')[1])]
            self.robot.paired_with_shelf = self.shelf


    def ros_close(self):
        rospy.signal_shutdown('break')




    def steps_map_to_shelf(self, robot):
        """
        steps_map_to_shelf function uses A* algrothim to plan the path to the shelf
        and moves the robot to it.
        """
        start_time = time.time()


        start = robot.current_location
        goal = robot.paired_with_shelf.current_location

        robot.astart_map = utils.convert_warehouse_map_to_astart_map(self.map.map.copy(), robot.id)

        route = self.path_algorithms.astar(robot.astart_map, start, goal)
        print(robot.id, route)
        
        # skip if no path is found
        # if route == False:
        #     continue

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
                
                    # robot is now physically connected to shelf
                    robot.physically_connected_to_shelf = robot.paired_with_shelf
                    robot.paired_with_shelf.physically_connected_to_robot = robot


        if len(route) == 2:
            # self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})

            if horizontal_steps == 0 and vertical_steps == 0:
                self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
                self.ros_map.publish(str(self.map.map))

        elif len(route) >2:
            self.map.update_objects_locations({robot.id:robot.locations})
            # self.map.update_objects_locations({self.shelf.id:self.shelf.locations})

            self.ros_map.publish(str(self.map.map))
        
        
        # self.map.show_astar_map(robot.id, robot.astart_map, robot.current_location, goal, route)    
            
        
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

            
            # self.database.update_db(table="Robots", id=robot.id, parameters={"CurrentLocationX":robot.current_location[0], "CurrentLocationY":robot.current_location[1]})
            # self.database.update_db(table="Shelves", id=robot.paired_with_shelf.id, parameters={"LocationX":robot.current_location[0], "LocationY":robot.current_location[1]})
            

            self.map.update_objects_locations({robot.id+robot.paired_with_shelf.id:robot.locations})
            
            i = i + 1

        self.logger.log(f'Control : steps_map_to_packaging : {time.time()-start_time} -->')


    def steps_map(self):
        """
        steps_map function gets the steps needed for each robot reach its goal
        """
        start_time = time.time()


        if self.robot.active_order_status:
            self.steps_map_to_shelf(self.robot)
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
            self.ros_topic_robot.publish(f'{robot.id}:{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
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
            self.ros_topic_robot.publish(f'{robot.id}:{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
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
            self.ros_topic_robot.publish(f'{robot.id}:{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
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
            self.ros_topic_robot.publish(f'{robot.id}:{direction}:{robot.locations}:{robot.speed}-{self.shelf.id}:{self.shelf.locations}')
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
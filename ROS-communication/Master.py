#!/usr/bin/env python3

import numpy as np
import rospy
import os
from std_msgs.msg import String
import time
import ast
from Map2D import Map2D



class master():

    def __init__(self, local_ip='127.0.0.1', roscore_ip='127.0.0.1'):

        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node('master', anonymous=True)

        # dictionary of shelves orders and their locations
        # {shelf_id:{shelf_id:Sx, movement_status:'moving or waiting', received_order_status:'True or False', 'paired_with_robot':R.id}, ....}
        rospy.Subscriber("ros_shelves_status", String, self.ros_shelves_status_callback)

        # string value of coming order at which shelf
        rospy.Subscriber("ros_order_at_shelf", String, self.ros_order_at_shelf_callback)


        rospy.Subscriber("ros_2dmap", String, self.ros_2dmap_callback)
        rospy.Subscriber("ros_robots_status", String, self.ros_robots_status_callback)

        self.ros_assign_robot_2_shelf_topic = rospy.Publisher('ros_assign_robot_2_shelf', String, queue_size=10)
        
        # ros_robots_status topic: dictionary of robots movement status 
        # ros_robots_status_dict = {'robot_id':{movement_status:'moving or charging or waiting for order', locations:[current, next], battery:x}, ....}
        self.ros_robots_status_topic = rospy.Publisher('ros_robots_status', String, queue_size=10)
        
        self.ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)


        map_size_x = 15
        map_size_y = 15
        self.ros_robots_status_dict = None

        # Create a 2d map
        self.map = Map2D(size_x=map_size_x, size_y=map_size_y)


    def ros_order_at_shelf_callback(self, data):
        start_time = time.time()

        order_at_shelf_id = str(data.data)

        self.ros_shelves_status_dict[order_at_shelf_id]['received_order_status'] = True

        # get all available robots
        available_robots_for_order = self.get_available_robots_for_order()
        
        #-------------- assign coming shelf to robot ------------
        # min cost robot
        min_cost_robot_id = self.min_cost_robot(order_at_shelf_id, available_robots_for_order)

        # update ros_shelves_status_dict with shelf paired_with_robot
        if min_cost_robot_id != None:
            self.ros_robots_status_dict[min_cost_robot_id]['paired_with_shelf_status'] = True
            self.ros_robots_status_dict[min_cost_robot_id]['paired_with_shelf'] = order_at_shelf_id
            self.ros_shelves_status_dict[order_at_shelf_id]['paired_with_robot_status'] = True
            self.ros_robots_status_dict[min_cost_robot_id]['paired_with_robot'] = min_cost_robot_id
            
            # update ros_robots_status_topic and  ros_shelves_status_topic
            self.ros_robots_status_topic.publish(str(self.ros_robots_status_dict))
            self.ros_shelves_status_topic.publish(str(self.ros_shelves_status_dict))
            
            # update ros_assign_robot_2_shelf topic
            assign_message = f'{min_cost_robot_id}-{order_at_shelf_id}'
            self.ros_assign_robot_2_shelf_topic.publish(assign_message)


    def ros_shelves_status_callback(self, data):
        start_time = time.time()

        ros_shelves_status_dict = str(data.data)
        self.ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)


        # self.logger.log(f'Control : ros_order_at_shelf_callback : {time.time()-start_time} -->')


    def ros_robots_status_callback(self, data):
        start_time = time.time()

        ros_robots_status_dict = str(data.data)
        self.ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)
        

        # self.logger.log(f'Control : ros_robots_movement_status_callback : {time.time()-start_time} -->')


    def get_available_robots_for_order(self):
        available_robots_for_order = []

        for robot_id in self.ros_robots_status_dict.keys():
            if self.ros_robots_status_dict[robot_id]['movement_status'] == 'waiting for order':
                available_robots_for_order.append(robot_id)


        return available_robots_for_order
    

    def ros_2dmap_callback(self, data):

        mapp = data.data
        map_str = mapp.strip()
        self.map.map = np.asarray(np.matrix(map_str)) 
        self.map.map = self.map.map.astype('object')
        self.map.map = np.resize(self.map.map,(self.map.size_x,self.map.size_y))
        self.map.map = np.squeeze(self.map.map)

        self.map.show_map()
    


    def min_cost_robot(self, order_at_shelf_id, available_robots_for_order):
        """
        min_cost_robot function gets robot with minimum cost to shelf, for each shelf
        we iterate over all robots that are not paired with other shelves and compute the
        corresponding cost until the robot with minimum cost is found.
        """
        start_time = time.time()


        min_cost = 100000
        min_cost_robot_id = None        
        for robot_id in available_robots_for_order:
            robot_current_location = self.ros_robots_status_dict[robot_id]['locations'][1]
            shelf_current_location = self.ros_shelves_status_dict[order_at_shelf_id]['locations'][1]
            
            cost_vector = abs(np.subtract(robot_current_location, shelf_current_location))
            cost = cost_vector[0] + cost_vector[1]
            
            if cost < min_cost:
                min_cost = cost
                min_cost_robot_id = robot_id


        if min_cost_robot_id != None:

            info = order_at_shelf_id + " ----> " + min_cost_robot_id + " (Min cost = " + str(min_cost) + ")"
            print(info)
            # self.logger.log(f'Master : min_cost_robot : {time.time()-start_time} -->' + info)

        else:
            info = order_at_shelf_id + " ----> " + "None" + " (Min cost)"
            print(info)
            # self.logger.log(f'Master : min_cost_robots : {time.time()-start_time} -->' + info)
            

        return min_cost_robot_id

master_node = master(local_ip='192.168.1.146', roscore_ip='192.168.1.146')

while True:

    for i in range(1000):
        x = i
        x = x + 1


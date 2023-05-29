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
        # {shelf_id:{movement_status:'moving or waiting', received_order_status:'True or False'}, ....}
        rospy.Subscriber("ros_shelves_status", String, self.ros_shelves_status_callback)
        rospy.Subscriber("ros_2dmap", String, self.ros_2dmap_callback)
        rospy.Subscriber("ros_robots_status", String, self.ros_robots_status_callback)

        self.ros_assign_robot_2_shelf_topic = rospy.Publisher('ros_assign_robot_2_shelf', String, queue_size=10)
        self.ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)

        
        map_size_x = 15
        map_size_y = 15
        self.ros_robots_status_dict = None

        # Create a 2d map
        self.map = Map2D(size_x=map_size_x, size_y=map_size_y)


    def ros_shelves_status_callback(self, data):
        start_time = time.time()

        ros_shelves_status_dict = str(data.data)
        ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)

        
        # get all available robots
        available_robots_for_order = self.get_available_robots_for_order()
        
        # self.logger.log(f'Control : ros_order_at_shelf_callback : {time.time()-start_time} -->')


    def ros_robots_movement_status_callback(self, data):
        start_time = time.time()

        ros_robots_status_dict = str(data.data)
        self.ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)
        

        # self.logger.log(f'Control : ros_robots_movement_status_callback : {time.time()-start_time} -->')


    def get_available_robots_for_order(self):
        available_robots_for_order = []

        for robot_id in self.ros_robots_status_dict.keys():
            if self.ros_robots_status_dict[robot_id] == 'waiting for order':
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
    


    
master_node = master(local_ip='127.0.0.1', roscore_ip='127.0.0.1')

while True:

    for i in range(1000):
        x = i
        x = x + 1


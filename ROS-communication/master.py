#!/usr/bin/env python3

import numpy as np
import rospy
import os
from std_msgs.msg import String
from Map2D import Map2D


class master():

    def __init__(self, local_ip='127.0.0.1', roscore_ip='127.0.0.1'):

        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node('master', anonymous=True)

        rospy.Subscriber("ros_2dmap", String, self.ros_2dmap_callback)


        map_size_x = 15
        map_size_y = 15

        # Create a 2d map
        self.map = Map2D(size_x=map_size_x, size_y=map_size_y)


    
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


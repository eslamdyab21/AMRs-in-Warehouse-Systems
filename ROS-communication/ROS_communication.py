#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import numpy as np
import os

class ROS_communication():

    def __init__(self, robot_id, robot_ip='127.0.0.1', roscore_ip='127.0.0.1'):
        local_ip = robot_ip
        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node(robot_id, anonymous=True)
        self.movement = rospy.Publisher(f'{robot_id}_movement', String, queue_size=10)
        self.speed = rospy.Publisher(f'{robot_id}_speed', Float32, queue_size=10)

    
    def move(self, movement):
        self.movement.publish(movement)
    

    def set_speed(self, speed):
        speed = Float32(speed)
        self.movement.publish(speed)


    def close(self):
        rospy.signal_shutdown('break')
      

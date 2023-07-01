#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from curtsies import Input
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_msgs.msg import Float32MultiArray
import os
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Byte

local_ip='192.168.1.8' 
roscore_ip='192.168.1.20'
roscore_port = 11311
os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)
def talker():
    # pub_yaw = rospy.Publisher('yaw', Float32, queue_size=10)
    pub_speeds = rospy.Publisher('ros_robot_move_stm', Int16MultiArray, queue_size=10)

    rospy.init_node('nh', anonymous=True)
    rate = rospy.Rate(0.2) # 10hz

    while not rospy.is_shutdown():
         y = 0
         yaw = 0
         with Input(keynames='curses') as input_generator:

             for e in input_generator:
                print(repr(e))
                if e == 'k':
                    y = 0
                    yaw = 0
                elif e == 'm':
                    y = int(input("y = "))
                    yaw = int(input("m = "))
            
                if e == 'w':
                    y = 33
                    yaw = 0
                elif e == 's':
                    y = -33
                    yaw = 0
                if e == 'd':
                    y = 0
                    yaw = 100
                elif e == 'a':
                    y = 0
                    yaw = -100

                

                s = Int16MultiArray(data=[y, yaw])
                pub_speeds.publish(s)
                rospy.loginfo(s)
               
         rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

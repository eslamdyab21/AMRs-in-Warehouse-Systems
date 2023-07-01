#!/usr/bin/env python3
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

local_ip='192.168.1.3' 
roscore_ip='192.168.1.20'
roscore_port = 11311
os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)
def talker():
    # pub_yaw = rospy.Publisher('yaw', Float32, queue_size=10)
    pub_speeds = rospy.Publisher('ros_robot_move_stm', Int16MultiArray, queue_size=10)
    pub_pid = rospy.Publisher('pid', Float32MultiArray, queue_size=10)
    pub_setPoint = rospy.Publisher('setPoint', Int16, queue_size=10)

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
                    setPoint = Int16(data=y)
                    pub_setPoint.publish(setPoint)
                    rospy.loginfo(setPoint)
                    yaw = 0
                    kp = 0
                    ki = 0
                    kd = 0
                    constants = Float32MultiArray(data=[kp, ki, kd])
                    pub_pid.publish(constants)
                    rospy.loginfo(constants)
                elif e == 'm':
                    y = int(input("Y speed = "))
                    yaw_sp = int(input("Yaw_sp = "))
                    yaw = 0
                    setPoint = Int16(data=yaw_sp)
                    pub_setPoint.publish(setPoint)
                    rospy.loginfo(setPoint)
                    
            
                if e == 'w':
                    y = 100
                    
                elif e == 's':
                    y = -100
                    
                if e == 'd':
                    yaw = 200
                elif e == 'a':
                    yaw = -200
                if e == 'q':
                    break
                if e == 'c':
                   kp = float(input("Enter Kp = "))
                   ki = float(input("\nEnter Ki = "))
                   kd = float(input("\nEnter Kd = "))
                   constants = Float32MultiArray(data=[kp, ki, kd])
                   pub_pid.publish(constants)
                   rospy.loginfo(constants)
                
                
                s = Int16MultiArray(data=[y, yaw])
                pub_speeds.publish(s)
                rospy.loginfo(s)
                
               
         rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

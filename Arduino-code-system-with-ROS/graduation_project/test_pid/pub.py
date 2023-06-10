#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from curtsies import Input
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_msgs.msg import Float32MultiArray

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Byte


def talker():
    # pub_yaw = rospy.Publisher('yaw', Float32, queue_size=10)
    pub_speeds = rospy.Publisher('speeds', Int16MultiArray, queue_size=10)
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
                    setPoint = Int16(data=y)
                    pub_setPoint.publish(setPoint)
                    rospy.loginfo(setPoint)
                    yaw = int(input("Yaw speed = "))
            
                if e == 'w':
                    y = 100
                    setPoint = Int16(data=y)
                    pub_setPoint.publish(setPoint)
                    rospy.loginfo(setPoint)
                elif e == 's':
                    y = -100
                    setPoint = Int16(data=y)
                    pub_setPoint.publish(setPoint)
                    rospy.loginfo(setPoint)
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

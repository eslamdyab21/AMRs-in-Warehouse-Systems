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
                    y = int(input("horizental limit = "))
                    yaw = int(input("vertical limit = "))
            
                if e == 'w':
                    y = 100
                elif e == 's':
                    y = -100
                if e == 'd':
                    yaw = 100
                elif e == 'a':
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

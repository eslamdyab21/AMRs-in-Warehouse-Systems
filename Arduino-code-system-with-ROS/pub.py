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


# local_ip = '192.168.1.25'
# roscore_ip = '192.168.1.9'
# roscore_port = 11311
# os.environ['ROS_IP'] = local_ip
# os.environ['ROS_HOSTNAME'] = local_ip
# os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

def talker():
    #pub_pressure = rospy.Publisher('pressure', Float32, queue_size=10)
    # pub_yaw = rospy.Publisher('yaw', Float32, queue_size=10)
    # pub_roll = rospy.Publisher('roll', Float32, queue_size=10)
    pub_speeds = rospy.Publisher('speeds', Int16MultiArray, queue_size=10)
    # pub_hamada = rospy.Publisher('hamada', Int16MultiArray, queue_size=10)
    # pub_boost = rospy.Publisher('Boost', Float32MultiArray, queue_size=10)
    # pub_yawpid = rospy.Publisher('yaw_pid', Float32MultiArray, queue_size=10)
    # pub_rollpid = rospy.Publisher('roll_pid', Float32MultiArray, queue_size=10)
    # pub_heightpid = rospy.Publisher('height_pid', Float32MultiArray, queue_size=10)
    # pub_yawsp = rospy.Publisher('yaw_sp', Float32, queue_size=10)
    # pub_rollsp = rospy.Publisher('roll_sp', Float32, queue_size=10)
    # pub_heightsp = rospy.Publisher('height_sp', Float32, queue_size=10)
    # pub_hyper = rospy.Publisher('hyper_parameter', Int16, queue_size=10)
    #pub_solenoids = rospy.Publisher('solenoids',  Byte, queue_size=10)



    rospy.init_node('nh', anonymous=True)
    rate = rospy.Rate(0.2) # 10hz

    while not rospy.is_shutdown():
         m = 0
         y = 0
         with Input(keynames='curses') as input_generator:

             for e in input_generator:
                print(repr(e))
                if e == 'k':
                    m = 0
                    y = 0
                elif e == 'm':
                    m = int(input("horizental limit = "))
                    y = int(input("vertical limit = "))

                if e == 'w':
                    y = 100
                elif e == 's':
                    y = -100
                if e == 'd':
                    m = 100
                elif e == 'a':
                    m = -100

    

               

                s = Int16MultiArray(data=[y, m])
                pub_speeds.publish(s)
                rospy.loginfo(s)
                #h = Int16MultiArray(data = [x, y, z, yaw, 0, roll])
                #pub_hamada.publish(h)


         #scale = 10
         #pub_hyper.publish(scale)
         #rospy.loginfo(scale)

         rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

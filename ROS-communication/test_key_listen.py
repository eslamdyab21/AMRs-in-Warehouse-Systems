#!/usr/bin/env python
import sys
import termios
import contextlib

import os
import rospy
from std_msgs.msg import String, Int16MultiArray
import ast
import time



local_ip='localhost'
roscore_ip='localhost'
roscore_port = 11311

os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

rospy.init_node('test_robot_movement', anonymous=True)
ros_robot_move_stm_topic = rospy.Publisher('ros_robot_R1_move_stm', Int16MultiArray, queue_size=10)

speed = 25

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)



def move(ch):
    if ch == 'w':
        ros_robot_move_stm_topic.publish(Int16MultiArray(data=[speed, 0]))
    elif ch == 's':
        ros_robot_move_stm_topic.publish(Int16MultiArray(data=[-speed, 0]))
    elif ch == 'd':
        ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, -1]))
    elif ch == 'a':
        ros_robot_move_stm_topic.publish(Int16MultiArray(data=[0, 1]))

def main():
    with raw_mode(sys.stdin):
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch or ch == 'c':
                    break
                print(ch)
                move(ch)
                time.sleep(0.5)

        except (KeyboardInterrupt, EOFError):
            pass


if __name__ == '__main__':
    main()
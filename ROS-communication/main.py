from ROS_communication import ROS_communication
import time 

ROS_comm = ROS_communication(robot_id = 'R1', robot_ip = '127.0.0.1', roscore_ip = '127.0.0.1')


i=0
while True:
    ROS_comm.move(f'Forward-{i}')
    ROS_comm.set_speed(i)

    if i == 100:
        i=0

    i = i + 1
    time.sleep(0.2)


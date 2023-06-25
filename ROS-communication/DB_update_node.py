#!/usr/bin/env python3

import numpy as np
import rospy
import os
from std_msgs.msg import String
import time
import ast
from Map2D import Map2D
from Database import Database
from Logger import Logger

class DB_update_node():

    def __init__(self, local_ip='127.0.0.1', roscore_ip='127.0.0.1'):
        self.logger = Logger()

        self.database = Database(self.logger)

        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node('DB_update_node', anonymous=True)

        # dictionary of shelves orders and their locations
        # {shelf_id:{shelf_id:Sx, movement_status:'moving or waiting', received_order_status:'True or False', 'paired_with_robot':R.id}, ....}
        rospy.Subscriber("ros_shelves_status", String, self.ros_shelves_status_callback)
        
        rospy.Subscriber("ros_robots_status", String, self.ros_robots_status_callback)

        self.ros_assign_robot_2_shelf_topic = rospy.Publisher('ros_assign_robot_2_shelf', String, queue_size=10)
        self.ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)

        

        self.ros_robots_status_dict = None

    
    
    def ros_shelves_status_callback(self, data):
        start_time = time.time()

        ros_shelves_status_dict = str(data.data)
        ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)

        for shelf_id,shelf_data in ros_shelves_status_dict.items():
            self.database.update_db(table="Shelves", id=shelf_id, parameters={"location_x":shelf_data['locations'][1][0], "location_y":shelf_data['locations'][1][1]})

        self.logger.log(f'DB_update_node : ros_shelves_status_callback : {time.time()-start_time} -->')


    def ros_robots_status_callback(self, data):
        start_time = time.time()

        ros_robots_status_dict = str(data.data)
        ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)
        
        for robot_id,robot_data in ros_robots_status_dict.items():
            self.database.update_db(table="Robots", id=robot_id, 
            parameters={"currentlocation_x":robot_data['locations'][1][0],
                        "currentlocation_y":robot_data['locations'][1][1], 
                        "shelfid":robot_data['paired_with_shelf'], 
                        "speed":robot_data['speed'],
                        "batterypercentage":robot_data['battery']})
            # ,"isCharging":robot_data['is_charging']

        self.logger.log(f'DB_update_node : ros_robots_status_callback : {time.time()-start_time} -->')
    


    
db_query_node = DB_update_node(local_ip='127.0.0.1', roscore_ip='127.0.0.1')

while True:
    rospy.spin()



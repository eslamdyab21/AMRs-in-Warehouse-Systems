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

class DB_query_node():

    def __init__(self, local_ip='127.0.0.1', roscore_ip='127.0.0.1'):
        self.logger = Logger()

        self.database = Database(self.logger)

        roscore_port = 11311
        os.environ['ROS_IP'] = local_ip
        os.environ['ROS_HOSTNAME'] = local_ip
        os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

        rospy.init_node('DB_query_node', anonymous=True)

        self.ros_order_at_shelf = rospy.Publisher('ros_order_at_shelf', String, queue_size=10)

        # dictionary of shelves orders and their locations
        # {shelf_id:{shelf_id:Sx, movement_status:'moving or waiting', received_order_status:'True or False', 'paired_with_robot':R.id}, ....}
        rospy.Subscriber("ros_shelves_status", String, self.ros_shelves_status_callback)



    def query_recived_order_shelfs_from_db(self):
        """
        query_recived_order_shelfs_from_db function queres shelves order status

        :return shelves_recived_order_list: a list of shelves ids who received orders
        """

        start_time = time.time()
        shelves_id_recived_order_list = self.database.query_recived_order_shelfs_id()
        


        # publishes the shelf who has an order
        for shelf in shelves_id_recived_order_list:
            if self.ros_shelves_status_dict[shelf]['paired_with_robot_status'] == False:
                self.ros_order_at_shelf.publish(shelf)
        
        self.logger.log(f'DB_query_node : query_recived_order_shelfs_from_db : {time.time()-start_time} -->')

        
    def ros_shelves_status_callback(self, data):
        start_time = time.time()

        ros_shelves_status_dict = str(data.data)
        self.ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)


        # self.logger.log(f'Control : ros_order_at_shelf_callback : {time.time()-start_time} -->')

    
db_query_node = DB_query_node(local_ip='127.0.0.1', roscore_ip='127.0.0.1')

while True:
    db_query_node.query_recived_order_shelfs_from_db()
    rospy.spin()



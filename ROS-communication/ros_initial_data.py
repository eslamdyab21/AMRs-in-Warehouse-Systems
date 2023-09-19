import os
import rospy
from std_msgs.msg import String
import ast
import numpy as np
from Map2D import Map2D


map_size_x = 15
map_size_y = 15
map = Map2D(size_x=map_size_x, size_y=map_size_y)
def ros_2dmap_callback(data):
    mapp = data.data
    map_str = mapp.strip()
    map.map = np.asarray(np.matrix(map_str)) 
    map.map = map.map.astype('object')
    map.map = np.resize(map.map,(map.size_x,map.size_y))
    map.map = np.squeeze(map.map)




def ros_shelves_status_callback(data):
    ros_shelves_status_dict = str(data.data)
    ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)

    for shelf_id,shelf_data in ros_shelves_status_dict.items():
        shelf_loc = shelf_data['locations']

        map.update_objects_locations({shelf_id:shelf_loc})

    ros_map.publish(str(map.map))



def ros_robots_status_callback(data):
    ros_robots_status_dict = str(data.data)
    ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)

    for robot_id,robot_data in ros_robots_status_dict.items():
        shelf_loc = robot_data['locations']

        map.update_objects_locations({robot_id:shelf_loc})

    ros_map.publish(str(map.map))
    


local_ip='localhost'
roscore_ip='localhost'
roscore_port = 11311

os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

rospy.init_node('ros_initial_data', anonymous=True)
rospy.Subscriber("ros_shelves_status", String, ros_shelves_status_callback)        
rospy.Subscriber("ros_robots_status", String, ros_robots_status_callback)
rospy.Subscriber("ros_2dmap", String, ros_2dmap_callback)
# ros_2dmap topic: 2dmap is self.map.map
ros_map = rospy.Publisher('ros_2dmap', String, queue_size=10)

ros_robots_status_pub = rospy.Publisher('ros_robots_status', String, queue_size=10)
ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)



ros_robots_status_dict = {'R1':{'movement_status':'waiting for order', 'locations':[[1,1], [1,1]], 'battery':100, 'speed':100, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R2':{'movement_status':'waiting for order', 'locations':[[2,1], [2,1]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R3':{'movement_status':'waiting for order', 'locations':[[4,1], [4,1]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R4':{'movement_status':'waiting for order', 'locations':[[2,4], [2,4]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R5':{'movement_status':'waiting for order', 'locations':[[1,2], [1,2]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R6':{'movement_status':'waiting for order', 'locations':[[8,5], [8,5]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R7':{'movement_status':'waiting for order', 'locations':[[5,8], [5,8]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False}}

ros_shelves_status_dict = {'S1':{'shelf_id':'S1', 'movement_status':'waiting', 'locations':[[6,2], [6,2]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S2':{'shelf_id':'S2', 'movement_status':'waiting', 'locations':[[5,4], [5,4]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S3':{'shelf_id':'S3', 'movement_status':'waiting', 'locations':[[5,9], [5,9]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S4':{'shelf_id':'S4', 'movement_status':'waiting', 'locations':[[3,10], [3,10]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S5':{'shelf_id':'S5', 'movement_status':'waiting', 'locations':[[6,11], [6,11]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S6':{'shelf_id':'S6', 'movement_status':'waiting', 'locations':[[11,6], [11,6]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0}}
# ros_robots_status_pub.publish(str(ros_robots_status_dict))
ros_robots_status_pub.publish(str(ros_robots_status_dict))
ros_shelves_status_topic.publish(str(ros_shelves_status_dict))





while not rospy.is_shutdown():
    ros_robots_status_pub.publish(str(ros_robots_status_dict))
    ros_shelves_status_topic.publish(str(ros_shelves_status_dict))
    rate = rospy.Rate(1) # 10hz
    rate.sleep()
    



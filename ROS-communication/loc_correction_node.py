import os
import rospy
from std_msgs.msg import String
import ast
import numpy as np
from Map2D import Map2D






ros_robots_status_dict = {}
ros_shelves_status_dict = {}
shelves_new_data = False
robots_new_data = False

def ros_shelves_status_callback(data):
    global ros_shelves_status_dict, shelves_new_data
    ros_shelves_status_dict = str(data.data)
    ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)

    shelves_new_data = True


def ros_robots_status_callback(data):
    global ros_robots_status_dict, robots_new_data
    ros_robots_status_dict = str(data.data)
    ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)

    robots_new_data = True


local_ip='localhost'
roscore_ip='localhost'
roscore_port = 11311

os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

rospy.init_node('loc_correction_node', anonymous=True)
rospy.Subscriber("ros_shelves_status", String, ros_shelves_status_callback)        
rospy.Subscriber("ros_robots_status", String, ros_robots_status_callback)

# ros_2dmap topic: 2dmap is self.map.map
ros_map = rospy.Publisher('ros_2dmap', String, queue_size=10)


correction_counter = 0
map_size_x = 15
map_size_y = 15
map = np.zeros((map_size_x, map_size_y))
map = map.astype('int')
map = map.astype('object')


while not rospy.is_shutdown():
    rate = rospy.Rate(1) # 10hz
    rate.sleep()

    print(correction_counter)
    
    
    if correction_counter >= 1 and (robots_new_data or shelves_new_data):
        

        for x in range(0, map_size_x):
            for y in range(0, map_size_y):
                map[x,y] = 0

        for robot_id,robot_data in ros_robots_status_dict.items():
            robot_loc = robot_data['locations'][1]

            map[robot_loc[0]][robot_loc[1]] = robot_id

        for shelf_id,shelf_data in ros_shelves_status_dict.items():
            shelf_loc = shelf_data['locations'][1]

            if str(map[shelf_loc[0]][shelf_loc[1]])[0] == 'R':
                map[shelf_loc[0]][shelf_loc[1]] = str(map[shelf_loc[0]][shelf_loc[1]]) + shelf_id
            else:
                map[shelf_loc[0]][shelf_loc[1]] = shelf_id

        ros_map.publish(str(map))
        
        correction_counter = 0
        shelves_new_data = False
        robots_new_data = False

    
    correction_counter = correction_counter + 1

    



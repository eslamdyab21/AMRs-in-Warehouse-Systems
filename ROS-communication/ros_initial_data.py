import os
import rospy
from std_msgs.msg import String
import ast

def ros_shelves_status_callback(data):
        ros_shelves_status_dict = str(data.data)
        ros_shelves_status_dict = ast.literal_eval(ros_shelves_status_dict)


def ros_robots_status_callback(data):
    ros_robots_status_dict = str(data.data)
    ros_robots_status_dict = ast.literal_eval(ros_robots_status_dict)



local_ip='localhost'
roscore_ip='localhost'
roscore_port = 11311

os.environ['ROS_IP'] = local_ip
os.environ['ROS_HOSTNAME'] = local_ip
os.environ['ROS_MASTER_URI'] = 'http://' + roscore_ip + ':' + str(roscore_port)

rospy.init_node('ros_initial_data', anonymous=True)
rospy.Subscriber("ros_shelves_status", String, ros_shelves_status_callback)        
rospy.Subscriber("ros_robots_status", String, ros_robots_status_callback)

ros_robots_status_pub = rospy.Publisher('ros_robots_status', String, queue_size=10)
ros_shelves_status_topic = rospy.Publisher('ros_shelves_status', String, queue_size=10)



ros_robots_status_dict = {'R1':{'movement_status':'waiting for order', 'locations':[[1,1], [1,1]], 'battery':100, 'speed':100, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R2':{'movement_status':'waiting for order', 'locations':[[2,1], [2,1]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':False},
                          'R3':{'movement_status':'waiting for order', 'locations':[[4,1], [4,1]], 'battery':100, 'speed':35, 'paired_with_shelf':'NULL', 'paired_with_shelf_status':True}}

ros_shelves_status_dict = {'S1':{'shelf_id':'S1', 'movement_status':'waiting', 'locations':[[6,2], [6,2]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0},
                           'S2':{'shelf_id':'S2', 'movement_status':'waiting', 'locations':[[5,4], [5,4]], 'received_order_status':False,  'paired_with_robot_status':False, 'paired_with_robot':'NULL','num_of_orders':0}}
# ros_robots_status_pub.publish(str(ros_robots_status_dict))
ros_robots_status_pub.publish(str(ros_robots_status_dict))
ros_shelves_status_topic.publish(str(ros_shelves_status_dict))





while not rospy.is_shutdown():
    ros_robots_status_pub.publish(str(ros_robots_status_dict))
    ros_shelves_status_topic.publish(str(ros_shelves_status_dict))
    rate = rospy.Rate(1) # 10hz
    rate.sleep()
    



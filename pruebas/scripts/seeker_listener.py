#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

def callback(data):
    rospy.loginfo

def seeker_listener():
    rospy.init_node('seeker_listener', anonymous = True)
    rospy.Subscriber("")
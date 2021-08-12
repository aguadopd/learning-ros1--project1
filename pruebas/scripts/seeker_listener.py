#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

def callback(data):
    for box in data.bounding_boxes:
        rospy.loginfo(
            "I found: {} ".format(box.Class)
        )

def seeker_listener():
    rospy.init_node('seeker_listener', anonymous=True)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            seeker_listener()
    except rospy.ROSInterruptException:
        pass


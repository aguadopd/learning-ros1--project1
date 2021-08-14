#!/usr/bin/env python

""" This script logs found objects of interest' class names """

import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

objects_of_interest = ("person", "stop sign")


def callback(data):
    rospy.loginfo(f'I found :\
        {", ".join([box.Class for box in data.bounding_boxes if box.Class in objects_of_interest])}')
# def callback(data):
#    for box in data.bounding_boxes:
#        rospy.loginfo(
#            "I found: {} ".format(box.Class)
#        )


def seeker_listener():
    rospy.init_node('seeker_listener', anonymous=True)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    rospy.spin()


if __name__ == '__main__':
    # Not printed in the console... why???
    rospy.loginfo("Starting seeker_listener.")
    print("Waiting for bounding_boxes messages to be published...")
    try:
        while not rospy.is_shutdown():
            seeker_listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo('Closing...')

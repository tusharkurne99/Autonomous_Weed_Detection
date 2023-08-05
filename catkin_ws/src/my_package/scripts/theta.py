#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Point, PointStamped
from rospy.numpy_msg import numpy_msg


def straight_ahead_msgs():
    pub = rospy.Publisher("points", geometry_msgs(float64), queue_size=10)
    rospy.init_node('point_msgs', anonymous=True)
    # pub = rospy.Publisher(OBJ_LOC_TOPIC, PointStamped, queue_size=10)
    point = Point(x=5.0, y=0.0, z=0.0)
    while not rospy.is_shutdown():
        time.sleep(2)
        rospy.loginfo(point)
        pub.publish(point)
        # print("Sending point command:", point)
        # pub.publish(PointStamped(point=point))


if __name__ == "__main__":
    straight_ahead_msgs()
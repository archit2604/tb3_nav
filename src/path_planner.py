#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

rospy.init_node("path_planner")
pub = rospy.Publisher("path", Point, queue_size=1)
rate = rospy.Rate(10)
point = Point()
point.x = float(input("Enter x coordinate of goal"))
point.y = float(input("Enter y coordinate of goal"))
point.z = 0
while not rospy.is_shutdown():
    pub.publish(point)
    rate.sleep()

#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node("path_planner")
pub = rospy.Publisher("path", Path, queue_size=2)
rate = rospy.Rate(10)
path = Path()
pose = PoseStamped()
pose.pose.position.x = float(input("Enter x coordinate of goal"))
pose.pose.position.y = float(input("Enter y coordinate of goal"))
pose.pose.position.z = 0
path.poses.append(pose)
while not rospy.is_shutdown():
    pub.publish(path)
    rate.sleep()

#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion

def callback(msg):
    pass


def callback_path(msg):
    global o
    global x_g
    global y_g
    x_g = msg.poses[0].pose.position.x
    y_g = msg.poses[0].pose.position.y
    if o == 0:
        o = 1
        sub_odom = rospy.Subscriber("odom", Odometry, callback_odom)


def callback_odom(msg):
    global count1
    global count2
    global diff_sum
    global diff_prev
    global e
    global e_prev
    global e_sum
    global x_g
    global y_g
    global diff
    twist = Twist()
    euler = euler_from_quaternion(
        quaternion=(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
    )
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if True:
        theta_g = math.atan2(y_g - y, x_g - x)
        diff = theta_g - euler[2]
        diff_sum += diff
        dedt = diff - diff_prev
        twist.angular.z = 3 * (diff) + 0.00044 * (diff_sum) + 30 * dedt
        diff_prev = diff
        if diff <= 0.0008 and diff >= -0.0008:
            twist.angular.z = 0
        pub.publish(twist)
    if True:
        e = math.sqrt(((x_g - x) * (x_g - x)) + ((y_g - y) * (y_g - y)))
        e_sum += e
        dedt = e - e_prev
        twist.linear.x = (1.5 * (e) + 0.00044 * (e_sum) + 28 * dedt) / 30
        if twist.linear.x > 0.22:
            twist.linear.x = 0.22
        e_prev = e
        if e <= 0.05 and e >= -0.05:
            twist.linear.x = 0
        pub.publish(twist)


global count1
count1 = 0
global count2
count2 = 0
global o
o = 0
global diff
diff = 1
global diff_sum
diff_sum = 0
global diff_prev
diff_prev = diff
global e
e = 1
global e_sum
e_sum = 0
global e_prev
e_prev = 0
rospy.init_node("controller_ar")
sub_path = rospy.Subscriber("path", Path, callback_path)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)
rospy.spin()
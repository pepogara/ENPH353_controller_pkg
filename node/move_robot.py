#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size = 1)
pub2 = rospy.Publisher('/score_tracker', String, queue_size = 1)

startTime = rospy.get_time()

rate = rospy.Rate(2)
move = Twist()
move.linear.x = 1
move.angular.z = 0

while rospy.get_time() - startTime < 1:
    rate.sleep()

pub2.publish("a,a,0,a")

while not rospy.is_shutdown():

    pub.publish(move)
    rate.sleep()

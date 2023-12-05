#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import cv2
import numpy as np
from tf.transformations import quaternion_from_euler

class MovePublisher:
    """!
    @brief      This class represents a publisher for movement commands.

    Attributes:
        move_pub (rospy.Publisher): Publisher for movement commands.
        rate (rospy.Rate): Rate control for publishing commands.
    """

    def __init__(self):
        """!
        @brief      Initializes the MovePublisher object.

        This constructor sets up the publisher and rate control for publishing commands.
        """
        self.move_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(30)
        self.teleport = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def move_publisher(self, z, x = 0.4):
        """!
        @brief      Publishes movement commands to control the robot's movement.

        @param      z (float): Angular velocity command. (negative for right, positive for left)
        """
        move = Twist()
        move.linear.x = x
        move.angular.z = z

        self.move_pub.publish(move)
        self.rate.sleep()

    def stop_publisher(self):
        """!
        @brief      Publishes a stop command to stop the robot's movement.
        """
        stop = Twist()
        stop.linear.x = 0
        stop.angular.z = 0

        self.move_pub.publish(stop)
        self.rate.sleep()


    def teleport_to(self, position):
        """!
        @brief      Spawns the robot at the given position.

        @param      position (list): The position to spawn the robot at.
        """

        x, y, z, w = quaternion_from_euler(0, 0, np.radians(position[3]))

        msg = ModelState()
        msg.model_name = 'R1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w

        self.teleport(msg)

        rospy.wait_for_service('/gazebo/set_model_state')


    def center_of_road(self, masked_img, img = None):
        """!
        @brief      Calculates the center of the road in the image.

        @param      masked_img: The already masked image to process.
        @param      img: The original image to draw contours on. (optional)

        @return     The center of the road in the image.
                    If img is not None, also returns the image with contours drawn on it.            
        
                    None if the road is not detected.
        """
        height, width = masked_img.shape
        y = int(height * 2 / 3)  # a third from the bottom of the image
        left_x = None
        right_x = None

        for x in range(width // 2):
            if masked_img[y, x] == 255:
                left_x = x
                break

        for x in range(width - 1, width // 2, -1):
            if masked_img[y, x] == 255:
                right_x = x
                break

        if left_x is not None and right_x is not None:
            center_x = (left_x + right_x) // 2
            return (center_x, y)

        return None

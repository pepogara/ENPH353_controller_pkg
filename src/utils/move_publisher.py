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

    def move_publisher(self, z, x = 0.5):
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


    def center_of_road(self, masked_img, threshold=200, img = None):
        """!
        @brief      Calculates the center of the road in the image.

        @param      masked_img: The already masked image to process.
        @param      threshold: The threshold for the minimum area of a contour to be considered significant.
        @param      img: The original image to draw contours on. (optional)

        @return     The center of the road in the image.
                    If img is not None, also returns the image with contours drawn on it.            
        
                    None if the road is not detected.
        """
        # Find contours
        contours, _ = cv2.findContours(masked_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate center of significant contours about a third of the way up in the y-axis
        height, width = masked_img.shape[:2]
        roi_y = height // 3  # Adjust this value for the desired y-axis position

        significant_contours = []

        for contour in contours:
            # Get bounding rectangle for the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate center of the contour
            center_x = x + (w // 2)
            center_y = y + (h // 2)

            # Check if the contour is within the region of interest and its area is significant
            if y < roi_y and cv2.contourArea(contour) > threshold:  # Adjust threshold accordingly
                significant_contours.append(contour)

        # Calculate average center point
        if significant_contours:
            center_points = [((cv2.boundingRect(cnt)[0] + cv2.boundingRect(cnt)[2] // 2),
                            (cv2.boundingRect(cnt)[1] + cv2.boundingRect(cnt)[3] // 2)) for cnt in significant_contours]

            avg_center = np.mean(center_points, axis=0)
            avg_center = tuple(map(int, avg_center))

            if img.all() != None:
                # Draw contours on the image
                contour_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(contour_image, significant_contours, -1, (0, 255, 0), 2)

                # Draw a point at the calculated center
                cv2.circle(contour_image, avg_center, 5, (0, 0, 255), -1)
                return avg_center, contour_image
            else:
                return avg_center    
                
        else:
            print("No significant contours found within the region of interest.")

            if img.all() != None:
                return None, img
            else:
                return None

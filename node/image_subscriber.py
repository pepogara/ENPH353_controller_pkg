#! /usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber():
    """!
    @brief      Image subscriber class for the robot.
    """

    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.frame_callback, queue_size=10)
        self.image = None

    def frame_callback(self, data):
        """!
        @brief      Callback function for the image subscriber.

        @param      data  The data
        """
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def get_frame(self):
        """
        @brief      Returns the image frame from the camera.

        @return     frame (Image): Image frame from the camera.
        """
        return self.image.copy() if self.image is not None else None
    
    def get_frame_size(self):
        """!
        @brief      Returns the size of the image frame from the camera.

        @return     frame_size (tuple): Size of the image frame from the camera.
        """
        
        frame_size = self.image.shape
        return frame_size
    
    def get_frame_center(self):
        """!
        @brief      Returns the center of the image frame from the camera.

        @return     frame_center (tuple): Center of the image frame from the camera.
        """
        frame_center = (self.image.shape[1]//2, self.image.shape[0]//2)
        return frame_center


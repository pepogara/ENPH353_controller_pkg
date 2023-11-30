#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import image_treating as imgt

class Controller:
    """
    Controller class for the robot.
    """

    def __init__(self):
        """
        Constructor for the Controller class.

        Parameters:
            None

        Returns:
            None
        """
        #rospy.init_node('controller_node', anonymous=True)
        
        self.camera = CameraSubscriber()
        self.move_pub = MovePublisher()
        self.score_pub = ScorePublisher()

        self.startTime = rospy.get_time()

        rospy.Timer(rospy.Duration(1), self.start_timer_callback, oneshot=True)

    def start_timer_callback(self, event):
        """
        Callback function to start the timer after one second.
        """
        self.score_pub.start()
        # Move the robot after starting the timer
        self.move_robot()

    def move_robot(self):
        """
        Function to move the robot for about 2 seconds.
        """
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 8 and not rospy.is_shutdown():
            img = self.camera.get_frame()
            hsv = imgt.HSV(img)
            hint = imgt.homography(hsv, img)
            if hint is not None:
                cv2.namedWindow("hint_frame", cv2.WINDOW_AUTOSIZE)
                cv2.imshow("hint_frame", hint)
                cv2.waitKey(1)
            else:
                cv2.destroyAllWindows()
            # Move the robot for 2 seconds
            # self.move_pub.move_publisher(0.0)
        
        # Stop the robot movement
        cv2.imwrite("bright_hint.jpg", hint)
        self.stop_robot()

    def stop_robot(self):
        """
        Function to stop the robot movement and end the timer.
        """
        self.move_pub.stop_publisher()
        self.score_pub.stop()
        
    def spin(self):
        """
        Spin function for the Controller class.

        """
        rospy.spin()

class CameraSubscriber:
    """
    This class represents a subscriber for the camera.
    
    Attributes: 
        cam_sub (rospy.Subscriber): Subscriber for the camera.
        frame (Image): Image frame from the camera.
    """

    def __init__(self):
        """
        Initializes the CameraSubscriber object.

        This constructor sets up the subscriber for the camera.
        """
        self.cam_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.frame_callback, queue_size=10)
        self.frame = None

    def frame_callback(self, data):
        """
        Callback function for the camera subscriber.

        Parameters:
            data (Image): Image frame from the camera.

        Returns:
            None
        """
        try:
            self.frame = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def get_frame(self):
        """
        Returns the image frame from the camera.

        Parameters:
            None

        Returns:
            frame (Image): Image frame from the camera.
        """
        return self.frame.copy() if self.frame is not None else None
    
    def get_frame_size(self):
        """
        Returns the size of the image frame from the camera.

        Parameters:
            None

        Returns:
            frame_size (tuple): Size of the image frame from the camera.
        """
        frame_size = self.frame.shape
        return frame_size
    
    def get_frame_center(self):
        """
        Returns the center of the image frame from the camera.

        Parameters:
            None

        Returns:
            frame_center (tuple): Center of the image frame from the camera.
        """
        frame_center = (self.frame.shape[1]//2, self.frame.shape[0]//2)
        return frame_center

class MovePublisher:
    """
    This class represents a publisher for movement commands.

    Attributes:
        move_pub (rospy.Publisher): Publisher for movement commands.
        rate (rospy.Rate): Rate control for publishing commands.
    """

    def __init__(self):
        """
        Initializes the MovePublisher object.

        This constructor sets up the publisher and rate control for publishing commands.
        """
        self.move_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)# 30hz because the camera publishes at 30Hz

    def move_publisher(self, z):
        """
        Publishes movement commands to control the robot's movement.

        Parameters:
            z (float): Angular velocity command. (negative for right, positive for left)

        Returns:
            None
        """
        move = Twist()
        move.linear.x = 0.5
        move.angular.z = z

        self.move_pub.publish(move)
        self.rate.sleep()

    def stop_publisher(self):
        """
        Publishes a stop command to stop the robot's movement.

        Parameters:
            None

        Returns:
            None
        """
        stop = Twist()
        stop.linear.x = 0
        stop.angular.z = 0

        self.move_pub.publish(stop)
        self.rate.sleep()

class ScorePublisher:
    """
    This class represents a publisher for the score.

    Attributes:
        score_pub (rospy.Publisher): Publisher for the score.
    """

    def __init__(self):
        """
        Initializes the ScorePublisher object.

        This constructor sets up the publisher for the score.
        """
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)

    def clue_publisher(self, score):
        """
        Publishes the score to the score_tracker topic.

        Parameters:
            score (int): Score to be published.

        Returns:
            None
        """
        pass

    def start(self):
        """
        Starts the timer for the simulation.
        """
        self.score_pub.publish("a,a,0,a")
        
    def stop(self):
        """
        Stops the timer for the simulation.
        """
        self.score_pub.publish("a,a,-1,a")


def main(args):
    """
    Main function for the follow_line script.

    This function creates an instance of the controller package class and spins the ROS node.

    Parameters:
        args (list): Command line arguments.

    Returns:
        None
    """

    try:
        cntrl_pkg = Controller()
        cntrl_pkg.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=True)
    main(sys.argv)

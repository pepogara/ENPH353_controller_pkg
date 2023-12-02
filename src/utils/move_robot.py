#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

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
        rospy.init_node('controller_node', anonymous=True)

        # self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw topic', Image, self.frame_callback, queue_size=10)
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

        while rospy.get_time() - start_time < 3 and not rospy.is_shutdown():
            # Move the robot for 2 seconds
            self.move_pub.move_publisher(0.0)
        
        # Stop the robot movement
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
        move.linear.x = 1
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
    main(sys.argv)
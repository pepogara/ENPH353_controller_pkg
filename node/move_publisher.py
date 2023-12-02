import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
        self.move_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)# 30hz because the camera publishes at 30Hz

    def move_publisher(self, z):
        """!
        @brief      Publishes movement commands to control the robot's movement.

        @param      z (float): Angular velocity command. (negative for right, positive for left)
        """
        move = Twist()
        move.linear.x = 1
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
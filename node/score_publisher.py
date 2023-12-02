import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError


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
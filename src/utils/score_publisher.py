#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

class ScorePublisher:
    """!
    @brief      This class represents a publisher for the score.

    Attributes:
        score_pub (rospy.Publisher): Publisher for the score.
    """

    def __init__(self):
        """!
        @brief      Initializes the ScorePublisher object.

        This constructor sets up the publisher for the score.
        """
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)

        self.TEAM_NAME = "MAUPEPO"
        self.TEAM_PASSWORD = "password"

    def clue_publisher(self, clue, score):
        """!
        @brief      Publishes the score to the score_tracker topic.

        @param      clue (string): Clue to be published.
        @param      score (int): Clue number to be published.
        """
        self.score_pub.publish(f"{self.TEAM_NAME},{self.TEAM_PASSWORD},{score},{clue}")

    def start(self):
        """!
        @brief      Starts the timer for the simulation.
        """
        self.score_pub.publish(F"{self.TEAM_NAME},{self.TEAM_PASSWORD},0,")
        
    def stop(self):
        """!
        @brief      Stops the timer for the simulation.
        """
        self.score_pub.publish(F"{self.TEAM_NAME},{self.TEAM_PASSWORD},-1,")
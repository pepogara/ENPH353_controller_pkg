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

        self.all_clue_types = ['SIZE', 'VICTIM', 'CRIME', 'TIME', 'PLACE', 'MOTIVE', 'WEAPON', 'BANDIT']
        self.missing_clue_types = self.all_clue_types

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

    def levenshtein_distance(self, string1, string2):
        """!
        @brief      Calculates the Levenshtein distance between two strings.
        
        @param      string1 (string): The first string.
        @param      string2 (string): The second string.

        @return     The Levenshtein distance between the two strings.
        """
        matrix = [[0 for _ in range(len(string2) + 1)] for _ in range(len(string1) + 1)]

        for i in range(len(string1) + 1):
            matrix[i][0] = i
        for j in range(len(string2) + 1):
            matrix[0][j] = j

        for i in range(1, len(string1) + 1):
            for j in range(1, len(string2) + 1):
                if string1[i-1] == string2[j-1]:
                    cost = 0
                else:
                    cost = 1
                matrix[i][j] = min(matrix[i-1][j] + 1, matrix[i][j-1] + 1, matrix[i-1][j-1] + cost)

        return matrix[len(string1)][len(string2)]

    def most_similar_string(self, target):
        """!
        @brief      Finds the most similar string to the target string.
        
        @param      target (string): The target string.
        
        @return     The most similar string to the target string.
        """
        min_distance = float('inf')
        most_similar = None

        for string in self.missing_clue_types:
            distance = self.levenshtein_distance(target, string)
            if distance < min_distance:
                min_distance = distance
                most_similar = string

        return most_similar
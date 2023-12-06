import rospy
from std_msgs.msg import String
from typing import List, Union

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
        self.clue_types = {
            "road": self.all_clue_types[0:4],
            "off_road": self.all_clue_types[3:6],
            "mountain": self.all_clue_types[6:8],
            "hardcode": [self.all_clue_types[0], self.all_clue_types[1], self.all_clue_types[3], self.all_clue_types[6]],
            "all": self.all_clue_types
        }

    def clue_publisher(self, clue: str, score: int):
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
        self.score_pub.publish(f"{self.TEAM_NAME},{self.TEAM_PASSWORD},0,N/a")

    def stop(self):
        """!
        @brief      Stops the timer for the simulation.
        """
        self.score_pub.publish(f"{self.TEAM_NAME},{self.TEAM_PASSWORD},-1,N/a")

    def levenshtein_distance(self, string1: str, string2: str) -> int:
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

    def most_similar_string(self, target: str, state: str) -> Union[str, None]:
        """!
        @brief      Finds the most similar string to the target string.

        @param      target (string): The target string.
        @param      state (string): The state the robot is in.

        @return     The most similar string to the target string.
        """
        min_distance = float('inf')
        most_similar = None

        clue_list = self.clue_types.get(state, self.all_clue_types)

        for string in clue_list:
            distance = self.levenshtein_distance(target, string)
            if distance < min_distance:
                min_distance = distance
                most_similar = string

        return most_similar
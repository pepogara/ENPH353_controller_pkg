#! /usr/bin/env python3

import rospy
import sys

# from tensorflow import keras as ks

from utils.move_publisher import MovePublisher 
from utils.score_publisher import ScorePublisher
from utils.image_subscriber import ImageSubscriber
from utils.image_publisher import ImagePublisher

from states.road import RoadDrivingState

class StateMachine():
    """!
    @brief      State machine class for the robot.

    This class represents a state machine for the robot. It contains the logic for transitioning between states and executing the current state.
    """
    def __init__(self, first_state="road"):
        """!
        @brief      Constructs a new instance.

        @param      first_state (str): The next state to transition to after timer starts.
        """
        
        rospy.init_node('controller_node', anonymous=True)
        
        self.current_state = "start"

        self.next_state = first_state #TODO: make sure this thing works 

        # self.clue_board = ks.models.load_model("/home/fizzer/ros_ws/src/controller_pkg/nn_models/signNN_3.h5")

        self.move_pub = MovePublisher()
        self.score_pub = ScorePublisher()
        self.camera = ImageSubscriber()
        self.debug = ImagePublisher()
        self.road = RoadDrivingState(self)
        rospy.Timer(rospy.Duration(1), self.execute, oneshot=True)
        rospy.on_shutdown(self.on_shutdown)
    
    def transition_to(self, new_state):
        """!
        @brief      Transition to a new state.
        """
        # Perform state transition logic
        self.current_state = new_state
        
    def execute(self, event):
        """!
        @brief      Executes the given controller.

        @param      event  The event that triggered the execution.
        """
        while not rospy.is_shutdown():
            if self.current_state == "start":
                self.score_pub.start()

                self.transition_to(self.next_state)

            elif self.current_state == "idle":
                self.move_pub.stop_publisher()

            elif self.current_state == "road":

                self.road.execute()

            elif self.current_state == "respawn":
                pass

            elif self.current_state == "mountain":
                pass
    

    def spin(self):
        """!
        @brief      Function to keep the node running.
        """
        rospy.spin()

    def on_shutdown(self):
        """!
        @brief      Function to handle shutdown of the node.
        """
        self.score_pub.stop()
        self.move_pub.stop_publisher()
        rospy.loginfo("Shutting down node gracefully")


def main(args):
    """!
    @brief      Main function for the follow_line script.

    This function creates an instance of the controller package class and spins the ROS node.

    @param args (list): Command line arguments.

    """
    try:
        robot = StateMachine()
        robot.spin()

        #TODO: Add some command line arguments to start the robot in a specific state or smthn
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
        
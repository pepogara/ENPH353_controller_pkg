#! /usr/bin/env python3

import rospy
import sys

# from tensorflow import keras as ks

from utils.move_publisher import MovePublisher 
from utils.score_publisher import ScorePublisher
from utils.image_subscriber import ImageSubscriber
from utils.image_publisher import ImagePublisher

from states.road import RoadDrivingState
from states.off_road import OffRoadDrivingState
from states.mountain import MountainDrivingState
from states.hardcode import HardcodeDrivingState

class StateMachine():
    """!
    @brief      State machine class for the robot.

    This class represents a state machine for the robot. It contains the logic for 
    transitioning between states and executing the current state.
    """
    def __init__(self, first_state="road"):
        """!
        @brief      Constructs a new instance.

        @param      first_state (str): The next state to transition to after timer starts.
        """
        
        rospy.init_node('controller_node', anonymous=True)
        
        self.current_state = "start"

        self.next_state = first_state #TODO: make sure this thing works 

        self.move_pub = MovePublisher()
        self.score_pub = ScorePublisher()
        self.camera = ImageSubscriber()
        self.debug = ImagePublisher()

        self.road = RoadDrivingState(self)
        self.off_road = OffRoadDrivingState(self)
        self.mountain = MountainDrivingState(self)
        self.hardcode = HardcodeDrivingState(self)
        
        rospy.Timer(rospy.Duration(3), self.execute, oneshot=True)
        rospy.Timer(rospy.Duration(241), rospy.signal_shutdown, oneshot=True)

        self.backup = rospy.get_time()
        self.RESTART_TIME = 75

        self.start_time = rospy.get_time()

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

                rospy.sleep(3)

                self.transition_to(self.next_state)

            elif self.current_state == "idle":

                self.move_pub.stop_publisher()

            elif self.current_state == "road":
                
                self.road.execute()
                
                if self.road.done():
                    self.transition_to("off_road")

                if rospy.get_time() - self.backup > self.RESTART_TIME:
                    self.transition_to("not_finished_yet")

            elif self.current_state == "off_road":

                self.off_road.execute()

                if self.off_road.done():
                    self.transition_to("mountain_teleport")

                if rospy.get_time() - self.backup > self.RESTART_TIME:
                    self.transition_to("not_finished_yet")
            
            elif self.current_state == "hardcode":

                self.hardcode.execute()

                if self.hardcode.done():
                    rospy.signal_shutdown("Done")

            elif self.current_state == "not_finished_yet":
                self.hardcode.clue_num = 2
                self.transition_to("hardcode")
                
            elif self.current_state == "mountain_teleport":

                self.move_pub.teleport_to([-4.015, -2.299, 0.1, 0])
                
                self.transition_to("mountain")
                

            elif self.current_state == "mountain":

                self.mountain.execute()

                if self.mountain.done():
                    rospy.signal_shutdown("Done")
                    

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
        
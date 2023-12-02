#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

from controller_pkg.node.move_publisher import MovePublisher 
from controller_pkg.node.score_publisher import ScorePublisher
from controller_pkg.node.image_subscriber import ImageSubscriber

from controller_pkg.state_machine.states.road import RoadDrivingState
from controller_pkg.state_machine.states.idle import IdleState

import controller_pkg.node.image_treaiting as imgt

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

        self.move_pub = MovePublisher()
        self.score_pub = ScorePublisher()
        self.camera = ImageSubscriber()

        rospy.Timer(rospy.Duration(1), self.execute, oneshot=True)
    
    def transition_to(self, new_state):
        """!
        @brief      Transition to a new state.
        """
        # Perform state transition logic
        self.current_state = new_state
        
    def execute(self):
        """!
        @brief      Executes the given controller.
        """
        if self.current_state == "start":
            self.score_pub.start()

            self.transition_to(self.next_state)

        elif self.current_state == "idle":
            self.move_pub.stop_publisher()

        elif self.current_state == "road":

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

        elif self.current_state == "off_road":
            pass

        elif self.current_state == "mountain":
            pass

    

    def spin(self):
        """!
        @brief      Function to keep the node running.
        """
        rospy.spin()


def main(args):
    """!
    @brief      Main function for the follow_line script.

    This function creates an instance of the controller package class and spins the ROS node.

    @param args (list): Command line arguments.

    """
    try:
        robot = StateMachine(args[1])
        robot.spin()

        #TODO: Add some command line arguments to start the robot in a specific state or smthn
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
        
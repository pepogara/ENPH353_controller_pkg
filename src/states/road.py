#! /usr/bin/env python3

import numpy as np
import rospy
import sys
import cv2

import utils.image_treaiting as imgt

class RoadDrivingState:
    """!
    @brief      Class for road driving.
    """
    
    def __init__(self, state_machine):
        """!
        @brief      Constructs a new instance.
        """
        self.state_machine = state_machine
        self.current_substate = "follow_road"

    # def execute(self, controller):
    #     """!
    #     @brief      Executes the given controller.

    #     @param      controller  The controller
    #     """
    #     controller.road_driving()


    def transition_to_substate(self, substate):
        """!
        @brief      Transition to a new substate.
        """
        self.state_machine.transition_to(substate)
    

    def execute(self):
        """!
        @brief      Executes the given controller.
        """
        if self.current_substate == "follow_road":
            
            img = self.state_machine.camera.get_frame()

            hsv = imgt.HSV(img)
            hint, area = imgt.homography(hsv, img)
            if hint is not None:
                if (self.hint_found):
                    self.state_machine.debug.publish(self.past_hint)
                else:
                    if self.past_hint is not None:
                        # print(area)
                        # print(self.past_area)
                        # print("-----")
                        if area > self.past_area:
                            self.past_hint = hint
                            self.past_area = area
                        else:
                            # print(self.past_area)
                            self.hint_found = True
                    else:
                        self.past_hint = hint
                        self.past_area = area
                    # cv2.namedWindow("hint_frame", cv2.WINDOW_AUTOSIZE)
                    # cv2.imshow("hint_frame", hint)
                    # cv2.waitKey(1)
            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                self.state_machine.debug.publish(img)
                # cv2.destroyAllWindows()
            # Move the robot for 2 seconds
            # self.move_pub.move_publisher(0.0)

        elif self.current_substate == "pedestrian_crossing":
            # Call the execute method of sub-state 2
            pass

        elif self.current_substate == "truck_crossing":
            # Call the execute method of sub-state 3
            pass

        elif self.current_substate == "clue_board":
            # Call the execute method of sub-state 4
            pass
            
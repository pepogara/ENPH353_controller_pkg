#! /usr/bin/env python3

import numpy as np
import rospy
import sys
import cv2

from tensorflow import keras as ks

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
        self.past_hint = None
        self.past_area = 0
        self.hint_found = False

        self.model = ks.models.load_model("/home/fizzer/ros_ws/src/controller_pkg/nn_models/signNN_3.h5")


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
                        if area > self.past_area:
                            self.past_hint = hint
                            self.past_area = area
                        else:
                            self.hint_found = True

                            clue = self.clue_detect(self.past_hint).rstrip()

                            self.state_machine.score_pub.clue_publisher(clue, 1)

                    else:
                        self.past_hint = hint
                        self.past_area = area

            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                self.state_machine.debug.publish(img)
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
            
            self.state_machine.clue_board.predict(hint)

            self.transition_to_substate("follow_road")

    def clue_detect(self, hint):
        """!
        @brief      Detects the clue in the clue board.

        @param      hint  The hint image

        @return     The decoded clue.
        """

        characters = imgt.character_split(hint)
        decoded_chars = []

        for char in characters:
            prediction = self.model.predict(char)
            single_dig = imgt.onehotToStr(prediction)
            decoded_chars.append(single_dig)

        return ''.join(decoded_chars)

        
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

            # self.state_machine.debug.publish(img)

            hsv = imgt.HSV(img)
            hint = imgt.homography(hsv, img)

            if hint is not None:
                self.state_machine.debug.publish(hint)

                characters = imgt.character_split(hint)
                decoded_chars = []

                for char in characters:
                    # prediction = self.model.predict(char)
                    # single_dig = imgt.onehotToStr(prediction)
                    # decoded_chars.append(single_dig)
                    pass

                # word =  ''.join(decoded_chars)

                # print(decoded_chars)

            # Move the robot for 2 seconds
            # self.state_machine.move_pub.move_publisher(0.1)
            # self.transition_to_substate("clue_board")
            
            # Stop the robot movement
            # self.state_machine.move_pub.stop_publisher()

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
            
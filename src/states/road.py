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
        self.current_substate = "clue_board"
        self.past_hint = None
        self.past_area = 0
        self.hint_found = False

        self.model = ks.models.load_model("/home/fizzer/ros_ws/src/controller_pkg/nn_models/signNN_3.h5")

        self.integral = 0
        self.previous_error = 0

        self.Kp = 0.02
        self.Ki = 0.00
        self.Kd = 0.01


    def transition_to_substate(self, substate):
        """!
        @brief      Transition to a new substate.
        """
        self.current_substate = substate
    

    def execute(self):
        """!
        @brief      Executes the given controller.
        """

        img = self.state_machine.camera.get_frame()

        if self.current_substate == "follow_road":

            lines = imgt.HSV(img, "road")

            # For debugging
            self.state_machine.debug.publish(lines, "mono8")
            
            '''
            val = 0  # Current pixel value
            start = 0  # Index of the start of the road segment
            current_val = 255  # Initial pixel value assumed as non-road
            center = 0

            for index, val in enumerate(lines[-1]):
                if val != current_val:
                    if val == 0:
                        start = index
                        current_val = val
                    else:
                        if start == 0:  # Check if all values are 0
                            center = 0  # Set center to 0
                        else:
                            center = (start + index) // 2
                            # Calculate and return the center index
                        break
                if index == len(lines[-1]) - 1: # This occurs when the road goes off to the right
                    center = (start + lines.shape[1]) // 2
                else:
                    current_val = val

            error = lines.shape[1] // 2 - center

            self.integral += error
            derivative = error - self.previous_error
        
            output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

            self.previous_error = error

            '''
            # self.state_machine.move_pub.move_publisher(output)

            # self.transition_to_substate("clue_board")
            

        elif self.current_substate == "pedestrian_crossing":
            # Call the execute method of sub-state 2
            pass

        elif self.current_substate == "truck_crossing":
            # Call the execute method of sub-state 3
            pass

        elif self.current_substate == "clue_board":
            # Call the execute method of sub-state 4

            hsv = imgt.HSV(img, "clue")
            hint, area = imgt.homography(hsv, img)
            if hint is not None:
                if (self.hint_found):
                    # cv2.namedWindow("hint")
                    # cv2.imshow("hint", self.past_hint)
                    # cv2.waitKey(1)
                    self.state_machine.debug.publish(self.past_hint, "bgr8")
                    # pass
                else:
                    if self.past_hint is not None:
                        if area > self.past_area:
                            self.past_hint = hint
                            self.past_area = area
                        else:
                            self.hint_found = True

                            # clue = self.clue_detect(self.past_hint).rstrip()

                            # self.state_machine.score_pub.clue_publisher(clue, 1)

                    else:
                        self.past_hint = hint
                        self.past_area = area

            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                self.state_machine.debug.publish(img, "bgr8")

            # self.transition_to_substate("follow_road")

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

        
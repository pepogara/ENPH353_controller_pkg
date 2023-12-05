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

        self.clue_num = 1

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

            # lines = 

            # self.state_machine.debug.publish(img, "mono8")
            
            self.state_machine.move_pub.move_publisher(0)


            self.transition_to_substate("clue_board")

            if self.clue_num == 3:
                self.state_machine.move_pub.stop_publisher()
                self.transition_to_substate("teleport")
                self.clue_num = 4
                

            # self.state_machine.move_pub.move_publisher(output)


        elif self.current_substate == "teleport":
            # Call the execute method of sub-state 2
            self.state_machine.move_pub.teleport_to([0.50557, -0.027039, 0.1, 0, 0, 0, 0])
            rospy.sleep(0.5)
            self.transition_to_substate("follow_road")

        elif self.current_substate == "truck_crossing":
            # Call the execute method of sub-state 3
            pass

        elif self.current_substate == "clue_board":
            # Call the execute method of sub-state 4

            hsv = imgt.HSV(img, "clue")
            hint, area = imgt.homography(hsv, img)
            if hint is not None:
                if (self.hint_found):
                    self.state_machine.debug.publish(self.past_hint, "bgr8")
                    pass
                else:
                    if self.past_hint is not None:
                        if area > self.past_area:
                            self.past_hint = hint
                            self.past_area = area
                        else:
                            self.hint_found = True
                            clue = self.clue_detect(self.past_hint)
                            self.state_machine.score_pub.clue_publisher(clue, self.clue_num)
                            self.clue_num += 1

                    else:
                        self.past_hint = hint
                        self.past_area = area

            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                self.state_machine.debug.publish(img, "bgr8")

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

        word = ''.join(decoded_chars).rstrip()

        return word

        
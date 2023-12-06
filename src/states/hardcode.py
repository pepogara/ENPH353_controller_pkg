#! /usr/bin/env python3

import numpy as np
import rospy
import sys
import cv2

from tensorflow import keras as ks
import tensorflow as tf

import utils.image_treating as imgt

class HardcodeDrivingState:
    """!
    @brief      Class for road driving.
    """
    
    def __init__(self, state_machine):
        """!
        @brief      Constructs a new instance.
        """
        self.state_machine = state_machine
        self.current_substate = "hardcode"
        self.past_hint = None
        self.past_area = 0
        self.hint_found = False

        self.read_clues =[1]

        self.clue_num = 0

        self.last_clue = False

        self.model = ks.models.load_model("/home/fizzer/ros_ws/src/controller_pkg/nn_models/signNN_4.h5")

        self.integral = 0
        self.previous_error = 0

        self.Kp = 0.02
        self.Ki = 0.00
        self.Kd = 0.015

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

        if self.current_substate == "hardcode":

            if self.clue_num == 0:
                self.state_machine.move_pub.move_publisher(0)
                self.transition_to_substate("clue_board")

            elif self.clue_num == 1:
                self.state_machine.move_pub.move_publisher(0)
                self.transition_to_substate("clue_board")

            elif self.clue_num == 2:
                self.state_machine.move_pub.stop_publisher()
                self.transition_to_substate("teleport1")
                self.clue_num = 3

            elif self.clue_num == 3:
                self.state_machine.move_pub.move_publisher(0, -0.5)
                self.transition_to_substate("clue_board")

            elif self.clue_num == 4:
                self.state_machine.move_pub.stop_publisher()
                self.transition_to_substate("teleport2")
                self.clue_num = 6

            elif self.clue_num == 6:
                self.state_machine.move_pub.move_publisher(0, -0.5)
                self.transition_to_substate("clue_board")
            
            else:
                pass
                

        elif self.current_substate == "teleport1":
            self.state_machine.move_pub.teleport_to([0.50557, -0.027039, 0.1, 90])
            rospy.sleep(0.5)
            self.transition_to_substate("hardcode")
        
        elif self.current_substate == "teleport2":
            self.state_machine.move_pub.teleport_to([-4.015, -2.299, 0.1, 0])
            rospy.sleep(0.5)
            self.transition_to_substate("hardcode")

        elif self.current_substate == "clue_board":

            hsv = imgt.HSV(img, "clue")
            hint, area = imgt.homography(hsv, img)
            if hint is not None:
                if (self.hint_found):
                    # self.state_machine.debug.publish(self.past_hint, "bgr8")
                    pass
                else:
                    if self.past_hint is not None:
                        if area > self.past_area:
                            self.past_hint = hint
                            self.past_area = area
                        else:
                            self.hint_found = True
                            clue_type = self.type_detect(self.past_hint)

                            if clue_type not in self.read_clues:
                                clue = self.clue_detect(self.past_hint)
                                self.read_clues.append(clue_type)
                                self.clue_num += 1
                                self.state_machine.score_pub.clue_publisher(clue, clue_type)

                            if clue_type == 7:
                                self.last_clue = True
                                self.state_machine.move_pub.stop_publisher()
                                rospy.sleep(0.5)

                    else:
                        self.past_hint = hint
                        self.past_area = area

            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                # self.state_machine.debug.publish(img, "bgr8")

            self.transition_to_substate("hardcode")

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
    
    def type_detect(self, hint):
        """!
        @brief      Detects the type of clue in the clue board.

        @param      hint  The hint image

        @return     The index of the type of clue
        """

        characters = imgt.character_split(hint, False)
        decoded_chars = []

        for char in characters:
            prediction = self.model.predict(char)
            single_dig = imgt.onehotToStr(prediction)
            decoded_chars.append(single_dig)


        word = ''.join(decoded_chars).rstrip()

        similar = self.state_machine.score_pub.most_similar_string(word, "hardcode")

        index = self.state_machine.score_pub.all_clue_types.index(similar) + 1

        return index
    
    def done(self):
        """!
        @brief      Checks if the state is done.

        @return     True if the state is done, False otherwise.
        """
        return self.last_clue
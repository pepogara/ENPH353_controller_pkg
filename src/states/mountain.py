import numpy as np
import rospy
import sys
import cv2

from tensorflow import keras as ks

import utils.image_treating as imgt


class MountainDrivingState:
    """!
    @brief      Class for road driving.
    """
    
    def __init__(self, state_machine):
        """!
        @brief      Constructs a new instance.
        """
        self.state_machine = state_machine
        self.current_substate = "backward_hard"
        self.next_substate = "backward_hard"
        self.past_hint = None
        self.past_area = 0
        self.hint_found = False

        self.first_clue = True
        self.read_clues = []

        self.first_clue = False

        self.last_clue = False

        self.model = ks.models.load_model("/home/fizzer/ros_ws/src/controller_pkg/nn_models/signNN_4.h5")

        self.integral = 0
        self.previous_error = 0

        self.Kp = 0.025
        self.Ki = 0.00
        self.Kd = 0.02


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

        if self.current_substate == "backward_hard":
            self.state_machine.move_pub.move_publisher(0, -0.2)
            self.transition_to_substate("clue_board")

            if self.first_clue:
                self.next_substate = "pid"
                self.tunnel_time = rospy.get_time()
                self.transition_to_substate("forward_hard")

        elif self.current_substate == "forward_hard":
            self.state_machine.move_pub.move_publisher(0, 0.4)

            if rospy.get_time() - 6.5 > self.tunnel_time:
                self.pid_time = rospy.get_time()
                self.transition_to_substate("pid")
                
        elif self.current_substate == "turn_hard":
            self.state_machine.move_pub.move_publisher(0.6, 0.3)

            if rospy.get_time() - self.top_time > 1:
                self.transition_to_substate("pid")

        elif self.current_substate == "pid":
            if rospy.get_time() - self.pid_time > 7:
                lines = imgt.HSV(img, "mountain_light", True)
            else:
                lines = imgt.HSV(img, "mountain_dark", True)

            self.state_machine.debug.publish(lines, "mono8")

            center = self.state_machine.move_pub.center_of_road(lines, True)
            
            if center is not None:
                error = img.shape[1] // 2 - center[0]
            else:
                error = 0

            self.integral += error
            derivative = error - self.previous_error
        
            output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

            self.previous_error = error

            self.state_machine.move_pub.move_publisher(output, 0.5)

            self.transition_to_substate("clue_board")

            if rospy.get_time() - self.pid_time > 14:
                self.top_time = rospy.get_time()
                self.transition_to_substate("turn_hard")

        elif self.current_substate == "clue_board":
            # Call the execute method of sub-state 4

            hsv = imgt.HSV(img, "clue")
            hint, area = imgt.homography(hsv, img)
            if hint is not None:
                self.state_machine.debug.publish(hint, "bgr8")
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
                                self.state_machine.score_pub.clue_publisher(clue, clue_type)

                                if clue_type == 7: # to check if the first clue on the road is read
                                    self.first_clue = True
                                    # self.last_clue = True

                                if clue_type == 8: # to check if the last clue on the road is read
                                    self.last_clue = True
                                    self.state_machine.move_pub.stop_publisher()

                    else:
                        self.past_hint = hint
                        self.past_area = area

            else:
                self.hint_found = False
                self.past_hint = None
                self.past_area = 0
                # self.state_machine.debug.publish(img, "bgr8")

            self.transition_to_substate(self.next_substate)

        elif self.current_substate == "debug":
            imgt.HSV(img, "mountain", False)

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

        similar = self.state_machine.score_pub.most_similar_string(word, "mountain")

        index = self.state_machine.score_pub.all_clue_types.index(similar) + 1

        return index
        
    def done(self):
        """!
        @brief      Checks if the state is done.

        @return     True if the state is done, False otherwise.
        """
        return self.last_clue
import numpy as np

class RoadDrivingState:
    """!
    @brief      Class for road driving.
    """
    
    def __init__(self, state_machine):
        """!
        @brief      Constructs a new instance.
        """
        self.state_machine = state_machine
        self.current_substate = None

    def execute(self, controller):
        """!
        @brief      Executes the given controller.

        @param      controller  The controller
        """
        controller.road_driving()


    def transition_to_substate(self, substate):
        """!
        @brief      Transition to a new substate.
        """
        self.state_machine.transition_to(substate)
    

    def execute(self):
        """!
        @brief      Executes the given controller.
        """
        if self.current_sub_state == "follow_road":
            # Call the execute method of sub-state 1
            pass

        elif self.current_sub_state == "pedestrian_crossing":
            # Call the execute method of sub-state 2
            pass

        elif self.current_sub_state == "truck_crossing":
            # Call the execute method of sub-state 3
            pass

        elif self.current_sub_state == "clue_board":
            # Call the execute method of sub-state 4
            pass
            
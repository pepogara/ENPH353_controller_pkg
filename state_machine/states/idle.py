import numpy as np

class IdleState:
    """!
    @brief      Class for idle.
    """
    
    def __init__(self, state_machine):
        """!
        @brief      Constructs a new instance.
        """
        self.state_machine = state_machine
        self.current_substate = None


    def transition_to_substate(self, substate):
        """!
        @brief      Transition to a new substate.
        """
        self.state_machine.transition_to(substate)
    

    def execute(self):
        """!
        @brief      Executes the given controller.
        """
        # i need to be image proccessing and not moving
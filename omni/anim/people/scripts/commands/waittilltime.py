# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .base_command import Command
import carb

class WaitTillTime(Command):
    """
    Command class to wait until a specified time relative to sim start time.
    """
    def __init__(self, character, command, navigation_manager, current_time):
        #print("WaitTillTime __init__")
        super().__init__(character, command, navigation_manager)
        #self.sim_start_real_time = sim_start_real_time
        # if current time upon init of the command is already past wait_end_time, we skip the command
        if len(command) > 1:
            #wait_end_time = int(command[1])
            wait_end_time = float(command[1])
            #print("WaitTillTime: ")
            #print(current_time)
            #print(wait_end_time)
            if current_time > wait_end_time:
                #print("current_time > wait_end_time")        
                carb.log_info("[Warning] WaitTillTime: Current time (" + str(current_time) + ") is beyound wait time (" 
                    + str(wait_end_time) + ")" )
                self.duration = 0.1
            else:
                #print("current_time < wait_end_time")
                # wait_end_time is relative to 0 (i.e. sim_start)
                # time.time() is relative to epoch time
                # sim_start_real_time is the epoch time when the simulation started
                # so we need to adjust the wait_end_time to be relative to epoch time
                self.duration = wait_end_time - current_time   
            
    def setup(self):
        super().setup()
        self.character.set_variable("Action", "None")

    def update(self, dt):
        #print("WaitTillTime update")
        #if( dt == 0.0 ):
        #    print("GotoNew:Warning dt is 0.0 and skip update ")
        #    return False
    
        return super().update(dt)
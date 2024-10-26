# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .base_command import Command
import carb

class GoToNew(Command):
    """
    Command class to go to a location/locations.
    """
    def __init__(self, character, command, navigation_manager, current_time):
        #print("GoToNew __init__")
        super().__init__(character, command, navigation_manager)
        
        self.max_goto_duration = 1200
        command_start_time = float(self.command[1])

        # if current time is already past command start time, we skip the command
        # sim_start_real_time is the epoch time when the simulation started
        # command start time is relative to 0 (i.e. sim_start)
        # time.time() is relative to epoch time
        # so we need to adjust the command start time to be relative to epoch time

        #if self.command_start_time + self.sim_start_real_time <= time.time():
        #print("GoToNew: init ")
        #print(current_time)
        #print(command_start_time + self.max_goto_duration)
        if command_start_time + self.max_goto_duration * 0.1 < current_time :
            #print("command_start_time + self.max_goto_duration < current_time")
            carb.log_info("[Warning] GotoNew: Current time (" + str(current_time) + ") is beyound command time (" 
            + str(command_start_time) + ") + max duration (" + str(self.max_goto_duration) + ")")
            self.max_goto_duration = 0.1

    def setup(self):
        #print("GoToNew setup")
        super().setup()
        self.character.set_variable("Action", "Walk")
        self.navigation_manager.generate_goto_path(self.command[2:])

    def execute(self, dt):
        #print("GotoNew:execute")
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def update(self, dt):
        #print("GotoNew:update")
        #if( dt == 0.0 ):
        #    print("GotoNew:Warning dt is 0.0 and skip update ")
        #    return False

        self.time_elapsed += dt
        #print(dt)
        #print(self.time_elapsed)
        #print(self.max_goto_duration)
        if self.walk(dt) or (self.max_goto_duration < 1200 and self.time_elapsed > self.max_goto_duration):
            return self.exit_command()
        #if self.time_elapsed > self.max_goto_duration:
        #    print("GotoNew:force to exit")
        #    print(dt)
        #    print(self.time_elapsed)
        #    print(self.max_goto_duration)
        #    self.character.set_variable("Action", "None")
        #    self.navigation_manager.set_path_target_rot(None)
        #    self.navigation_manager.clean_path_targets()
        #    self.character.set_variable("PathPoints", self.navigation_manager.get_path_points())
        #    #return self.exit_command()

        #if self.walk(dt):
        #    print("walk return True")
        #    return self.exit_command()
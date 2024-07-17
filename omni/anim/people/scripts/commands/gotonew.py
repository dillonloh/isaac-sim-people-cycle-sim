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
        super().__init__(character, command, navigation_manager)
        
        self.max_goto_duration = 15
        command_start_time = float(self.command[1])

        # if current time is already past command start time, we skip the command
        # sim_start_real_time is the epoch time when the simulation started
        # command start time is relative to 0 (i.e. sim_start)
        # time.time() is relative to epoch time
        # so we need to adjust the command start time to be relative to epoch time

        #if self.command_start_time + self.sim_start_real_time <= time.time():
        print("GoToNew: ")
        print(current_time)
        print(command_start_time + self.max_goto_duration)
        if command_start_time + self.max_goto_duration < current_time :
            print("command_start_time + self.max_goto_duration < current_time")
            carb.log_info("[Warning] GotoNew: Current time (" + str(current_time) + ") is beyound command time (" 
            + str(command_start_time) + ") + max duration (" + str(self.max_goto_duration) + ")")
            self.max_goto_duration = 0 # command will exit immediately and effectively be skipped

    def setup(self):
        super().setup()
        self.character.set_variable("Action", "Walk")
        self.navigation_manager.generate_goto_path(self.command[2:])

    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def update(self, dt):
        self.time_elapsed += dt
        if self.walk(dt) or self.time_elapsed > self.max_goto_duration:
            return self.exit_command()
        
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .base_command import Command

class GoToLoop(Command):
    """
    Command class to pace back and forth between two destinations
    """
    def __init__(self, character, command, navigation_manager):
        super().__init__(character, command, navigation_manager)
        self._current_destination = 1

    def setup(self):
        super().setup()
        self.character.set_variable("Action", "Walk")
        self.navigation_manager.generate_goto_path(self.command[1:5]) # index 1 to 5 is first destination (note the index ignores the character name at start)

    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def update(self, dt):
        self.time_elapsed += dt
        if self.walk(dt):
            # return self.exit_command()
            if self._current_destination == 1:
                print(f"Going to destination 2: {self.command[5:9]}")
                self._current_destination = 2
                self.navigation_manager.generate_goto_path(self.command[5:9])
            else:
                print(f"Going to destination 1: {self.command[1:5]}")
                self._current_destination = 1
                self.navigation_manager.generate_goto_path(self.command[1:5]) # index 6 to 9 is second destination 
        return False
        
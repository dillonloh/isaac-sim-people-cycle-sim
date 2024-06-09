# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .robot_command import Robot_Command

class Robot_GoTo(Robot_Command):
    """
    Command class to go to a location/locations.
    """
    def __init__(self,robot, controller, command, navigation_manager):
        super().__init__(robot, controller, command, navigation_manager)
        
    def setup(self):
        super().setup()
        self.navigation_manager.generate_goto_path(self.command[1:])

    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def update(self, dt):
        self.time_elapsed += dt
        if self.move(dt):
            return self.exit_command()
        
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations
import numpy as np

class Robot_Command:
    """
    Base class for command, provides default implementation for setup, update, execute and exit_command.
    This follows the same structure as found in the character Command base class
    Also implements the move function which moves a robot to a location based on the target location set in NavigationManager.
    """
    def __init__(self, robot, controller, command, navigation_manager):
        self.robot = robot
        self.controller = controller
        self.command = command
        self.navigation_manager = navigation_manager
        self.time_elapsed = 0
        self.is_setup = False
        self.duration = 5
        
    def setup(self):
        self.time_elapsed = 0
        self.is_setup = True

    def exit_command(self):
        self.is_setup = False
        return True

    def update(self, dt):
        self.time_elapsed += dt
        if self.time_elapsed > self.duration:
            return self.exit_command()

    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def move(self, dt):
        if self.navigation_manager.destination_reached():
            self.navigation_manager.set_path_points(None)
            self.navigation_manager.clean_path_targets()
            return True

        self.navigation_manager.update_current_path_point()

        path_points = self.navigation_manager.get_path_points()
        position, orientation = self.robot.get_world_pose()

        if path_points:
            target_position = np.array([path_points[0][0], path_points[0][1]])
            self.robot.apply_wheel_actions(self.controller.forward(start_position=position,
                                                            start_orientation=orientation,
                                                            goal_position= target_position))
        # After reaching the target position feed the robot with its current position to make it stop
        else:
            self.navigation_manager.clean_path_targets()
            self.robot.apply_wheel_actions(self.controller.forward(start_position=position,
                                                            start_orientation=orientation,
                                                            goal_position= position))
        return
        
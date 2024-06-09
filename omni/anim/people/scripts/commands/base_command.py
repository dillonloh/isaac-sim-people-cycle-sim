# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations
from ..utils import Utils

class Command:
    """
    Base class for command, provides default implementation for setup, update, execute and exit_command.

    Also implements the walk function which moves a character to a location based on the target location set in NavigationManager.
    """
    def __init__(self, character, command, navigation_manager):
        self.character = character
        self.command = command
        self.navigation_manager = navigation_manager
        self.time_elapsed = 0
        self.is_setup = False
        self.desired_walk_speed = 0
        self.actual_walk_speed = 0
        self.rotation_time = 0
        self.rotation_time_threshold = 2
        self.set_rotation = None
        self.char_start_rot = None
        self.duration = 5
        
        
    def setup(self):
        self.time_elapsed = 0
        self.desired_walk_speed = 0
        self.actual_walk_speed = 0
        self.is_setup = True


    def exit_command(self):
        self.is_setup = False
        self.character.set_variable("Action", "None")
        return True


    def update(self, dt):
        self.time_elapsed += dt
        if self.time_elapsed > self.duration:
            return self.exit_command()
    

    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)


    def rotate(self, dt):
        if self.set_rotation == False:
            rot_diff = self.navigation_manager.calculate_rotation_diff()
            self.rotation_time_threshold = (rot_diff/90)*Utils.CONFIG["SecondPerNightyDegreeTurn"]
            self.set_rotation = True

        trans, rot= Utils.get_character_transform(self.character)
        target_rot = self.navigation_manager.get_path_target_rot()
        
        if Utils.dot4(rot, target_rot) < 0.0:
           target_rot = Utils.scale4(target_rot, -1.0)
        
        if self.rotation_time > self.rotation_time_threshold:
            self.char_start_rot = None
            self.rotation_time = 0 
            self.character.set_world_transform(trans,target_rot)
            return True

        if self.char_start_rot == None:
            self.char_start_rot = rot

        # Calculate fraction of rotation at each delta time and set that rotation.
        self.rotation_time += dt
        time_fraction_of_completion = min(self.rotation_time/self.rotation_time_threshold, 1.0)
        rotation_fraction = Utils.nlerp4(self.char_start_rot, target_rot, time_fraction_of_completion)
        self.character.set_world_transform(trans,rotation_fraction)


    def walk(self, dt):
        if self.navigation_manager.destination_reached():
            self.desired_walk_speed = 0.0
            if self.actual_walk_speed < 0.001:
                self.character.set_variable("Action", "None")
                self.navigation_manager.set_path_points(None)
                if self.navigation_manager.get_path_target_rot() is not None:   
                    if self.rotate(dt):
                        self.character.set_variable("Action", "None")
                        self.navigation_manager.set_path_target_rot(None)
                        self.navigation_manager.clean_path_targets()
                        return True
                    return False
                else:
                    self.character.set_variable("Action", "None")
                    self.navigation_manager.clean_path_targets()
                    return True
        else:
            self.set_rotation = False
            self.desired_walk_speed = 1.0

        self.character.set_variable("Action", "Walk")
        self.navigation_manager.update_path()
        self.character.set_variable("PathPoints", self.navigation_manager.get_path_points())
        
        # Blends walking animation when starting or stopping.
        max_change = dt / Utils.CONFIG["WalkBlendTime"]
        delta_walk = Utils.cap(self.desired_walk_speed - self.actual_walk_speed, -1 * max_change, max_change)
        self.actual_walk_speed = Utils.cap(self.actual_walk_speed + delta_walk, 0.0, 1.0)
        self.character.set_variable("Walk", self.actual_walk_speed)
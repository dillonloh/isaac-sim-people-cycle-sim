# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations
import carb
import logging
import threading

#### ADDED CODE ####
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Pose

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)-8s %(message)s', datefmt='%a, %d %b %Y %H:%M:%S', filename='/home/dillon/test.log', filemode='w')

class GlobalCharacterPositionManager:
    """Global class which stores current and predicted positions of all characters and moving objects."""

    __instance: GlobalCharacterPositionManager = None

    def __init__(self):
        if self.__instance is not None:
            print("Only one instance of GlobalCharacterPositionManager is allowed")
            raise RuntimeError("Only one instance of GlobalCharacterPositionManager is allowed")
        self._character_positions = {}
        self._character_future_positions = {}
        self._character_radius = {}
        GlobalCharacterPositionManager.__instance = self

    def destroy(self):
        GlobalCharacterPositionManager.__instance = None

    def __del__(self):
        self.destroy()

    @classmethod
    def get_instance(cls) -> GlobalCharacterPositionManager:
        if cls.__instance is None:
            print("Creating new instance")
            GlobalCharacterPositionManager()
        
        # print("Returning GlobalCharacterPositionManager instance")
        return cls.__instance


    def set_character_radius(self, char_prim_path, radius):
        self._character_radius[char_prim_path] = radius

    def get_character_radius(self, char_prim_path):
        return self._character_radius[char_prim_path]

    def set_character_current_pos(self, char_prim_path, pos):
        self._character_positions[char_prim_path] = pos

    def set_character_future_pos(self, char_prim_path, pos):
        self._character_future_positions[char_prim_path] = pos

    def get_character_current_pos(self, char_prim_path):
        return self._character_positions[char_prim_path]

    def get_character_future_pos(self, char_prim_path):
        return self._character_future_positions[char_prim_path]

    def get_all_character_pos(self):
        return self._character_positions.values()

    def get_all_character_future_pos(self):
        return self._character_future_positions.values()

    def get_all_managed_characters(self):
        return self._character_positions.keys()
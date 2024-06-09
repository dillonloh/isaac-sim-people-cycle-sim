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

        #### ADDED CODE ####
        # Initialize ROS
        
        #check if ROS is already initialized
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node("character_position_manager")
        self._publishers = {}

        self._spin_thread = threading.Thread(target=self._spin_node)
        self._spin_thread.daemon = True
        self._spin_thread.start()

        # Start the thread for printing character positions
        # self._printing_thread = threading.Thread(target=self._print_positions)
        # self._printing_thread.daemon = True 
        # self._printing_thread.start()

        self._ros_node.create_publisher(Pose, "_female_adult_police_01", 10)

    #### ADDED CODE (DILLON)####
    def _spin_node(self):
        """Method to spin the node in a separate thread."""
        try:
            rclpy.spin(self._ros_node)
        except Exception as e:
            pass
        
    #### ADDED CODE (DILLON)####
    def _print_positions(self):
        """Continuously prints and publishes the current character positions."""
        print("START THREAD")
        while True:
            try:
                if rclpy.ok():
                    for character_name, position in self._character_positions.items():
                        # Format the output
                        formatted_position = f"{character_name}: Position x={position.x}, y={position.y}, z={position.z}"
                        # print(formatted_position)

                        # Check if publisher exists for this character, if not create one
                        if character_name not in self._publishers:
                            topic_name = character_name.replace('/', '_').split("ManRoot_")[-1]
                            print(f"Creating publisher for {character_name} on topic {topic_name}")
                            self._publishers[character_name] = self._ros_node.create_publisher(Pose, topic_name, 10)
                        
                        # Create the ROS message and publish it
                        msg = Pose()
                        msg.position.x = position.x
                        msg.position.y = position.y
                        msg.position.z = position.z

                        self._publishers[character_name].publish(msg)

                else:
                    break

                threading.Event().wait(0.5)
            except Exception as e:
                print(f"Error in printing thread: {e}")
                break

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
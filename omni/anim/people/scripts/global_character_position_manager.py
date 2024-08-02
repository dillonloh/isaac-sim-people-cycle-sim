# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations

#### ADDED CODE ####
import asyncio
import rospy
from geometry_msgs.msg import Pose

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
        try:
            rospy.init_node("hello", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")
        
        self._publishers = {}
        asyncio.ensure_future(self._print_positions())

    async def my_task(self):
        print("my_task")
        from std_msgs.msg import String
        pub = rospy.Publisher("/hello_topic", String, queue_size=10)

        for frame in range(10):
            pub.publish("hello world " + str(frame))
            await asyncio.sleep(1.0)
        pub.unregister()
        pub = None
        
    #### ADDED CODE (DILLON)####
    async def _print_positions(self):
        """publishes the current character positions."""
        print("START THREAD publishes the current character positions ")
        while not rospy.is_shutdown():
            try:
                for character_name, position in self._character_positions.items():
                    # Format the output
                    formatted_position = f"{character_name}: Position x={position.x}, y={position.y}, z={position.z}"
                    print(formatted_position)

                    # Check if publisher exists for this character, if not create one
                    if character_name not in self._publishers:
                        #topic_name = character_name.replace('/', '_').split("ManRoot_")[-1]
                        topic_name = character_name.split("/")[3]
                        print(f"Creating publisher for {character_name} on topic {topic_name}")
                        self._publishers[character_name] = rospy.Publisher(topic_name, Pose, queue_size=10)
                        
                    print(character_name.split("/")[3])
                    # Create the ROS message and publish it
                    msg = Pose()
                    msg.position.x = position.x
                    msg.position.y = position.y
                    msg.position.z = position.z

                    self._publishers[character_name].publish(msg)
                
                await asyncio.sleep(0.5)

            except Exception as e:
                print(f"Error in printing thread: {e}")
                break

        for k,v in self._publishers.items():
            v.unregister()
            v = None
        self._publishers = {}

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
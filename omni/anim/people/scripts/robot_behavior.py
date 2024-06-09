# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
import asyncio
import carb
import numpy as np
from omni.kit.scripting import BehaviorScript
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core import World
from omni.anim.people import PeopleSettings
from omni.replicator.character.core.stage_util import RobotUtil
from .robot_navigation_manager import RobotNavigationManager
from .commands.robot_goto import *
from .commands.robot_idle import *
import importlib

"""
    Control the Isaac Nova Carter Robot
    Given a command file, the robot will execute the commands sequentially
"""
class RobotBehavior(BehaviorScript):
    def on_init(self):
        # Loading Isaac.Core.World, which is created and managed by the Simulation Manager
        if World.instance() is None:
            RobotUtil.set_robot_world()
        self.world = World.instance()

        # Path: prim path, Name: the name to be used in the command file
        self._carter_path = str(self.prim_path)
        self._carter_name = self._carter_path.split("/")[-1]
        self._action_name = self._carter_name + "_actions"
        self.commands = []

        # Read settings from anim.people
        self.setting =  carb.settings.get_settings()
        self.command_path = self.setting.get(PeopleSettings.ROBOT_COMMAND_FILE_PATH)
        self.navmeshEnabled = self.setting.get(PeopleSettings.NAVMESH_ENABLED)
        self.avoidanceOn = self.setting.get(PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED)
        self.navigation_manager = None 
        self.current_command = None

        # Register the carter robot and its controller
        self._carter = None
        if self.world.scene.get_object(self._carter_path) is None:
            self._carter = WheeledRobot(prim_path=self._carter_path,
                                        name=self._carter_path,
                                        wheel_dof_names = ["joint_wheel_left","joint_wheel_right"],
                                        create_robot=False)
            self.world.scene.add(self._carter)
        else:
            err_msg = "Robot " + self._carter_name + " already created and added to World"
            carb.log_error(err_msg)
        self._controller = WheelBasePoseController(name="cool_controller_"+ self._carter_name,
                                                    open_loop_wheel_controller = DifferentialController(name="refine_control_"+self._carter_name, wheel_radius=0.152, wheel_base=0.413),
                                                    is_holonomic=False)

        self.paused = False     # Used to handle when the user pauses the play

    # Commands and Navigation settings may be changed every time before playing, hence need to be set before play
    def setup_for_play(self):
        self.command_path = self.setting.get(PeopleSettings.ROBOT_COMMAND_FILE_PATH)
        self.commands = self.get_simulation_commands()
        self.navmeshEnabled = self.setting.get(PeopleSettings.NAVMESH_ENABLED)
        self.avoidanceOn = self.setting.get(PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED)
        self.navigation_manager= RobotNavigationManager(self._carter_path, self._carter, self.navmeshEnabled, self.avoidanceOn)           
        return

    # Equivalent to on_update(), step_size is fixed and is determined by world: physics_dt
    def send_robot_actions(self, step_size):
        if self.navigation_manager is None:
            err_msg = self.name + "has no navigation manager. No action will be executed."
            carb.log_error(err_msg)
            return
        if self.commands:
            self.execute_command(self.commands, step_size)
        # Publish the position of the robot after it's been updated
        if self.avoidanceOn:
            # Set the radius of the robot to be avoided by the characters
            self.navigation_manager.publish_robot_position(step_size, 0.5)

    # Clean the navigation manager, robot registry, and physics callback when removing the robot from the scene
    def on_destroy(self):
        if self.navigation_manager is not None:
            self.navigation_manager.destroy()
            self.navigation_manager = None
        if self.world.scene.get_object(self._carter_path) is not None:
            self.world.scene.remove_object(self._carter_path, True)
        if self.world.physics_callback_exists(self._action_name):
            self.world.remove_physics_callback(self._action_name)
    
    def on_play(self):
        # If paused rather than stopped, no need to reset the robot
        if self.paused:
            self.paused = False
            return
        self.setup_for_play()
        # Add physics callback and initialize the robot, must be called in async and everytime before it plays
        async def init_robot():
            self.world.add_physics_callback(self._action_name, callback_fn=self.send_robot_actions)
            self._carter.initialize()
            return 
        asyncio.ensure_future(init_robot())
    
    # Must clean the robot physics callback and current command when stop playing
    def on_stop(self):
        if self.navigation_manager is not None:
            self.navigation_manager = None
        self.current_command = None
        self.world.remove_physics_callback(self._action_name)
    
    def on_pause(self):
       self.paused = True 

    def execute_command(self, commands, delta_time):
        """
        Executes commands in commands list in sequence. Removes a command once completed.

        :param list[list] commands: list of commands.
        :param float delta_time: time elapsed since last execution.
        """
        if self.current_command is None:
            if commands:
                self.current_command = self.get_command(commands[0])
            else:
                return

        if self.current_command.execute(delta_time):
            commands.pop(0)
            self.current_command = None

    """
    The following three methods are related to command loading
    Recommended Action: make a base class for all agents and put those functions in the base class
    """
    def read_commands_from_file(self):
        """
        Reads commands from file pointed by self.command_path. Creates a Queue using queue manager if a queue is specified.
        :return: List of commands.
        :rtype: python list
        """
        result, version, context = omni.client.read_file(self.command_path)
        if result != omni.client.Result.OK:
            return []

        cmd_lines = memoryview(context).tobytes().decode("utf-8").splitlines()
        return cmd_lines

    def get_command(self, command):
        """
        Returns an instance of a command object based on the command.

        :param list[str] command: list of strings describing the command.
        :return: instance of a command object.
        :rtype: python object
        """
        if command[0] == "GoTo":
            return Robot_GoTo(self._carter, self._controller, command, self.navigation_manager)
        if command[0] == "Idle":
            return Robot_Idle(self._carter, self._controller, command, self.navigation_manager)
        else:
            module_str = ".commands.{}".format(command[0].lower(), package = None)
            try:
                custom_class = getattr(importlib.import_module(module_str, package=__package__), command[0])
            except (ImportError, AttributeError) as error:
                carb.log_error("Module or Class for the command do not exist. Check the command again.")
            return custom_class(self._carter, self._controller, command, self.navigation_manager)
    
    def get_simulation_commands(self):
        cmd_lines = []
        # Get commands from cmd_file
        cmd_lines.extend(self.read_commands_from_file())
        commands = []
        for cmd_line in cmd_lines:
            if not cmd_line:
                continue
            words = cmd_line.strip().split(' ')
            if words[0] == self._carter_name:
                command = []
                for word in words[1:]:
                    command.append(word)
                commands.append(command)
            if words[0][0] == "#":
                continue
        return commands
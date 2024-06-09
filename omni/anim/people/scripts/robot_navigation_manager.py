# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations
import math
import carb
import omni.anim.navigation.core as Navigation_core
from .utils import Utils
from .global_character_position_manager import GlobalCharacterPositionManager

class RobotNavigationManager:
    """
    Manages navigation for a robot, such as generating static obstacle free paths and publishing current and future positions for the robot so that characters can avoid it
    Follows the same structure as found in the navigation_manager.py
    Robot will only publish its position for the characters to avoid but it will not avoid the characters
    """
    def __init__(self,robot_name, robot, navmesh_enabled, dynamic_avoidance_enabled = True):
        self.navigation_interface = Navigation_core.acquire_interface()
        self.robot_manager = GlobalCharacterPositionManager.get_instance()
        self.robot = robot
        self.robot_name = robot_name
        self.navmesh_enabled = navmesh_enabled
        self.dynamic_avoidance_enabled = dynamic_avoidance_enabled
        self.collision_list = []
        self.positions_over_time = []
        self.delta_time_list = []
        self.path_points = []
        self.path_targets = []
        self.path_final_target_rot = None 

    def destroy(self):
        self.navigation_interface = None
        self.robot_manager = None
        self.robot_name = None
        self.robot = None
        self.navmesh_enabled = None
        self.collision_list = None
        self.positions_over_time = None
        self.delta_time_list = None
        self.path_points = None
        self.path_targets = None
        self.path_final_target_rot = None 

    def set_path_points(self, path_points):
        self.path_points = path_points

    def get_path_points(self):
        return self.path_points

    def update_current_path_point(self):
        if self.path_points:
            current_next_step = self.path_points[0]
            # Here you can set the minimum distance to decide whether the robot has reached a position
            reach_point = self.check_proximity_to_point(current_next_step, 0.25)
            if reach_point:
                self.path_points.pop(0)

    def check_proximity_to_point(self, point, proximity_dist):
        cart_pos,_ = self.robot.get_world_pose()
        # get the point on the same xz plane as the obstacle
        cart_pos_carb = carb.Float3(cart_pos[0], cart_pos[1], cart_pos[2])
        point_on_character_plane = carb.Float3(point[0], point[1], point[2])
        distance = Utils.dist3(cart_pos_carb, point_on_character_plane)
        return distance <= proximity_dist

    def clean_path_targets(self):
        self.path_targets = []
        self.path_final_target_rot = None

    def destination_reached(self):
        if not self.path_targets:
            return True

    def publish_robot_position(self, delta_time, radius):
        if delta_time == 0:
            return 

        cart_pos, _ = self.robot.get_world_pose()
        cart_pos_xy = carb.Float3(cart_pos[0], cart_pos[1], 0)
        num_frames = 10 

        # Store the positions and dts of the num_frames last frames
        if len(self.positions_over_time) < num_frames:
            self.positions_over_time.append(cart_pos_xy)
            self.delta_time_list.append(delta_time)
        else:
            self.positions_over_time.pop(0)
            self.positions_over_time.append(cart_pos_xy)
            self.delta_time_list.pop(0)
            self.delta_time_list.append(delta_time)

        # the estimated velocity (per sec) of the current obstacle
        if len(self.positions_over_time) > 0 and len(self.delta_time_list) > 0:
            self.velocity_vec = Utils.scale3(Utils.sub3(self.positions_over_time[-1], self.positions_over_time[0]), 1 / sum(self.delta_time_list))
        else:
            self.velocity_vec = carb.Float3(0,0,0)
        self.robot_manager.set_character_current_pos(self.robot_name, cart_pos_xy)
        # Here you can set the number of frames to buffer for predicting future positions
        self.robot_manager.set_character_future_pos(self.robot_name, Utils.add3(cart_pos_xy, Utils.scale3(self.velocity_vec, 10)))
        self.robot_manager.set_character_radius(self.robot_name, radius)

    def generate_path(self, coords):
        self.path_targets = []
        prev_point = coords[0]
        path = []
        if not self.navmesh_enabled:
            path.append(prev_point)
        for point in coords[1:]:
            if self.navmesh_enabled:
                generated_path = self.navigation_interface.query_navmesh_path(prev_point, point)
                if generated_path is None:
                    carb.log_error("There is no valid path between point position : " + str(prev_point) + " and " + "position : " + str(point))
                    return 
                points = generated_path.get_points()
                path.extend(points)
                prev_point = point
            else:
                path.append(point)
            self.path_targets.append(point)
        self.path_points = path

    def generate_goto_path(self, coords):
        if len(coords) < 3 or len(coords) % 2 != 1:
            raise ValueError("Invalid coordinate list for path generation. Coordinate list must be a sequence of x,y,z with the last cooridnate also specifying the ending rotation.")
        cart_pos, _ = self.robot.get_world_pose()
        start_pos = carb.Float3(cart_pos[0], cart_pos[1], 0)
        self.path_targets = []
        path = []

        for i in range(0, len(coords)//3):
            curr_point = carb.Float3(float(coords[i*3]), float(coords[i*3+1]), float(coords[i*3+2]))
            path.append(curr_point)
            
        path.insert(0, start_pos)      
        self.generate_path(path)

# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .base_command import Command
from ..utils import Utils
from pxr import Gf, UsdGeom
import carb
import omni.usd

class Sit(Command):
    """
    Command class that implements the sit command.
    """
    def __init__(self, character, command, navigation_manager):
        super().__init__(character, command, navigation_manager)
        self.stage = omni.usd.get_context().get_stage()
        self.seat_prim = self.stage.GetPrimAtPath(command[1])
        if len(command) > 2:
            self.duration = float(command[2])
        self.sit_time = 0
        self.interaction_pos = None
        self.current_action = None
        
        
    def setup(self):
        super().setup()
        self.generate_path_to_seat()
        self.set_interaction_position()
        self.current_action = "walk"
        self._char_lerp_t = 0
        self.stand_animation_time = 0


    def get_prim_transform(self, prim):
        gf_transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        gf_transform.Orthonormalize()
        gf_translation = gf_transform.ExtractTranslation()
        gf_rot = gf_transform.ExtractRotation()
        return gf_translation, gf_rot


    def generate_path_to_seat(self):
        start_point = Utils.get_character_pos(self.character)
        prim_trans, prim_rot = self.get_prim_transform(self.seat_prim)
        offset = None
        if not self.seat_prim.HasAttribute("walk_to_offset"):
            offset = Gf.Vec3d(0,0,0)
            carb.log_warn("Please attach walk_to_offset value to seat prim")
        else:
            walk_to_offset = self.seat_prim.GetAttribute("walk_to_offset")
            offset = walk_to_offset.Get()
            if offset is None:
                carb.log_warn("walk_to_offset is null. Default offset Gf.Vec3d(0,0,0) is used")
                offset = Gf.Vec3d(0,0,0)

        # convert offset according to chair's rotation 
        offset = offset * Gf.Matrix3d(prim_rot)
        end_point = carb.Float3(prim_trans[0]+offset[0], prim_trans[1]+offset[1], prim_trans[2]+offset[2])
        gf_quat = prim_rot.GetQuat()
        gf_i = gf_quat.GetImaginary()
        end_rot = carb.Float4(gf_i[0], gf_i[1], gf_i[2], gf_quat.GetReal())
        self.navigation_manager.generate_path([start_point, end_point], end_rot)
    

    def set_interaction_position(self):
        prim_trans, prim_rot = self.get_prim_transform(self.seat_prim)
        anim_offset = None
        if not self.seat_prim.HasAttribute("interact_offset"):
            anim_offset = Gf.Vec3d(0,0,0)
            carb.log_warn("Please attach interact_offset value to seat prim")
        else:    
            interact_offset = self.seat_prim.GetAttribute("interact_offset")
            anim_offset = interact_offset.Get()
            if anim_offset is None:
                carb.log_warn("interact_offset is null. Default offset Gf.Vec3d(0,0,0) is used")
                anim_offset = Gf.Vec3d(0,0,0)
        # convert offset according to chair's rotation 
        anim_offset = anim_offset* Gf.Matrix3d(prim_rot)
        self.interaction_pos = carb.Float3(prim_trans[0]+anim_offset[0], prim_trans[1]+anim_offset[1], prim_trans[2]+anim_offset[2])


    def update(self, dt):
        if self.current_action == "walk" or self.current_action == None:
            if self.walk(dt):
                self.current_action = "sit"
                self._char_start_pos, self._char_start_rot = Utils.get_character_transform(self.character)
        
        elif self.current_action == "sit":
            # Start to play sit animation 
            # At the same time adjust players's tranlatation to fit the seat
            self._char_lerp_t = min(self._char_lerp_t + dt, 1.0)
            lerp_pos = Utils.lerp3(self._char_start_pos, self.interaction_pos, self._char_lerp_t)
            self.character.set_world_transform(lerp_pos, self._char_start_rot)
            self.character.set_variable("Action", "Sit")
            self.sit_time += dt
            if self.sit_time > self.duration:
                self.character.set_variable("Action", "None")
                self._char_lerp_t = 0.0
                self.current_action = "stand"

        elif self.current_action == "stand":
            if self.stand_animation_time < 1.5:
                # adjust character's position while play "stand" animation
                self._char_lerp_t = min(self._char_lerp_t + dt, 1.0)
                lerp_pos = Utils.lerp3(self.interaction_pos, self._char_start_pos, self._char_lerp_t)
                current_pos, current_rot = Utils.get_character_transform(self.character)
                self.character.set_world_transform(lerp_pos, current_rot)
                self.stand_animation_time += dt

            if self.stand_animation_time > 1.5:
                # set character's position to position before the sit animation, enter the idle stage
                current_pos, current_rot = Utils.get_character_transform(self.character)
                self.character.set_world_transform(self._char_start_pos, current_rot)
                return self.exit_command()
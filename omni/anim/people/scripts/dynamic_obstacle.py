# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.kit.scripting import BehaviorScript
from .global_character_position_manager import GlobalCharacterPositionManager
from .utils import Utils
import omni.usd

class DynamicObstacle(BehaviorScript):
    """
    Class representing a moving obstacle. When attached to a moving obstacle, the script publishes the objects current and future locations to the GlobalCharacterPositionManager, allowing characters to avoid it.

    The script should be attached to small and slow moving objects like robots.
    """
    def on_init(self):
        self._usd_context = omni.usd.get_context()
        self.prim_obj = None
        self.stage_obj = None
        if self._usd_context is not None:
            self.init_movable_object()
    

    def init_movable_object(self):
        if self.stage_obj is None:
            self.stage_obj = omni.usd.get_context().get_stage()
        if self.stage_obj is None:
            return 

        self.positions_over_time = []
        self.delta_time_list = []
        self.object_path = self._prim_path
        self.prim_obj = self.stage_obj.GetPrimAtPath(self.object_path)
        self.character_manager = GlobalCharacterPositionManager.get_instance()
    

    def clean(self):
        self.object_path = None
        self.positions_over_time = []
        self.delta_time_list = []
        self.prim_obj = None
        self.character_manager = None


    def publish_object_position(self, delta_time):
        t = Utils.get_prim_pos(self.stage_obj, self.object_path, self.prim_obj)
        n = 10
        if len(self.positions_over_time) < n:
            self.positions_over_time.append(t)
            self.delta_time_list.append(delta_time)
        else:
            self.positions_over_time.pop(0)
            self.positions_over_time.append(t)
            self.delta_time_list.pop(0)
            self.delta_time_list.append(delta_time)
        
        self.velocity_vec = Utils.scale3(Utils.sub3(self.positions_over_time[-1], self.positions_over_time[0]), 1 / sum(self.delta_time_list))
        # the current position of all the characters
        self.character_manager.set_character_current_pos(self.object_path, t)
        # the predicted position in one second of all the characters
        radius = Utils.get_object_radius(self.object_path)
        self.character_manager.set_character_future_pos(self.object_path, Utils.add3(t, Utils.scale3(self.velocity_vec, radius/0.5)))
        self.character_manager.set_character_radius(self.object_path, radius)


    def on_destroy(self):
        self.clean()


    def on_update(self, current_time: float, delta_time: float):
        if self.prim_obj is None:
            self.init_movable_object()
        self.publish_object_position(delta_time)


    def on_stop(self):
        self.on_destroy()


    def on_play(self):
        self.init_movable_object()

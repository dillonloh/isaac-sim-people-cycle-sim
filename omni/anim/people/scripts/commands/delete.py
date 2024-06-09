# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import omni.usd
import omni.isaac.core.utils.prims as prims_utils
from .base_command import Command

class Delete(Command):
    def __init__(self, character, prim_path, command, navigation_manager):
        super().__init__(character, command, navigation_manager)
        self.prim_path = prim_path  # Passed explicitly

    def execute(self, dt):
        print("Delete command")
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        print(prim)
        print("TEST DELETE")
        # prims_utils.delete_prim(self.prim_path)  #doesnt work because of ancestral prim https://forums.developer.nvidia.com/t/how-to-remove-ancestral-prim/270308
        if prim.IsValid():
            stage.RemovePrim(self.prim_path)
            print(f"Deleted prim at path: {self.prim_path}")
            carb.log_info(f"Deleted prim at path: {self.prim_path}")
            return True
        else:
            print(f"No valid prim found at path: {self.prim_path}")
            carb.log_error(f"No valid prim found at path: {self.prim_path}")
            return False
        return True
    def update(self, dt):
        return super().update(dt)
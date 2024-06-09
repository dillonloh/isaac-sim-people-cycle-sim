# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import omni.ui
import omni.ext

import omni.kit.app
import asyncio
from .ui import BehaviorControlUI

class Main(omni.ext.IExt):
    def on_startup(self, ext_id):
        carb.log_info("[omni.anim.people] startup")
        self._behavior_ui = None
        async def build():
            await omni.kit.app.get_app().next_update_async()
            self._behavior_ui = BehaviorControlUI(self)
        asyncio.ensure_future(build())


    def on_shutdown(self):
        carb.log_info("[omni.anim.people] shutdown")

        # Shutdown Behavior Control UI
        if self._behavior_ui:
            self._behavior_ui.shutdown()
            self._behavior_ui = None

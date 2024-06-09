# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni import ui
import __main__
from ..collapsable_widget import CollapsableWidget
from .navmesh_control_widget import NavMeshControlWidget
from .avoidance_control_widget import AvoidanceControlWidget

class NavigationSettingsPanel(CollapsableWidget):
    def __init__(self, ext_ui_instance):
        super().__init__("Navigation Settings")
        self.title_v_stack = None
        self.content_v_stack = None
        self._recast_control = NavMeshControlWidget(ext_ui_instance)
        self._avoidance_control = AvoidanceControlWidget(ext_ui_instance)
        

    def shutdown(self):
        super().shutdown()    

        if self._recast_control:
            self._recast_control.shutdown()
            self._recast_control = None

        if self._avoidance_control:
            self._avoidance_control.shutdown()
            self._avoidance_control = None
        self.title_v_stack = None
        self.content_v_stack = None

    def _build_content(self):
        with ui.HStack(spacing=30):
            self.title_v_stack = ui.VStack(spacing = 30)
            self.content_v_stack = ui.VStack(spacing = 30)
        self._recast_control._build_content(self.title_v_stack,self.content_v_stack)
        self._avoidance_control._build_content(self.title_v_stack,self.content_v_stack)

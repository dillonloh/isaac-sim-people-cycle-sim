# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import omni.kit.app
import omni.kit.ui
import omni.ui as ui

from . import ui_components

class BehaviorControlUI:
    def __init__(self, ext_instance):
        self._ext_instance = ext_instance
        
        self._window = ui_components.ToggleableWindow(
            title="People Simulation",
            menu_prefix="Window",
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
            width=ui_components.base_widget_setting.WINDOW_WIDTH,
            height=ui_components.base_widget_setting.WINDOW_HEIGHT / 2
        )
        self._NavigationSettingsPanel  = ui_components.NavigationSettingsPanel(self)
        self._CommandSettingsPanel  = ui_components.CommandSettingsPanel(self)
        self._CharacterSettingsPanel = ui_components.CharacterSettingsPanel(self)
        self._build()
        
    def shutdown(self):

        if self._NavigationSettingsPanel:
            self._NavigationSettingsPanel.shutdown()
            self._NavigationSettingsPanel = None
        
        if self._CommandSettingsPanel:
            self._CommandSettingsPanel.shutdown()
            self._CommandSettingsPanel = None
        
        if self._CharacterSettingsPanel:
            self._CharacterSettingsPanel.shutdown()
            self._CharacterSettingsPanel = None
        
        self._ext_instance = None
        if self._window is not None:
            self._window.shutdown()
            self._window.destroy()
            self._window = None
            
    def _build(self):
        self._window.frame.clear()
        with self._window.frame:
            self._build_content()

    def _build_content(self):
        with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
            with ui.VStack():
                self._CommandSettingsPanel.build()
                self._NavigationSettingsPanel.build()
                self._CharacterSettingsPanel.build()
                
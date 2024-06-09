# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni import ui
import carb
import __main__
from omni.kit.widget.settings import SettingsWidgetBuilder, SettingType, create_setting_widget
from omni.anim.people import PeopleSettings

class AvoidanceControlWidget:
    def __init__(self, ext_ui_instance):
        self._ext_ui_instance = ext_ui_instance
        self._checkbox = None
        self._checkbox_label = None

    def shutdown(self):
        self._checkbox = None
        self._checkbox_label = None

    def _build_content(self, title_stack, content_stack):
        with title_stack:
            self._checkbox_label = ui.Label("Dynamic Obstacle Avoidance")
        with content_stack:
            with ui.HStack():
                ui.Spacer(width = 5)
                widget, model = create_setting_widget(
                    PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED, SettingType.BOOL, range_from = 0, range_to = 0, speed = 1, hard_range = False
                )
                model.set_value(True)
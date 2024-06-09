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

class CommandTextWidget:

    textbox_commands = ""
    def __init__(self, ext_ui_instance):
        self._ext_ui_instance = ext_ui_instance
        self._textbox= None
        self._textbox_label = None
        self._defualt_command = '# Sample Command Guide\n#\n# Spawn Tom\n# Spawn Jerry 10 10 0 0\n# Tom GoTo 10 10 0 _\n# Jerry GoTo 0 0 0 _'

    def shutdown(self):
        self._textbox= None
        self._textbox_label = None

    def _build_content(self, title_stack, content_stack):
        with content_stack:
            with ui.HStack(spacing=30):
                self._textbox_label = ui.Label("Command Text Box")
                self._textbox = ui.StringField(height = 100, multiline = True)
                self._textbox.model.add_value_changed_fn(self._on_changed)
                self._textbox.model.set_value(self._defualt_command)


    def _on_changed(self, model):
        CommandTextWidget.textbox_commands = model.get_value_as_string()

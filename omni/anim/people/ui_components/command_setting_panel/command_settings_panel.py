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
from .command_selection import CommandWidget
from .command_text_widget import CommandTextWidget
from .character_setup_widget import CharacterSetupWidget


class CommandSettingsPanel(CollapsableWidget):
    def __init__(self, ext_ui_instance):
        super().__init__("Command Panel")
        self.title_v_stack = None
        self.content_v_stack = None
        self._command_control =  CommandWidget(ext_ui_instance)
        self._command_text_widget = CommandTextWidget(ext_ui_instance)
        self._character_setup_widget = CharacterSetupWidget(ext_ui_instance)

    def shutdown(self):
        super().shutdown()
        if self._command_control:
            self._command_control.shutdown()
            self._command_control = None      
        
        if self._command_text_widget:
            self._command_text_widget.shutdown()
            self._command_text_widget = None

        if self._character_setup_widget:
            self._character_setup_widget.shutdown()
            self._character_setup_widget = None
        
        self.title_v_stack = None
        self.content_v_stack = None

    def _build_content(self):
        self.content_v_stack = ui.VStack(spacing = 40)
        self._command_text_widget._build_content(None, self.content_v_stack)
        self._command_control._build_content(None,self.content_v_stack)
        self._character_setup_widget._build_content(None, self.content_v_stack)
        
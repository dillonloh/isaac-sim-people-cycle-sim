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
from .random_clothes_widget import RandomClotheWidget

class CharacterSettingsPanel(CollapsableWidget):
    def __init__(self, ext_ui_instance):
        super().__init__("Character Settings")
        self.title_v_stack = None
        self.content_v_stack = None
        self._random_clothes_widget = RandomClotheWidget(ext_ui_instance)
        

    def shutdown(self):
        super().shutdown()    

        if self._random_clothes_widget:
            self._random_clothes_widget.shutdown()
            self._random_clothes_widgetl = None
        
        self.title_v_stack = None
        self.content_v_stack = None

    def _build_content(self):
        with ui.HStack(spacing=30):
            self.title_v_stack = ui.VStack(spacing = 100)
            self.content_v_stack = ui.VStack(spacing = 100)
        self._random_clothes_widget._build_content(self.title_v_stack,self.content_v_stack)

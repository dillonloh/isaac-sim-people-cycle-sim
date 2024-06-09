# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni import ui


ELEM_MARGIN = 4
BORDER_RADIUS = 4

CollapsableFrameStyle = {
    "CollapsableFrame": {
        "background_color": 0xFF333333,
        "secondary_color": 0xFF333333,
        "color": 0xFFFFFFFF,
        "border_radius": BORDER_RADIUS,
        "border_color": 0x0,
        "border_width": 0,
        "font_size": 14,
        "padding": ELEM_MARGIN * 2,
        "margin_width": ELEM_MARGIN,
        "margin_height": ELEM_MARGIN,
    },
    "CollapsableFrame:hovered": {"secondary_color": 0xFF3C3C3C},
    "CollapsableFrame:pressed": {"secondary_color": 0xFF333333},
    "Button": {"margin_height": 0, "margin_width": ELEM_MARGIN, "border_radius": BORDER_RADIUS},
    "Button:selected": {"background_color": 0xFF666666},
    "Slider": {"margin_height": 0, "margin_width": ELEM_MARGIN, "border_radius": BORDER_RADIUS},
    "ComboBox": {"margin_height": 0, "margin_width": ELEM_MARGIN, "border_radius": BORDER_RADIUS},
    "Label": {"margin_height": 0, "margin_width": ELEM_MARGIN},
    "Rectangle": {
        "background_color": 0xFF333333,
    },
}


class CollapsableWidget:
    def __init__(self, title, default_collapsed=False):
        self._frame = None
        self._collapsable_main = None
        self._title = title
        self._default_collapsed = default_collapsed

    def shutdown(self):
        self._collapsable_main = None
        self._frame = None

    def build(self):
        self._frame = ui.VStack(height=0, spacing=0)
        with self._frame:
            self._build_content_collapsed()

    def _build_content_collapsed(self):
        if self._collapsable_main is not None:
            collapsed = self._collapsable_main.collapsed
        else:
            collapsed = self._default_collapsed
        self._collapsable_main = ui.CollapsableFrame(
            self._title, collapsed=collapsed, style=CollapsableFrameStyle
        )
        with self._collapsable_main:
            self._build_content()  # Should be implemened in derived class

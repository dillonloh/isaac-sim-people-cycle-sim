# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.ui as ui
import omni.kit.ui
import omni.stageupdate
import omni.usd

WINDOW_WIDTH = 540
WINDOW_HEIGHT = 780
LEFT_COLUMN_WIDTH_IN_PERCENT = 30
LEFT_COLUMN_WIDTH_IN_PERCENT_WIDE = 80
FRAME_SPACING = 5
RIGHT_SPACING = 12
GLYPH_CODE_UP = omni.kit.ui.get_custom_glyph_code("${glyphs}/arrow_up.svg")
GLYPH_CODE_DOWN = omni.kit.ui.get_custom_glyph_code("${glyphs}/arrow_down.svg")
GLYPH_CODE_SETTINGS = omni.kit.ui.get_custom_glyph_code("${glyphs}/cog.svg")
GLYPH_CODE_FOLDER = omni.kit.ui.get_custom_glyph_code("${glyphs}/folder.svg")
GLYPH_CODE_FOLDER_OPEN = omni.kit.ui.get_custom_glyph_code("${glyphs}/folder_open.svg")
GLYPH_CODE_JUMP_TO = omni.kit.ui.get_custom_glyph_code("${glyphs}/share.svg")
GLYPH_CODE_LINK = omni.kit.ui.get_custom_glyph_code("${glyphs}/link.svg")
GLYPH_CODE_UNLINK = omni.kit.ui.get_custom_glyph_code("${glyphs}/unlink.svg")
TUNE_ARROW_SIZE = 10
CHECKBOX_BACKGROUND_COLOR = 0xFF9E9E9E
CHECKBOX_COLOR = 0xFF23211F


WINDOW_DARK_STYLE = {
    # "Button.Image::res_link": {"image_url": MovieCaptureIcons().get("link")},
    "Button::res_link": {"background_color": 0x0},
    # "Button.Image::res_unlink": {"image_url": MovieCaptureIcons().get("link_dark")},
    "Button::res_unlink": {"background_color": 0x0},
    "Rectangle::input_with_spinners": {"background_color": 0x0},
    "CheckBox::green_check": {"font_size": 12, "background_color": CHECKBOX_BACKGROUND_COLOR, "color": CHECKBOX_COLOR, "border_radius": 1.5},
}

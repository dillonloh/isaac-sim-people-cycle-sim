# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni import ui
import __main__
import os
from .. import base_widget_setting
from omni.kit.window.filepicker import FilePickerDialog
from omni.kit.widget.filebrowser import FileBrowserItem
from omni.kit.widget.settings import SettingsWidgetBuilder, SettingType, create_setting_widget
from pxr import Sdf
from omni.anim.people import PeopleSettings
import carb
import omni.stageupdate

class CommandWidget:
    def __init__(self, ext_ui_instance):
        self._filepicker = None
        self._filepicker_selected_folder = ""
        self._ext_ui_instance = ext_ui_instance
        self.cmd_path_token = "cmd_path"
        self._command_path = None
        self._stage_update = omni.stageupdate.get_stage_update_interface()
        self._stage_subscription = self._stage_update.create_stage_update_node(
            "CharacterControlPanel",
            on_attach_fn=self.on_attach
        )
        self._settings = carb.settings.acquire_settings_interface()
        self.command_path_setting_listener = None

    def on_attach(self, stage_id, meters_per_unit):
        self.refresh_setting()

    def refresh_setting(self):
        cmd_path = self.get_command_path()
        if self._command_path is not None:
            self._command_path.set_value(cmd_path)

    def shutdown(self):
        self._filepicker = None
        self._filepicker_selected_folder = None
        self.cmd_path_token = None
        self._command_path = None
        self._stage_update = None
        self._stage_subscription = None
        if self.command_path_setting_listener:
            carb.settings.get_settings().unsubscribe_to_change_events(self.command_path_setting_listener)
            self.command_path_setting_listener = None

    def get_command_path(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            # "stage is not ready"
            return ""
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim is None or (not world_prim.IsValid()):
            # root prim is null or invalid root_prm
            return ""
        if not world_prim.HasAttribute(self.cmd_path_token):
            # root prim do not have path attribute
            return ""
        cmd_path = world_prim.GetAttribute(self.cmd_path_token).Get()
        if cmd_path is None or len(str(cmd_path)) == 0:
            # The cmd path prim is an empty string 
            return ""        
        str_cmd_path = str(cmd_path)
        return str_cmd_path

    def _build_content(self, title_stack, content_stack):
        with content_stack:
            with ui.HStack(spacing=30):
                self._path_label = ui.Label("Command File Path")
                with ui.HStack(spacing=5):
                    default_dir = self.get_command_path()
                    widget, model = create_setting_widget(
                    PeopleSettings.COMMAND_FILE_PATH, SettingType.STRING, range_from = 0, range_to = 0, speed = 1, hard_range = False)
                    self._command_path = model
                    self._ui_kit_change_path = ui.Label(
                        f"{base_widget_setting.GLYPH_CODE_FOLDER}",
                        mouse_pressed_fn=lambda x, y, b, _: self._on_path_change_clicked(),
                    )
                    self.command_path_setting_listener = self._settings.subscribe_to_node_change_events(
			        PeopleSettings.COMMAND_FILE_PATH, lambda *_: self._on_path_changed())
                    self._command_path.set_value(default_dir)
    
    
    def _on_path_changed(self):
        path = carb.settings.get_settings().get(PeopleSettings.COMMAND_FILE_PATH)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        world_prim = stage.GetPrimAtPath("/World")
        cmd_attr = None
        if (world_prim is None) or (not world_prim.IsValid()):
            # raise ValueError("There is no /World prim inside stage")
            return
        
        if not world_prim.HasAttribute(self.cmd_path_token):
            cmd_attr = world_prim.CreateAttribute(self.cmd_path_token, Sdf.ValueTypeNames.String)
        else:
            cmd_attr = world_prim.GetAttribute(self.cmd_path_token)
        cmd_attr.Set(path)
    
    
    def _on_path_change_clicked(self):
        if self._filepicker is None:
            self._filepicker = FilePickerDialog(
                "Select Command File",
                # show_only_collections="my-computer",
                apply_button_label="Select",
                item_filter_fn=lambda item: self._on_filepicker_filter_item(item),
                file_extension_options=[("*.txt","Plain Text File")],
                selection_changed_fn=lambda items: self._on_filepicker_selection_change(items),
                click_apply_handler=lambda filename, dirname: self._on_dir_pick(self._filepicker, filename, dirname),
            )
        self._filepicker.set_filebar_label_name("Command File Name: ")
        self._filepicker.refresh_current_directory()
        self._filepicker.show(self._command_path.get_value_as_string())
    
    def _on_filepicker_filter_item(self, item: FileBrowserItem) -> bool:
        if not item or item.is_folder:
            return True
        return os.path.splitext(item.path)[1] == ".txt"
        
    def _on_filepicker_selection_change(self, items: [FileBrowserItem] = []):
        last_item = items[-1]
        # self._filepicker.set_filename(last_item.name)
        self._filepicker_selected_folder = last_item.path


    def _on_dir_pick(self, dialog: FilePickerDialog, filename: str, dirname: str):
        dialog.hide()
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if (world_prim is None) or (not world_prim.IsValid()):
            raise ValueError("There is no /World prim inside stage")
        self._command_path.set_value(self._filepicker_selected_folder)
        
    
    
    
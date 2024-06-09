# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni import ui
import __main__
import omni.usd
import random
from pxr import Sdf, Usd, UsdGeom, Gf
import carb
import omni.kit.commands

class RandomClotheWidget:
    def __init__(self, ext_ui_instance):
        self._ext_ui_instance = ext_ui_instance
        self._randomize_clothes_button = None
        self._randomize_skin_button = None

    def shutdown(self):
        self._randomize_clothes_button = None
        self._randomize_skin_button = None

    def _build_content(self, title_stack, content_stack):
        with title_stack:
            ui.Label("Character Appearance")
        with content_stack:
            with ui.HStack():
                self._randomize_clothes_button = ui.Button("Randomize Clothing")
                self._randomize_skin_button = ui.Button("Randomize Skin Tone")
                self._randomize_clothes_button.set_clicked_fn(self._randomize_clothing_on_clicked) 
                self._randomize_skin_button.set_clicked_fn(self._randomize_skin_on_clicked)

    def _randomize_clothing_on_clicked(self):
        self.stage = omni.usd.get_context().get_stage()
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "SkelRoot" and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible:
                man_root_prim = prim.GetParent()
                character_root_prim = man_root_prim.GetParent()
                self.randomize_character_clothes(character_root_prim)

    def _randomize_skin_on_clicked(self):
        self.stage = omni.usd.get_context().get_stage()
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "SkelRoot" and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible:
                man_root_prim = prim.GetParent()
                character_root_prim = man_root_prim.GetParent()
                self.randomize_character_skin(character_root_prim)

    def randomize_character_clothes(self, character_root_prim):
        looks_folder = character_root_prim.GetChildren()[0]
        for cloth in looks_folder.GetChildren():
            if not "skin" in str(cloth.GetPath()):
                self.random_cloth_color(cloth)


    def randomize_character_skin(self, character_root_prim):
        looks_folder = character_root_prim.GetChildren()[0]
        for cloth in looks_folder.GetChildren():
            if "skin" in str(cloth.GetPath()):
                self.random_skin_color(cloth)

    def random_skin_color(self, cloth_prim):
        material_prim_path = cloth_prim.GetPath()
        property_path = str(material_prim_path)+"/Shader.inputs:diffuse_tint"
        random_color = random.uniform(0.5,2)

        args = {
            "prop_path": Sdf.Path(property_path),
            "value":(random_color, random_color, random_color),
            "prev": None,
            "timecode": Usd.TimeCode.Default(),
            "type_to_create_if_not_exist": Sdf.ValueTypeNames.Color3f
        }
        omni.kit.commands.execute("ChangePropertyCommand", **args)

        shader = cloth_prim.GetChildren()[0]
        color_tint = shader.GetAttribute("inputs:diffuse_tint")
        if color_tint is None:
            carb.log_warn("No color tint on this prim")

    def random_cloth_color(self, cloth_prim):
        material_prim_path = cloth_prim.GetPath()
        property_path = str(material_prim_path)+"/Shader.inputs:diffuse_tint"
        random_r = random.uniform(0,2)
        random_g = random.uniform(0,2)
        random_b = random.uniform(0,2)

        args = {
            "prop_path": Sdf.Path(property_path),
            "value":(random_r, random_g, random_b),
            "prev": None,
            "timecode": Usd.TimeCode.Default(),
            "type_to_create_if_not_exist": Sdf.ValueTypeNames.Color3f
        }
        omni.kit.commands.execute("ChangePropertyCommand", **args)

        shader = cloth_prim.GetChildren()[0]
        color_tint = shader.GetAttribute("inputs:diffuse_tint")
        if color_tint is None:
            carb.log_warn("No color tint on this prim.")
        
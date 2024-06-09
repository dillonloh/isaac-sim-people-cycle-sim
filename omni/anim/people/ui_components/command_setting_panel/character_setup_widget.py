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
from pxr import Sdf, Gf, UsdGeom
import carb
import omni.kit.commands
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils import prims
import omni.anim.graph.core as ag
from .command_text_widget import CommandTextWidget
from omni.anim.people import PeopleSettings
import random
import omni.stageupdate


class CharacterSetupWidget:

    def __init__(self, ext_ui_instance):
        self._ext_ui_instance = ext_ui_instance
        self._character_setup_button = None
        self.cmd_path_token = "cmd_path"
        self.default_biped_usd = "Biped_Setup"
        self.default_biped_asset_name = "biped_demo"
        # Root path of character assets
        self.assets_root_path = ""
        # List to track characters that are available for Spawning.
        self.available_character_list = []
        self.spawned_agents_list = []

    def shutdown(self):
        self.spawned_agents_list = None
        self.available_character_list = None

    def _build_content(self, title_stack, content_stack):
        with content_stack:
            with ui.HStack(spacing=50):
                ui.Spacer()
                self._character_load_button = ui.Button("Load Characters", width = 200, height =10, alignment = ui.Alignment.H_CENTER)
                self._character_setup_button = ui.Button("Setup Characters", width = 200, height =10, alignment = ui.Alignment.H_CENTER)
                self._character_load_button.set_clicked_fn(self._load_characters_on_clicked)
                self._character_setup_button.set_clicked_fn(self._setup_characters_on_clicked)
                ui.Spacer()

    def _load_characters_on_clicked(self):
        self.stage = omni.usd.get_context().get_stage()
        world_prim = self.stage.GetPrimAtPath("/World")
        cmd_file_path = world_prim.GetAttribute(self.cmd_path_token).Get()

        # Commands need to be provided either via the cmd_file or the textbox.
        if not cmd_file_path and not CommandTextWidget.textbox_commands:
            carb.log_info("No people simulation commands provided. Provide a command file or enter commands in the cmd textbox.")
            return
        
        cmd_lines = []
        if cmd_file_path:
            result, version, context = omni.client.read_file(cmd_file_path)
            if result != omni.client.Result.OK:
                raise ValueError("Unable to read specified command file path.")
            cmd_lines.extend(memoryview(context).tobytes().decode("utf-8").splitlines())

        if CommandTextWidget.textbox_commands:
            cmd_lines.extend(CommandTextWidget.textbox_commands.splitlines())
        
        self._init_characters(cmd_lines)


    def _init_characters(self, cmd_lines):
        # Reset state from past simulation.
        self.available_character_list = []
        self.spawned_agents_list = []
        setting_dict = carb.settings.get_settings()
        # Get root assets path from setting, if not set, get the Isaac-Sim asset path
        people_asset_folder = setting_dict.get(PeopleSettings.CHARACTER_ASSETS_PATH)
        character_root_prim_path = setting_dict.get(PeopleSettings.CHARACTER_PRIM_PATH)
        if not character_root_prim_path:
            character_root_prim_path = "/World/Characters"

        if people_asset_folder:
            self.assets_root_path = people_asset_folder
        else:   
            root_path = get_assets_root_path()
            if root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
                return
            self.assets_root_path  = "{}/Isaac/People/Characters".format(root_path)

        if not self.assets_root_path:
            carb.log_error("Could not find people assets folder")
        
        result, properties = omni.client.stat(self.assets_root_path)
        if result != omni.client.Result.OK:
            carb.log_error("Could not find people asset folder : " + str(self.assets_root_path))
            return

        if not Sdf.Path.IsValidPathString(character_root_prim_path):
            carb.log_error(str(character_root_prim_path) + " is not a valid character root prim's path")
    
        if not self.stage.GetPrimAtPath(character_root_prim_path):
            prims.create_prim(character_root_prim_path, "Xform")
    
        character_root_prim = self.stage.GetPrimAtPath(character_root_prim_path)
        # Delete all previously loaded agents
        for character_prim in character_root_prim.GetChildren():
            if character_prim and character_prim.IsValid() and character_prim.IsActive():
                prims.delete_prim(character_prim.GetPath())


        # Reload biped and animations
        if not self.stage.GetPrimAtPath("{}/{}".format(character_root_prim_path, self.default_biped_usd)):
            biped_demo_usd = "{}/{}.usd".format(self.assets_root_path, self.default_biped_usd)
            prim = prims.create_prim("{}/{}".format(character_root_prim_path,self.default_biped_usd), "Xform", usd_path=biped_demo_usd)
            prim.GetAttribute("visibility").Set("invisible")

        # Reload character assets
        for cmd_line in cmd_lines:
            if not cmd_line:
                continue
            words = cmd_line.strip().split(' ')
            if words[0] != "Spawn":
                continue

            if len(words) != 6 and len(words) != 2:
                carb.log_error("Invalid 'Spawn' command issued, use command format - Spawn char_name or Spawn char_name x y z char_rotation.")
                return 

            # Add Spawn defaults
            if len(words) == 2:
                words.extend([0] * 4)

            # Do not use biped demo as a character name
            if str(words[1]) == "biped_demo":
                carb.log_warn("biped_demo is a reserved name, it cannot be used as a character name.")
                continue
            
            # Don't allow duplicates
            if str(words[1]) in self.spawned_agents_list:
                carb.log_warn(str(words[1]) + " has already been generated")
                continue
            
            # Check if prim already exists
            if self.stage.GetPrimAtPath("{}/{}".format(character_root_prim_path,words[1])):
                carb.log_warn("Path: " + str("{}/{}".format(character_root_prim_path,words[1])) + "has been taken, please try another character name")
                continue
            
            char_name, char_usd_file = self.get_path_for_character_prim(words[1])
            if char_usd_file:
                prim = prims.create_prim("{}/{}".format(character_root_prim_path,words[1]), "Xform", usd_path=char_usd_file)
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(float(words[2]),float(words[3]), float(words[4])))
                if type(prim.GetAttribute("xformOp:orient").Get()) == Gf.Quatf:
                    prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1), float(words[5])).GetQuat()))
                else:
                    prim.GetAttribute("xformOp:orient").Set(Gf.Rotation(Gf.Vec3d(0,0,1), float(words[5])).GetQuat())
                
                # Store agent names for deletion
                self.spawned_agents_list.append(words[1])

    def get_path_for_character_prim(self, agent_name):

        # Get a list of available character assets
        if not self.available_character_list:
            self.available_character_list = self.get_character_asset_list()
            if not self.available_character_list:
                return

        # Check if a folder with agent_name exists. If exists we load the character, else we load a random character
        agent_folder = "{}/{}".format(self.assets_root_path, agent_name)
        result, properties = omni.client.stat(agent_folder)
        if result == omni.client.Result.OK:
            char_name = agent_name
        else:
            # Pick a random character from available character list
            char_name = random.choice(self.available_character_list)
        
        # Get the usd present in the character folder
        character_folder = "{}/{}".format(self.assets_root_path, char_name)
        character_usd = self.get_usd_in_folder(character_folder)
        if not character_usd:
            return
        
        if len(self.available_character_list) != 0 and (char_name in self.available_character_list):
            self.available_character_list.remove(char_name)
    
        # Return the character name (folder name) and the usd path to the character
        return (char_name, "{}/{}".format(character_folder, character_usd))
        
        
    def get_character_asset_list(self):
        # List all files in characters directory
        result, folder_list = omni.client.list("{}/".format(self.assets_root_path))

        if result != omni.client.Result.OK:
            carb.log_error("Unable to get character assets from provided asset root path.")
            return

        # Prune items from folder list that are not directories.
        pruned_folder_list = [folder.relative_path for folder in folder_list 
            if (folder.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN) and not folder.relative_path.startswith(".")]

        if self.default_biped_asset_name in pruned_folder_list:
            pruned_folder_list.remove(self.default_biped_asset_name)
        return pruned_folder_list


    def get_usd_in_folder(self, character_folder_path):
        result, folder_list = omni.client.list(character_folder_path)
        
        if result != omni.client.Result.OK:
            carb.log_error("Unable to read character folder path at {}".format(character_folder_path))
            return

        for item in folder_list:
            if item.relative_path.endswith(".usd"):
                return item.relative_path

        carb.log_error("Unable to file a .usd file in {} character folder".format(character_folder_path))


    def _setup_characters_on_clicked(self):
        self.stage = omni.usd.get_context().get_stage()
        anim_graph_prim = None
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "AnimationGraph":
                anim_graph_prim = prim
                break

        if anim_graph_prim is None:
            carb.log_warn("Unable to find an animation graph on stage.")
            return

        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "SkelRoot" and UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible:
                
                # remove animation graph attribute if it exists
                omni.kit.commands.execute(
                    "RemoveAnimationGraphAPICommand",
                    paths=[Sdf.Path(prim.GetPrimPath())]
                )

                omni.kit.commands.execute(
                    "ApplyAnimationGraphAPICommand",
                    paths=[Sdf.Path(prim.GetPrimPath())],
                    animation_graph_path=Sdf.Path(anim_graph_prim.GetPrimPath())
                )
                omni.kit.commands.execute(
                    "ApplyScriptingAPICommand",
                    paths=[Sdf.Path(prim.GetPrimPath())]
                )
                attr = prim.GetAttribute("omni:scripting:scripts")
                
                setting_dict = carb.settings.get_settings()
                ext_path = setting_dict.get(PeopleSettings.BEHAVIOR_SCRIPT_PATH)
                if not ext_path:
                    ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__) + "/omni/anim/people/scripts/character_behavior.py"
                attr.Set([r"{}".format(ext_path)])

  
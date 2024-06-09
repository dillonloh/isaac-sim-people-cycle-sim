# Omni.Anim.People (BETA)

Omni.Anim.People is an extension for simulating human events in environments such as retail stores, warehouses, office buildings, traffic intersections etc. A simulation consists of a sequence of actions for each character in the scene to perform. The character actions currently supported are - idle, look around, sit, stand, walk, and queue. The extension consists of a UI to configure simulation settings, behavior scripts that control the characters in a simulation, and pre-configured character assets and animations that can be readily used.

We have implemented 2 new features in our version:

1) Added new Teleport command to teleport an animated character
- omni/anim/people/scripts/character_behavior.py
- omni/anim/people/scripts/commands/teleport.py
  
2) Added ROS topic integration to publish coordinates of animated characters
- omni/anim/people/scripts/global_character_position_manager.py

Simply replace your omni.anim.people extension folder with this repo's files.
You should be able to find the folder in a file path similar to the below (assuming you are using IsaacSim v2023.1.1)

/home/$USER/.local/share/ov/pkg/isaac_sim-2023.1.1/extscache/omni.anim.people-0.2.4

![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/14a3f71c-ff95-4470-8a47-45a1b8b8c759)# Omni.Anim.People (BETA)

Omni.Anim.People is an extension for simulating human events in environments such as retail stores, warehouses, office buildings, traffic intersections etc. A simulation consists of a sequence of actions for each character in the scene to perform. The character actions currently supported are - idle, look around, sit, stand, walk, and queue. The extension consists of a UI to configure simulation settings, behavior scripts that control the characters in a simulation, and pre-configured character assets and animations that can be readily used.

We have implemented 2 new features in our version:

1) Added new Teleport command to teleport an animated character
- omni/anim/people/scripts/character_behavior.py
- omni/anim/people/scripts/commands/teleport.py
  
2) Added ROS topic integration to publish coordinates of animated characters
- omni/anim/people/scripts/global_character_position_manager.py


# How to use

Simply replace your omni.anim.people extension folder with this repo's files.
You should be able to find the folder in a file path similar to the below (assuming you are using IsaacSim v2023.1.1)

/home/$USER/.local/share/ov/pkg/isaac_sim-2023.1.1/extscache/omni.anim.people-0.2.4

Here are the specific changes:

## Teleport Command

Ensure that the new command file is created as omni/anim/people/scripts/commands/teleport.py
![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/fc11c321-fc5e-4d16-a472-a1141de800b8)

Import it into the omni/anim/people/scripts/character_behavior.py file and then add it as a valid command in the `get_command` function.
![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/924f4323-dfd9-48c4-904c-fc05b0b7f45b)
![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/99ebe007-b3c5-4a54-9816-85f20c5aba6a)

## Ros Topic Coordinate Publishing
Ensure that your terminal sources ros2 properly by modifying your ~/.bashrc file (necessary for ros2 to work in IsaacSim since it runs from terminal)
![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/70ada254-f5cc-42e6-84c4-05d60057a3e3)


Add the ros topic initialisation and publishing logic into the `__init__` function of `GlobalCharacterPositionManager`
![image](https://github.com/dillonloh/isaac-sim-people-cycle-sim/assets/72494545/7b9fdba7-b4e1-4bfa-afda-bdd269f3c540)



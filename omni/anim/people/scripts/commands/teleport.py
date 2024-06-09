from .base_command import Command

class Teleport(Command):
    """
    Command class to teleport a character to a specific location.
    """
    def __init__(self, character, command, navigation_manager):
        super().__init__(character, command, navigation_manager)
        
    def setup(self):
        super().setup()
        # Assuming the target location is passed as part of the command.
        # Ensure the target_location is a tuple of three floats (x, y, z).

        target_location = tuple(float(coord) for coord in self.command[1:])

        # Default rotation (identity quaternion) since no rotation is specified.
        default_rotation = (0.0, 0.0, 0.0, 1.0)
        
        self.character.set_variable("Action", "Teleport")
        # Set the character's position and rotation instantly.
        print("Teleporting character to", target_location)
        self.character.set_world_transform(target_location, default_rotation)
        
    def execute(self, dt):
        if not self.is_setup:
            self.setup()
        return self.update(dt)

    def update(self, dt):
        # Since teleportation is instantaneous, we mark the command as completed immediately.
        return self.exit_command()

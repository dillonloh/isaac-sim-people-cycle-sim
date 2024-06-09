# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse

class CodeGenerator():
    """
    Generates a base command file from the given parameters. Uses the simple_cmd_template.txt as a template.
    """
    def __init__(self, base_template, cmd_name, transition_name):
        self.base_template = base_template
        self.cmd_name = cmd_name
        self.transition_name = transition_name

    def generate_cmd_file(self):
        with open(self.base_template, "r") as file_in:
            output_file_string = "{}.py".format(self.cmd_name.lower())
            with open(output_file_string, "w") as file_out:
                for line in file_in:
                    file_out.write(line.replace("$CMD_NAME", self.cmd_name).replace("$TRANSITION_NAME", self.transition_name))
        print("Successfully Generated the python command file")
    

def create_parser():
    parser = argparse.ArgumentParser("Dataset generator")
    parser.add_argument('--template', '-t', help="Template to use")
    parser.add_argument('--name', '-n', help="Name of the command")
    parser.add_argument('--transition', '-tr', help="Transition condition for moving to command")
    return parser


def main():
    parser = create_parser()
    args = parser.parse_args()
    code_gnt = CodeGenerator(args.template, args.name, args.transition)
    code_gnt.generate_cmd_file()


if __name__ == '__main__':
    main()

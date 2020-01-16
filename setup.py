import os
import argparse
import subprocess

import numpy as np

from time import time
from os.path import join


VS2017_COMMUNITY = "C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/"


def env_settings(args) -> str:
    settings_path = join(args.envs_root, "settings.json")
    with open(settings_path) as settings_file:
        settings = settings_file.read()
    return settings


def run_env(args) -> None:
    subprocess.run(["run.bat"], cwd=args.environment, shell=True)


def open_blocks_sln(args, build_airsim=False) -> None:
    """ Build AirSim, deploy it to Blocks and open its VS 2017 solution (.sln) """
    cmds = [
        "&&", "cd", "Unreal/Environments/Blocks",
        "&&", "update_from_git.bat",
        "&&", "start", "Blocks.sln",
    ]

    if build_airsim:
        cmds = ["&&", "git", "pull", "&&", "build.cmd"] + cmds

    subprocess.Popen(
        ["cmd", "/K", join(VS2017_COMMUNITY, "VC/Auxiliary/Build/vcvars64.bat")]
        + cmds,
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=args.airsim_root,
    )


def open_vs2017_cmd_prompt(args) -> None:
    """ Open `x64 Native Tools Command Prompt for VS 2017` on the AirSim directory """
    subprocess.Popen(
        ["cmd", "/K", join(VS2017_COMMUNITY, "VC/Auxiliary/Build/vcvars64.bat")],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=args.airsim_root,
    )


###############################################################################


def get_parser():
    parser = argparse.ArgumentParser(description="Automate AirSim workflow.")
    parser.add_argument(
        "--airsim_root",
        type=str,
        default="D:/dev/AirSim",
        help="AirSim directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--envs_root",
        type=str,
        default="D:/Documents/AirSim",
        help="UE4 environments directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--env_name",
        type=str,
        default="Blocks",
        help="Environment folder name  (default: %(default)s)",
    )
    parser.add_argument(
        "--sim_mode",
        type=str,
        choices=["ComputerVision", "Multirotor", "Car"],
        default="Multirotor",
        help="SimMode  (default: %(default)s)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    return parser


if __name__ == "__main__":
    args = get_parser().parse_args()

    args.environment = join(args.envs_root, args.env_name)

    print(f"AirSim root: {args.airsim_root}")
    print(f"Environment: {args.environment}")
    print(f"SimMode:     {args.sim_mode}")

    open_blocks_sln(args)

###############################################################################

# ref.: https://www.bogotobogo.com/python/python_subprocess_module.php

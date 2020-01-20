import argparse
import subprocess

from os.path import join


VS2017_COMMUNITY = "C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/"


def get_parser():
    parser = argparse.ArgumentParser(description="Automate AirSim workflow.")
    parser.add_argument(
        "--airsim_root",
        type=str,
        default="D:\\dev\\AirSim",
        help="AirSim directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--settings_root",
        type=str,
        default="D:\\Documents\\AirSim",
        help="Directory where settings.json is stored  (default: %(default)s)",
    )
    parser.add_argument(
        "--envs_root",
        type=str,
        default="D:\\Documents\\AirSim",
        # default="D:\\dev\\AirSim\\Unreal\\Environments",
        # default="D:\\dev\\UE4\\Custom Environments",
        help="Downloaded environments directory  (default: %(default)s)",
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


###############################################################################
# AirSim BAT Files ###########################################################
###############################################################################


def build_cmd_list(cmds, conditional_execution=False):
    separator = "&&" if conditional_execution else ";"
    return [arg for cmd in cmds for arg in (cmd.split(" ") + [separator])][:-1]


###############################################################################
# Downloaded Environments #####################################################
###############################################################################


def env_settings(args) -> str:
    """ Return settings.json as a string """
    settings_path = join(args.settings_root, "settings.json")
    with open(settings_path) as settings_file:
        settings = settings_file.read()
    return settings


def run_env(args, width: int = None, height: int = None) -> None:
    """ Run a downloaded environment in windowed mode """
    cmds = ["start", args.env_name]
    if width:
        cmds += [f"ResX={width}"]
        cmds += [f"ResY={height}" if height is not None else int(3 * width / 4)]
    cmds += ["-windowed"]
    subprocess.run(cmds, cwd=args.environment, shell=True)


###############################################################################
# x64 Native Tools Command Prompt for VS 2017 #################################
###############################################################################


def open_vs2017_cmd_prompt(args) -> None:
    """ Open `x64 Native Tools Command Prompt for VS 2017` on the AirSim directory """
    subprocess.Popen(
        ["cmd", "/K", join(VS2017_COMMUNITY, "VC/Auxiliary/Build/vcvars64.bat")],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=args.airsim_root,
    )


def open_blocks_sln(args, build_airsim=False) -> None:
    """ Build AirSim, deploy it to `Blocks` and open its VS 2017 solution (.sln) """
    vs2017_cmds = []  # run on `x64 Native Tools Command Prompt for VS 2017`

    if build_airsim:
        vs2017_cmds += ["&&", "git", "pull"]
        vs2017_cmds += ["&&", "build.cmd"]

    vs2017_cmds += ["&&", "cd", "Unreal/Environments/Blocks"]
    vs2017_cmds += ["&&", "update_from_git.bat"]
    vs2017_cmds += ["&&", "start", "Blocks.sln"]

    subprocess.Popen(
        ["cmd", "/K", join(VS2017_COMMUNITY, "VC/Auxiliary/Build/vcvars64.bat")]
        + vs2017_cmds,
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=args.airsim_root,
    )


def sync_blocks_sln(args) -> None:
    """ Rebuild `Blocks`' project files """
    vs2017_cmds = []  # run on `x64 Native Tools Command Prompt for VS 2017`

    vs2017_cmds += ["&&", "cd", "Unreal/Environments/Blocks"]
    vs2017_cmds += ["&&", "clean.bat"]
    vs2017_cmds += ["&&", "GenerateProjectFiles.bat"]

    subprocess.Popen(
        ["cmd", "/K", join(VS2017_COMMUNITY, "VC/Auxiliary/Build/vcvars64.bat")]
        + vs2017_cmds,
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=args.airsim_root,
    )


###############################################################################
###############################################################################
###############################################################################


if __name__ == "__main__":
    args = get_parser().parse_args()

    args.environment = join(args.envs_root, args.env_name)

    print(f"AirSim root: {args.airsim_root}")
    print(f"Environment: {args.environment}")
    print(f'SimMode:     "{args.sim_mode}"')
    if args.verbose:
        print(f"settings.json: {env_settings(args)}")


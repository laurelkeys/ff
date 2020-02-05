import os
import sys
import glob
import time
import random
import psutil
import argparse
import warnings
import subprocess

from typing import List, Tuple

try:
    import airsim
except:
    pass  # don't worry, it'll be imported later


# aliases for directories with downloaded environments
ENV_ROOT = {
    "doc": "D:\\Documents\\AirSim",  # .exe files
    "dev": "D:\\dev\\AirSim\\Unreal\\Environments",  # .sln and .uproject files
    "custom": "D:\\dev\\UE4\\Custom Environments",   # .sln and .uproject files
}

#######################################################
#######################################################


def fly(client: airsim.MultirotorClient, args) -> None:
    if args.verbose:
        print(f"[ff] HomeGeoPoint:\n{client.getHomeGeoPoint()}\n")
        print(f"[ff] VehiclePose:\n{client.simGetVehiclePose()}\n")
        #print(f"[ff] MultirotorState:\n{client.getMultirotorState()}\n")

    print(f"[ff] Taking off")
    client.takeoffAsync(timeout_sec=8).join()

    ned_coordinates = []
    z = 4
    for x in [-2, 2]:
        for y in [-2, 2]:
            ned_coordinates.append((y, x, -z))  # North, East, Down

    for coord in ned_coordinates:
        print(f"[ff] Moving to {coord}")
        client.simPrintLogMessage("NED coordinates: ", message_param=str(coord))
        client.moveToPositionAsync(*coord, velocity=5).join()

    client.goHomeAsync().join()
    print(f"[ff] Done")
    # client.reset()


###############################################################################
# airsim (general) ############################################################
###############################################################################


def import_airsim(airsim_path: str, create_symbolic_link: bool = False) -> None:
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        assert(os.path.exists(client_path)), f"\nexpected '{client_path}' doesn't exist\n"

        if create_symbolic_link:
            symlink_cmd = ["ln", "-s", airsim_path, "airsim"]
            if args.verbose:
                symlink_cmd.append("--verbose")
            subprocess.run(symlink_cmd)

            airsim_client_root = os.getcwd()
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # ModuleNotFoundError will be raised if the path is incorrect


def connect_to_airsim(vehicle_name: str = None) -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    if vehicle_name is not None:
        client.enableApiControl(True, vehicle_name)
        client.armDisarm(True, vehicle_name)
    else:
        client.enableApiControl(True)
        client.armDisarm(True)
    return client


def build_run_cmds(env_path: str, res: Tuple[int, int]=(1280, 720), ue4editor_path: str=None) -> List[str]:
    _, env_ext = os.path.splitext(os.path.basename(env_path))
    assert(env_ext in [".exe", ".sln", ".uproject"]), f"\ninvalid extension '{env_ext}'\n"
    
    if env_ext == ".exe":
        cmds = ["start", env_path]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        cmds.append("-windowed")
    
    elif env_ext == ".sln":
        cmds = ["start", env_path]
        # TODO use Visual Studio's devenv command to run after launch
    
    else:  # ".uproject"
        assert(os.path.isfile(ue4editor_path)), f"\nUnreal Editor '{ue4editor_path}' doesn't exist\n"
        cmds = [ue4editor_path, env_path, "-game"]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}", 
                         f"-WinX={res[0]}", f"-WinY={res[1]}"])
        cmds.append("-windowed")  
    
    return cmds


def run_env(env_path: str, env_proc: str, **kwargs) -> None:
    env_name, _ = os.path.splitext(os.path.basename(env_path))
    assert(os.path.isfile(env_path)), f"\n'{env_path}' doesn't exist\n"
    
    already_running = [p for p in psutil.process_iter() if p.name() == env_proc]
    if already_running:
        print(f"{env_name} is already running:")
        for p in already_running:  # there should (usually) only be one
            print(f" - name={p.name()}, pid={p.pid()}")  # p.name() == env_proc
        return

    subprocess.run(build_launch_cmds(env_path, **kwargs), shell=True)
    time.sleep(5)  # wait for the drone to spawn


###############################################################################
# main ########################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    try:
        airsim_path = airsim.__path__
    except:
        airsim_path = os.path.join(args.airsim_root, "PythonClient", "airsim")
        import_airsim(airsim_path, create_symbolic_link=args.symbolic_link)
    finally:
        if args.verbose:
            path_str = f"'airsim' path: {airsim.__path__[0]}"
            print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    if args.env_name is not None:  # --launch option was passed    
        launch_env(args)
    quit()

    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
    finally:
        if args.disable_api_on_exit:
            client.enableApiControl(False)


def possible_env_folders(env_root: str) -> List[str]:
    env_folders = []
    for ext in ["*.exe", "*.sln", "*.uproject"]:
        env_folders.extend(glob.glob(os.path.join(args.env_root, "*", ext)))
    return env_folders


def launch_env(args: argparse.Namespace) -> None:
    if args.env_root in ENV_ROOT:  # root alias
        args.env_root = ENV_ROOT[args.env_root]
    
    if len(args.env_name) == 0:
        env_folders = possible_env_folders(args.env_root)
        assert(env_folders), f"\nno environment folder was found in '{args.env_root}'\n"
        args.env_name = random.choice(env_folders)
        if args.verbose and len(env_folders) > 1:
            print("Possible environment folders found:\n - ", ('\n - ').join(env_folders))
            print(f"Randomly selected: {args.env_name}")
    
    env_dir = os.path.join(args.env_root, args.env_name)
    
    # FIXME the environment's folder name might be different from its .exe file name
    if args.edit:
        launch_env(env_path=os.path.join(env_dir, args.env_name + ".sln"),
                    env_proc="devenv.exe")
    elif glob.glob(os.path.join(env_dir, args.env_name + ".exe")):
        launch_env(env_path=os.path.join(env_dir, args.env_name + ".exe"),
                   env_proc=args.env_name + ".exe")
    else:  # assume it's a .uproject file
        launch_env(env_path=os.path.join(env_dir, args.env_name + ".uproject"),
                   env_proc="UE4Editor.exe", ue4editor_path=args.unreal_editor_exe)

###############################################################################
# argument parsing ############################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "--airsim_root",
        type=str,
        default="D:\\dev\\AirSim",
        help="AirSim directory  (default: %(default)s)",
    )

    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory.",
    )

    parser.add_argument(
        "--launch",
        dest="env_name",
        metavar="ENV_NAME",
        type=str,
        nargs="?",
        const="",  # (randomly) selects a valid environment folder in env_root
        help="Name of the folder that contains the environment (.exe or .uproject file) to run.",
    )

    parser.add_argument(
        "--from",
        dest="env_root",
        metavar="ENV_ROOT",
        type=str,
        default="D:\\Documents\\AirSim",
        help="Directory that contains the environment folder  (default: %(default)s)."
             "\nAliases: {"
             + ", ".join([f"'{alias}': {env_root}" for alias, env_root in ENV_ROOT.items()])
             + "}.",
    )

    parser.add_argument(
        "--edit",
        "-vs",
        action="store_true",
        help="Launch the specified environment's .sln in Visual Studio (instead of running its .uproject).",
    )

    parser.add_argument(
        "--unreal_editor_exe",
        "-ue4",
        type=str,
        default="D:\\Program Files\\Epic Games\\UE_4.18\\Engine\\Binaries\\Win64\\UE4Editor.exe",
        help="Path to UE4Editor.exe, needed to launch .sln environments  (default: %(default)s)",
    )

    parser.add_argument(
        "--disable_api_on_exit",
        action="store_true",
        help="Disable API control on exit by calling client.enableApiControl(False).",
    )

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

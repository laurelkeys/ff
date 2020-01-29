import os
import sys
import time
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
    "dev": "D:\\dev\\AirSim\\Unreal\\Environments",  # .sln files
    "custom": "D:\\dev\\UE4\\Custom Environments",   # .sln files
}

#######################################################
# NOTE do stuff here ##################################
#######################################################
def fly(client: airsim.MultirotorClient, args) -> None:
    if args.verbose:
        print(f"[ff] SceneObjects:\n{client.simListSceneObjects()}\n")
        print(f"[ff] HomeGeoPoint:\n{client.getHomeGeoPoint()}\n")
        print(f"[ff] VehiclePose:\n{client.simGetVehiclePose()}\n")
        # print(f"[ff] MultirotorState:\n{client.getMultirotorState()}\n")
        # print(f"[ff] GroundTruthEnvironment:\n{client.simGetGroundTruthEnvironment()}\n")
        # print(f"[ff] GroundTruthKinematics:\n{client.simGetGroundTruthKinematics()}\n")
    
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
        if not os.path.exists(client_path):
            print(f"\nWARNING: expected '{client_path}' does not exist\n")

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


def run_env(env_name: str, env_dir: str, res_x: int = None, res_y: int = None) -> None:
    fpath = os.path.join(env_dir, env_name)
    if os.path.isfile(f"{fpath}.exe"):
        already_running = [p for p in psutil.process_iter() if p.name() == f"{env_name}.exe"]
        if not already_running:
            cmds = ["start", env_name]
            if res_x is not None:
                cmds.append(f"ResX={res_x}")
                cmds.append(f"ResY={res_y}" if res_y is not None else int(3 * res_x / 4))
            cmds.append("-windowed")
            subprocess.run(cmds, cwd=env_dir, shell=True)
            time.sleep(4)  # wait for the drone to spawn
        else:
            print(f"{env_name}.exe is already running")
    elif os.path.isfile(f"{fpath}.sln"):
        # TODO
        pass
    else:
        print(f"\nWARNING: no environment .exe or .sln found in '{env_dir}'\n")


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

    if args.env_name is not None:
        run_env(
            args.env_name,
            env_dir=os.path.join(
                ENV_ROOT[args.env_root] if args.env_root in ENV_ROOT else args.env_root,
                args.env_name,
            ),
        )

    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
    finally:
        if args.disable_api_on_exit:
            client.enableApiControl(False)


###############################################################################
# argument parsing ############################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "--airsim_root",
        type=str, default="D:\\dev\\AirSim",
        help="AirSim directory  (default: %(default)s)"
    )

    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory."
    )

    parser.add_argument(
        "--launch",
        dest="env_name", metavar="ENV_NAME",
        type=str, nargs="?", const="Blocks",
        help="Run the specified environment  (default: %(const)s)."
    )

    parser.add_argument(
        "--from",
        dest="env_root", metavar="ENV_ROOT",
        type=str, default="D:\\Documents\\AirSim",
        help="Directory that contains the environment folder  (default: %(default)s)."
             "\nAliases: {"
             + ", ".join([f"'{alias}': {env_root}" for alias, env_root in ENV_ROOT.items()])
             + "}."
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

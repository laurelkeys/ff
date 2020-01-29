import os
import sys
import argparse
import subprocess

from typing import List, Tuple

try:
    import airsim
except:
    pass  # don't worry, it'll be imported later


###############################################################################
# specific ####################################################################
###############################################################################

# Aliases for directories with downloaded environments
ENVS_ROOT = {
    "doc": "D:\\Documents\\AirSim",
    "dev": "D:\\dev\\AirSim\\Unreal\\Environments",
    "custom": "D:\\dev\\UE4\\Custom Environments",
}


class Rect:
    def __init__(self, center: Tuple[float, float], width: float, height: float):
        self.center = center
        self.half_x = width / 2
        self.half_y = height / 2

    def coordinates(self) -> List[Tuple[float, float]]:
        return [
            (self.center + x, self.center + y)
            for (x, y) in [
                (-self.half_x, -self.half_y),
                (+self.half_x, -self.half_y),
                (-self.half_x, +self.half_y),
                (+self.half_x, +self.half_y),
            ]
        ]


###############################################################################
# airsim (specific) ###########################################################
###############################################################################


def fly(client: airsim.MultirotorClient, args) -> None:
    initial_pose: airsim.Pose = client.simGetVehiclePose()

    print(
        f"[ff] Taking off from ({initial_pose.position.x_val}, {initial_pose.position.y_val}, {initial_pose.position.z_val})"
    )
    client.takeoffAsync(timeout_sec=10).join()
    print(client.simGetVehiclePose())

    ned_coordinates = []
    z = 4
    for x in [-2, 2]:
        for y in [-2, 2]:
            ned_coordinates.append((y, x, -z))  # North, East, Down

    for coord in ned_coordinates:
        print(f"Moving to {coord}")
        client.moveToPositionAsync(*coord, velocity=5).join()

    client.reset()


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

def run_env(env_name: str, env_dir: str, res_x: int = None, height: int = None) -> None:
    cmds = ["start", env_name]
    if width is not None:
        cmds.append(f"ResX={width}")
        cmds.append(f"ResY={height}" if height is not None else int(3 * width / 4))
    cmds.append("-windowed")
    subprocess.run(cmds, cwd=os.path.join(env_dir, env_name), shell=True)

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

    if args.launch is not None:
        run_env(env_name=args.launch, env_dir=args.from)


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
        type=str,
        nargs="?",
        metavar="ENV",
        const="Blocks",
        help="Run the specified environment  (default: %(const)s).",
    )
    parser.add_argument(
        "--from",
        type=str,
        metavar="ENV_DIR",
        default="D:\\Documents\\AirSim",
        help="Environment directory  (default: %(default)s)."
        "\nAliases: {"
        + ", ".join([f"'{alias}': {env_dir}" for alias, env_dir in ENVS_ROOT.items()])
        + "}.",
    )
    parser.add_argument(
        "--disable_api_on_exit",
        action="store_true",
        help="Disable API control on exit by calling client.enableApiControl(False).",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

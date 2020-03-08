import os
import sys
import time
import msvcrt
import argparse

import ff

try:
    import airsim
except ModuleNotFoundError:
    airsim_path = ff.Default.AIRSIM_CLIENT_PATH
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))
    import airsim

from ff.types import to_xyz_str, to_xyzw_str, angles_to_str


###############################################################################
# preflight (called before connecting) ########################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    args.flight_velocity = 5


###############################################################################
# fly (called after connecting) ###############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()
    initial_state = client.getMultirotorState()

    if args.verbose:
        print(
            "[ff] VehiclePose \n"
            f"     .position    = {to_xyz_str(initial_pose.position)}\n"
            f"     .orientation = {to_xyzw_str(initial_pose.orientation)}\n"
            f"                    {angles_to_str(airsim.to_eularian_angles(initial_pose.orientation))}\n"
        )

    if initial_state.landed_state == airsim.LandedState.Landed:
        print(f"[ff] Taking off")
        client.takeoffAsync(timeout_sec=8).join()
    else:
        client.hoverAsync().join()  # airsim.LandedState.Flying

    print("args:", args)

    time.sleep(4)
    print(f"[ff] Drone reset")
    client.reset()


###############################################################################
# main ########################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        path_str = f"'airsim' path: {airsim.__path__[0]}"
        print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    if args.env_name is not None:
        ff.launch_env(args)  # the --launch option was passed
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset


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


###############################################################################
# argument parsing ############################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fly viewpoints capturing images")

    parser.add_argument(
        "--out",
        dest="output_folder",
        metavar="OUTPUT_FOLDER",
        type=str,
        default="tmp",
        help="Image output folder  (default: %(default)s/)",
    )

    parser.add_argument(
        "--viewpoints_path",
        type=str,
        default=os.path.join("viewpoints", "default.py"),
        help="Path to the viewpoints file  (default: %(default)s)",
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

# See https://github.com/microsoft/AirSim/blob/master/docs/apis.md#apis-for-multirotor
#     https://github.com/microsoft/AirSim/blob/master/docs/image_apis.md#python-1

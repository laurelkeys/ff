import os
import sys
import time
import json
import msvcrt
import argparse

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.viewpoints_path), f"Couldn't find file '{args.viewpoints_path}'"
    with open(args.viewpoints_path, "r") as viewpoints_file:
        viewpoints = json.load(viewpoints_file)
    args.viewpoints = zip(viewpoints["positions"], viewpoints["orientations"])


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()

    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)

    points, poses = [], []
    for position, orientation in args.viewpoints:
        point = airsim.Vector3r(*position)
        points.append(point)
        poses.append(airsim.Pose(point, airsim.Quaternionr(*orientation)))  # FIXME wxyz or xyzw?

    # client.simPlotPoints(points, duration=10)
    # client.simPlotLineStrip(points, duration=10)
    client.simPlotArrows(points, points[1:] + [points[0]], duration=10)

    print("[ff] Done")


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.VehicleClient()
    client.confirmConnection()
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "viewpoints_path", type=str, help="Path to a viewpoints file .json",
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

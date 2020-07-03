import os
import sys
import json
import time
import msvcrt
import argparse

from pynput import keyboard

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

    points, poses, names = [], [], []
    for position, orientation in args.viewpoints:
        position = airsim.Vector3r(*position)
        orientation = airsim.Quaternionr(*orientation)  # FIXME wxyz or xyzw?
        points.append(position)
        poses.append(airsim.Pose(position, orientation))
        names.append(ff.to_xyz_str(position) + "\n" + ff.to_xyzw_str(orientation))

    # client.simPlotPoints(points, duration=10)
    # client.simPlotLineStrip(points, thickness = 2.0, duration=10)
    # client.simPlotTransforms(poses, scale = 20.0, thickness = 4.0, duration=10)
    # client.simPlotTransformsWithNames(poses, names, tf_scale = 20.0, tf_thickness = 4.0, text_scale = 1.0, duration=10)

    checked = False
    def on_press(key):
        nonlocal checked
        # https://pythonhosted.org/pynput/keyboard.html#pynput.keyboard.Key
        if key == keyboard.Key.space:
            checked = not checked
            if checked:
                client.simPlotLineStrip(points, thickness = 2.0, is_persistent=True)
            else:
                client.simFlushPersistentMarkers()

    def on_release(key):
        if key == keyboard.Key.esc:
            return False  # stop the listener

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # TODO https://stackoverflow.com/questions/24072790/detect-key-press-in-python
    # https://pythonhosted.org/pynput/keyboard.html
    # https://zsiegel92.github.io/evilpython/lesson_6.html

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

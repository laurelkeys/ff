import os
import json
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
    assert os.path.isfile(args.viewpoints_path), f"Invalid file path: '{args.viewpoints_path}'"

    with open(args.viewpoints_path, "r") as viewpoints_file:
        viewpoints = json.load(viewpoints_file)

    args.viewpoints = zip(viewpoints["positions"], viewpoints["orientations"])

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


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
        orientation = airsim.Quaternionr(*orientation)  # xyzw
        points.append(position)
        poses.append(airsim.Pose(position, orientation))
        names.append(ff.to_xyz_str(position) + "\n" + ff.to_xyzw_str(orientation))

    plot_linestrips = True  # else plot_transforms
    curr_press_keys = set()
    key_combination = {keyboard.Key.space, keyboard.Key.shift}

    def on_press(key):
        nonlocal plot_linestrips, curr_press_keys, key_combination
        if key in key_combination:
            curr_press_keys.add(key)
            if curr_press_keys == {keyboard.Key.space}:
                if (plot_linestrips := not plot_linestrips) :
                    client.simPlotLineStrip(points, thickness=3, duration=10)
                else:
                    client.simPlotTransforms(poses, thickness=3, scale=40, duration=10)

    def on_release(key):
        nonlocal curr_press_keys, key_combination
        if key == keyboard.Key.esc:
            return False  # stop the listener
        if key in key_combination:
            curr_press_keys.remove(key)

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        print("[ff] Press [esc] to quit")
        listener.join()  # NOTE waits for an on_release() callback to return False

    # TODO only listen for keys in the environment application:
    # - https://stackoverflow.com/a/53210441
    # - https://stackoverflow.com/a/43791403
    # - https://stackoverflow.com/a/58333425
    # - https://askubuntu.com/a/909678
    # - https://askubuntu.com/a/1199313


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done")


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # NOTE don't `enableApiControl` or `armDisarm` since we are in CV mode
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("viewpoints_path", type=str, help="Path to a viewpoints JSON file")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

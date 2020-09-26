from __future__ import annotations
from ff.sim import SimMode
from ff.helper import settings_dict, settings_dict_to_json

import os
import argparse

import numpy as np

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Vector3r


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(
            *ff.LaunchEnvArgs(args),
            settings=settings_dict_to_json(settings_dict())
            # settings='"D:\\Documents\\Github\\ff\\src\\logs\\car_settings.json"'
            # settings='"{ \\"SettingsVersion\\": 1.2, \\"SimMode\\": \\"Car\\" }"'
            # settings='D:/Documents/Temp/Blocks/settings.json'
        )
        # FIXME see https://github.com/microsoft/AirSim/issues/2824#issuecomment-658988789

        input_or_exit("\nPress [enter] to connect to AirSim ")

    else:
        ff.log("Flying to ROI...")


def input_or_exit(prompt: str):
    try:
        input(prompt)
    except KeyboardInterrupt:
        exit()


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    assert os.path.isfile(args.roi), f"Invalid file path: '{args.roi}'"

    client.reset()
    initial_pose = client.simGetVehiclePose()
    initial_state = client.getMultirotorState()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
    if initial_state.landed_state == airsim.LandedState.Landed:
        print("[ff] Taking off")
        client.takeoffAsync(timeout_sec=2).join()

    ff.log_info(f"Loading ROI from '{args.roi}'")
    with open(args.roi, "r") as f:
        zone = Rect.from_dump(f.read())
        if args.verbose:
            ff.log_info(zone)
    print()

    # NOTE AirSim uses NED coordinates (hence, negative z values are up)
    ff.log("Teleporting to the center of the ROI..")

    # FIXME this doesn't work reliably in Multirotor mode...
    #       in this case, maybe it's best to load in the ROI on
    #       a settings.json file we pass AirSim to start up, such that:
    #       - if AirSim is alread launched, we simply fly to the ROI
    #       - otherwise, we read the file first, then launch AirSim
    client.simSetVehiclePose(Vector3r(np.nan, -10, np.nan), ignore_collison=True)

    ff.log("Rising and then hovering..")
    client.moveByVelocityZAsync(vx=0, vy=1, z=-7, duration=10).join()
    client.hoverAsync().join()

    client.reset()
    ff.log("Done")


###############################################################################
## main #######################################################################
###############################################################################


from cv_map_flight_zone import Rect
# TODO extract this to another file (maybe inside ff/), as this script
#      and cv_map_flight_zone.py depend on it being synced between them.


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("roi", type=str, help="Path to the flight zone (ROI) file")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

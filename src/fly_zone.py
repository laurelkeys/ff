from __future__ import annotations

import os
import time
import argparse

import ff

from Rect import Rect
from ff.helper import settings_str_from_dict
from wrappers.airsimy import AirSimSettings

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
    assert os.path.isfile(args.roi), f"Invalid file path: '{args.roi}'"

    with open(args.roi, "r") as f:
        args.roi = Rect.from_dump(f.read())  # region of interest

    if args.env_name is not None:
        # the --launch option was passed

        # FIXME debugging..
        start_pos = Vector3r(*ff.to_xyz_tuple(args.roi.center))
        start_pos.y_val += 4
        start_pos.z_val -= 10  # start higher up, to avoid crashing with objects

        ff.launch_env(
            *ff.LaunchEnvArgs(args),
            settings=settings_str_from_dict(
                AirSimSettings(
                    sim_mode=ff.SimMode.Multirotor,
                    ##view_mode=ff.ViewMode.SpringArmChase,
                    ##vehicles=[AirSimSettings.Vehicle("Drone1", position=start_pos)],
                ).as_dict()
            ),
        )

        # FIXME check if there's actually an env already running
        ##ff.log("Spawning at ROI...")
        args.fly_to_roi = True  ##False

        ff.input_or_exit("\nPress [enter] to connect to AirSim ")

    else:
        ##ff.log("Flying to ROI...")
        args.fly_to_roi = True


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(True)

    client.simFlushPersistentMarkers()
    client.simPlotLineStrip(points=args.roi.corners(repeat_first=True), is_persistent=True)

    initial_pose = client.simGetVehiclePose()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)

    if args.fly_to_roi:
        args.fly_to_roi = False
        center = ff.to_xyz_tuple(args.roi.center)
        ff.log(f"Flying to ROI {ff.xyz_to_str(center)}...")
        # NOTE ideally, we'd use `client.simSetVehiclePose(center, True)`
        #      in here, but it doesn't work reliably in Multirotor mode..

        corners_and_dists = [(corner, corner.distance_to(initial_pose.position)) for corner in args.roi.corners()]
        closest_corner, _ = min(corners_and_dists, key=lambda corner_and_dist: corner_and_dist[1])

        ff.log_debug(f"Closest corner {ff.to_xyz_str(closest_corner)}")
        client.simPlotPoints([closest_corner], color_rgba=[1.0] * 4, duration=11)
        client.moveToPositionAsync(*ff.to_xyz_tuple(closest_corner), velocity=5, timeout_sec=12).join()

        time.sleep(2)
        fly_zone(client, args.roi)

    ff.log("Done")


def fly_zone(client: airsim.MultirotorClient, zone: Rect) -> None:
    path = [
        Vector3r(corner.x_val, corner.y_val, corner.z_val - 5)
        for corner in zone.corners(repeat_first=True)
    ]
    client.simPlotLineStrip(points=path, is_persistent=True)
    client.moveOnPathAsync(path, velocity=2).join()

    ff.log_debug("Stretching path and surveying over it...")
    altitude_shift = Vector3r(0, 0, -8)
    test_zone = Rect(
        altitude_shift + zone.center,
        altitude_shift + zone.half_width * 4,
        altitude_shift + zone.half_height * 4
    )
    test_path = [
        Vector3r(corner.x_val, corner.y_val, corner.z_val - 5)
        for corner in test_zone.corners(repeat_first=True)
    ]
    client.simPlotLineStrip(points=test_path, is_persistent=True)
    # client.moveOnPathAsync(test_path, velocity=2).join()
    test_zigzag_path = test_zone.zigzag(4)
    client.simPlotLineStrip(points=test_zigzag_path, is_persistent=True, color_rgba=[0.0, 1.0, 0.0, 1.0])
    client.moveOnPathAsync(test_zigzag_path, velocity=2).join()

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

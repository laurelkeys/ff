import os
import time
import argparse

import ff

from ds import Rect, Controller
from wrappers.airsimy import AirSimSettings, Rgba

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Vector3r, Pose, Quaternionr


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.roi), f"Invalid file path: '{args.roi}'"

    with open(args.roi, "r") as f:
        args.roi = Rect.from_dump(f.read())

    args.start_pos = Vector3r(*ff.to_xyz_tuple(args.roi.center))
    if (z := args.z_offset) is not None:
        args.start_pos.z_val -= z # start higher up, to avoid crashing with objects
    else:
        # XXX debugging...
        args.start_pos.y_val += 4
        args.start_pos.z_val -= 10

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(
            *ff.LaunchEnvArgs(args),
            settings=ff.settings_str_from_dict(
                AirSimSettings(
                    sim_mode=ff.SimMode.Multirotor,
                    clock_speed=1.0 if not args.clock else args.clock,
                    view_mode=ff.ViewMode.SpringArmChase,
                ).as_dict()
            ),
        )
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")

        ff.log(f"Teleporting to {ff.to_xyz_str(args.start_pos)}...")
        args.fly_to_roi = False

    else:
        ff.log(f"Flying to {ff.to_xyz_str(args.start_pos)}...")
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

    closest_corner = args.roi.closest_corner(initial_pose.position)
    ff.log_debug(f"Closest corner {ff.to_xyz_str(closest_corner)}")
    client.simPlotPoints([closest_corner], color_rgba=Rgba.White, duration=11)

    # FIXME use args.start_pos instead of closest_corner
    if args.fly_to_roi:
        # NOTE ideally, we'd use `client.simSetVehiclePose(center, True)`
        #      in here, but it doesn't work reliably in Multirotor mode..
        client.moveToPositionAsync(
            *ff.to_xyz_tuple(closest_corner),
            velocity=5, timeout_sec=12
        ).join()
        ff.log(f"Flying to ROI {ff.to_xyz_str(closest_corner)}...")

    else:
        # NOTE using `nanQuaternionr()` should work for `simSetVehiclePose`.. but it doesn't
        new_pose = Pose(position_val=closest_corner, orientation_val=initial_pose.orientation)
        ff.log(f"Teleporting to ROI {ff.to_xyz_str(new_pose.position)}...")
        Controller.teleport(client, to=new_pose, ignore_collison=True)

    ff.log("Flying over zone...")
    time.sleep(2)
    fly_zone(client, args.roi, altitude_shift=6.5)


def fly_zone(client: airsim.MultirotorClient, zone: Rect, altitude_shift: float = 0.0) -> None:
    path = [
        Vector3r(corner.x_val, corner.y_val, corner.z_val - altitude_shift)
        for corner in zone.corners(repeat_first=True)
    ]

    client.simPlotLineStrip(points=path, is_persistent=True)
    Controller.fly_path(client, path) ##client.moveOnPathAsync(path, velocity=2).join()

    # NOTE testing.. stretching Rect, zigzagging path and flying over it
    test_zigzag_path = Rect(
        Vector3r(0, 0, -altitude_shift) + zone.center,
        Vector3r(0, 0, -altitude_shift) + zone.half_width * 4,
        Vector3r(0, 0, -altitude_shift) + zone.half_height * 4,
    ).zigzag(4)

    client.simPlotLineStrip(points=test_zigzag_path, is_persistent=True, color_rgba=[0.0, 1.0, 0.0, 1.0])
    Controller.fly_path(client, test_zigzag_path) ##client.moveOnPathAsync(test_zigzag_path, velocity=2).join()


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
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("roi", type=str, help="Path to the region of interest (ROI) file")
    parser.add_argument("--clock", type=float, help="Change AirSim's clock speed  (default: 1.0)")
    parser.add_argument("--z_offset", type=float, help="Set a positive value in meters to start higher up (e.g. to avoid crashing)")
    parser.add_argument("--closest_corner", type=float, help="Instead of flying to the center of the ROI, go to its closest corner to the drone")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

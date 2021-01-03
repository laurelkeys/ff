import os
import time
import argparse

import ff

from ds import Rect, Rgba, Controller
from wrappers.airsimy import AirSimSettings, connect

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Pose, Vector3r

AUGMENT_PATHS = True  # FIXME move to args

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.roi), f"Invalid file path: '{args.roi}'"

    with open(args.roi, "r") as f:
        args.roi = Rect.from_dump(f.read())

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(
            *ff.LaunchEnvArgs(args),
            settings=ff.settings_str_from_dict(
                AirSimSettings(
                    clock_speed=args.clock,
                    sim_mode=ff.SimMode.Multirotor,
                    view_mode=ff.ViewMode.SpringArmChase,
                    # camera_defaults=AirSimSettings.Camera(
                    #     capture_settings=[AirSimSettings.CaptureSettings()]
                    # ),
                    subwindows=[
                        AirSimSettings.Subwindow(0, camera_name=ff.CameraName.front_center),
                        AirSimSettings.Subwindow(1, camera_name=ff.CameraName.back_center),
                        AirSimSettings.Subwindow(2, camera_name=ff.CameraName.bottom_center),
                    ],
                ).as_dict()
            ),
        )
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    # Reset the drone
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(True)

    # Draw the ROI outline (erasing previous plots)
    client.simFlushPersistentMarkers()
    client.simPlotLineStrip(points=args.roi.corners(repeat_first=True), is_persistent=True)

    # Get the first position the drone will fly to
    initial_pose = client.simGetVehiclePose()
    closest_corner = args.roi.closest_corner(initial_pose.position)

    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
        ff.log_info(f"Closest corner {ff.to_xyz_str(closest_corner)}")

    start_pos = Vector3r(*ff.to_xyz_tuple(closest_corner if args.corner else args.roi.center))

    # NOTE AirSim uses NED coordinates, so negative Z values are "up" actually
    if (z := args.z_offset): start_pos.z_val -= z  # start higher up, to avoid crashing with objects
    if (y := args.y_offset): start_pos.y_val += y
    if (x := args.x_offset): start_pos.x_val += x

    # Plot a point at the start position and go to it
    client.simPlotPoints([start_pos], color_rgba=Rgba.White, is_persistent=True)

    if args.teleport:
        ff.log(f"Teleporting to {ff.to_xyz_str(start_pos)}...")
        # NOTE using `nanQuaternionr` for the orientation should work for `simSetVehiclePose`
        #      not to change it (which is called by `Controller.teleport`)... but it doesn't
        new_pose = Pose(start_pos, initial_pose.orientation)
        Controller.teleport(client, to=new_pose, ignore_collison=True)
    else:
        ff.log(f"Flying to {ff.to_xyz_str(start_pos)}...")
        client.moveToPositionAsync(*ff.to_xyz_tuple(start_pos), velocity=5, timeout_sec=12).join()

    # Fly over the region of interest now that we reached the starting position
    ff.log("Flying over zone...")
    time.sleep(2)
    fly_zone(client, args.roi, altitude_shift=args.z_offset + args.z_shift)


def fly_zone(client: airsim.MultirotorClient, zone: Rect, altitude_shift: float = 0.0) -> None:
    path = [
        Vector3r(corner.x_val, corner.y_val, corner.z_val - altitude_shift)
        for corner in zone.corners(repeat_first=True)
    ]
    if AUGMENT_PATHS:
        path = Controller.augment_path(path, max_dist=2)

    client.simPlotPoints(points=path, is_persistent=True, color_rgba=Rgba.White, size=5)
    client.simPlotLineStrip(points=path, is_persistent=True, color_rgba=Rgba.White)
    # Controller.fly_path(client, path) ##client.moveOnPathAsync(path, velocity=2).join()

    # XXX testing.. stretching Rect, zigzagging path and flying over it
    zz_path = Rect(
        Vector3r(0, 0, -altitude_shift) + zone.center,
        # Vector3r(0, 0, -altitude_shift) + zone.half_width * 4,
        # Vector3r(0, 0, -altitude_shift) + zone.half_height * 4,
        zone.half_width * 2,
        zone.half_height * 2,
    ).zigzag(4)
    if AUGMENT_PATHS:
        zz_path = Controller.augment_path(zz_path, max_dist=2)

    client.simPlotPoints(points=zz_path, is_persistent=True, color_rgba=Rgba.White, size=5)
    client.simPlotLineStrip(points=zz_path, is_persistent=True, color_rgba=Rgba.Green)
    Controller.fly_path(client, zz_path)  ##client.moveOnPathAsync(zz_path, velocity=2).join()

    # client.simFlushPersistentMarkers()
    # path = [
    #     Vector3r(corner.x_val, corner.y_val, corner.z_val - altitude_shift)
    #     for corner in zone.zigzag(4, 1, True)
    # ]
    # # client.simPlotLineStrip(points=path, is_persistent=True, color_rgba=Rgba.Yellow)
    # try:
    #     client.startRecording()
    #     Controller.fly_path(client, path)
    # except:
    #     pass
    # finally:
    #     client.stopRecording()


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect(ff.SimMode.Multirotor)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("roi", type=str, help="Path to the region of interest (ROI) file")
    parser.add_argument("--clock", type=float, default=1.0, help="Change AirSim's clock speed")
    parser.add_argument("--corner", action="store_true", help="Fly to the corner closest to the drone, instead of to the ROI center")
    parser.add_argument("--teleport", action="store_true", help="Teleport the drone, instead of flying it")
    parser.add_argument("--z_shift", type=float, default=6.5)  # FIXME add help string
    parser.add_argument("--z_offset", type=float, help="Set a positive value (in meters) to offset the starting position")
    parser.add_argument("--y_offset", type=float, help="Set a value (in meters) to offset the starting position")
    parser.add_argument("--x_offset", type=float, help="Set a value (in meters) to offset the starting position")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

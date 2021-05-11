import os
import time
import argparse

import ff
import airsim

from ie import airsimy
from ds.rgba import Rgba
from ff.types import to_xyz_str
from ie.airsimy import AirSimImage, connect
from airsim.types import Pose

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import (
        parse_uavmvs,
        convert_uavmvs_to_airsim_pose,
        convert_uavmvs_to_airsim_position,
    )

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    _, ext = os.path.splitext(args.trajectory_path)
    assert os.path.isfile(args.trajectory_path), f"Invalid file path: '{args.trajectory_path}'"
    assert ext in parse_uavmvs.keys(), f"Invalid trajectory extension: '{args.trajectory_path}'"

    args.trajectory = parse_uavmvs[ext](args.trajectory_path)
    if args.verbose:
        ff.log(f"The trajectory has {len(args.trajectory)} camera poses")

    if args.capture_dir:
        args.capture_dir = os.path.abspath(args.capture_dir)
        assert os.path.isdir(args.capture_dir), args.capture_dir

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


# FIXME find a good way to pass this in via args
INGORE_LOOK_AT_TARGET = False  # use uavmvs pose
LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue
# LOOK_AT_TARGET = data_config.Ned.Urban_Building
CAPTURE_CAMERA = ff.CameraName.front_center


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
        ff.log(
            "(front_center)  camera info:",
            client.simGetCameraInfo(camera_name=ff.CameraName.front_center),
        )
        ff.log(
            "(bottom_center) camera info:",
            client.simGetCameraInfo(camera_name=ff.CameraName.bottom_center),
        )

    if args.flush or (args.capture_dir and not args.debug):
        client.simFlushPersistentMarkers()

    if not INGORE_LOOK_AT_TARGET:
        camera_poses = []
        for camera in args.trajectory:
            position = convert_uavmvs_to_airsim_position(
                camera.position, translation=args.offset, scaling=args.scale
            )

            # Compute the forward axis as the vector which points
            # from the camera eye to the region of interest (ROI):
            x_axis = LOOK_AT_TARGET - position
            x_axis /= x_axis.get_length()  # normalize

            z_axis = airsimy.vector_projected_onto_plane(airsimy.DOWN, plane_normal=x_axis)
            z_axis /= z_axis.get_length()  # normalize

            y_axis = z_axis.cross(x_axis)

            orientation = airsimy.quaternion_that_rotates_axes_frame(
                source_xyz_axes=airsimy.NED_AXES_FRAME,
                target_xyz_axes=(x_axis, y_axis, z_axis),
            )

            camera_poses.append(Pose(position, orientation))
    else:
        camera_poses = [
            convert_uavmvs_to_airsim_pose(_, translation=args.offset, scaling=args.scale)
            for _ in args.trajectory
        ]

    if args.debug:
        client.simPlotLineStrip(
            [_.position for _ in camera_poses],
            Rgba.White,
            thickness=2.5,
            is_persistent=True,
        )

    n_of_poses = len(camera_poses)
    pad = len(str(n_of_poses))

    record = []

    for i, pose in enumerate(camera_poses):
        client.simSetVehiclePose(pose, ignore_collison=True)

        pose_str = f"{i:{pad}} / {n_of_poses}"
        position_str = to_xyz_str(pose.position, show_hints=False)

        if args.capture_dir and not args.debug:
            name = os.path.join(args.capture_dir, f"{args.prefix}pose{args.suffix}_{i:0{pad}}.png")
            airsim.write_png(name, AirSimImage.get_mono(client, CAPTURE_CAMERA))
            ff.log(f"Saved image ({pose_str}) to '{name}' at {position_str}")
            record.append(
                airsimy.AirSimRecord.make_line_string(
                    pose.position,
                    pose.orientation,
                    time_stamp="0",  # HACK
                    image_file=name,
                )
            )
        else:
            ff.log(f"Going to pose ({pose_str}): {position_str}")
            if args.debug:
                client.simPlotTransforms([pose], scale=100, is_persistent=True)
                client.simPlotArrows(
                    [pose.position], [LOOK_AT_TARGET], Rgba.White, thickness=2.0, duration=10
                )

        time.sleep(args.sleep)

    if record:
        print()
        print(airsimy.AirSimRecord.make_header_string())
        for line in record:
            print(line)


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect(ff.SimMode.ComputerVision)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done\n")
        ff.log_warning(f"Used scale = {args.scale} and offset = {args.offset}")
        ff.log_warning(f"Used CAPTURE_CAMERA = {CAPTURE_CAMERA}")
        ff.log_warning(
            "Ignored LOOK_AT_TARGET"
            if INGORE_LOOK_AT_TARGET
            else f"Used LOOK_AT_TARGET = {to_xyz_str(LOOK_AT_TARGET)}"
        )


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Follow a trajectory generated by uavmvs in AirSim's ComputerVision mode."
    )

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

    parser.add_argument("--debug", action="store_true", help="Skip capturing images but plot poses")
    parser.add_argument("--capture_dir", type=str, help="Folder where image captures will be saved")
    parser.add_argument("--sleep", type=float, help="Delay between each capture", default=0.1)
    parser.add_argument("--flush", action="store_true", help="Flush old plots")

    parser.add_argument("--prefix", type=str, help="Prefix added to output image names", default="")
    parser.add_argument("--suffix", type=str, help="Suffix added to output image names", default="")

    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Offset added to all points "
        " (e.g. --offset 1.2375 -6.15 7.75)",  # data_config.Uavmvs.Cidadela_Statue_Offset
    )
    parser.add_argument(
        "--scale",
        type=float,
        help="Scale added to all points "
        " (e.g. --scale 0.168)",  # data_config.Uavmvs.Cidadela_Statue_Scale
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

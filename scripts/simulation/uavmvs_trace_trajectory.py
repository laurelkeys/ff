import os
import argparse

import ff
import airsim

from ie import airsimy
from ds.rgba import Rgba
from ff.types import to_xyz_str, to_xyz_tuple
from ie.airsimy import (
    AirSimImage,
    connect,
    pose_at_simulation_pause,
    quaternion_that_rotates_orientation,
)
from airsim.types import DrivetrainType, Pose, Quaternionr, Vector3r, YawMode

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

# NOTE much from this file is simply copy-pasted from cv_trace_uavmvs_trajectory.py

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
LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue
# LOOK_AT_TARGET = data_config.Ned.Urban_Building
CAPTURE_CAMERA = ff.CameraName.front_center

# HACK fix after testing
TEST_AIMING_AT_ROI = True
center_of_roi = data_config.Ned.Cidadela_Statue


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    client.moveToZAsync(z=-10, velocity=5).join()  # XXX avoid colliding
    client.hoverAsync().join()

    if args.flush or (args.capture_dir and not args.debug):
        client.simFlushPersistentMarkers()

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

    if args.debug:
        client.simPlotPoints(
            [pose.position for pose in camera_poses], Rgba.Blue, is_persistent=True
        )

    n_of_poses = len(camera_poses)
    pad = len(str(n_of_poses))

    record = []

    for i, camera_pose in enumerate(camera_poses):
        client.moveToPositionAsync(
            *to_xyz_tuple(camera_pose.position),
            velocity=2,
            drivetrain=DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=YawMode(is_rate=False, yaw_or_rate=airsimy.YAW_N),
        ).join()

        with pose_at_simulation_pause(client) as actual_drone_pose:
            # actual_camera_pose = client.simGetCameraInfo(CAPTURE_CAMERA).pose
            # fake_camera_pose = Pose(
            #     # NOTE simSetCameraPose expects relative values
            #     position_val=Vector3r(0, 0, 0),
            #     orientation_val=quaternion_that_rotates_orientation(
            #         actual_camera_pose.orientation, camera_pose.orientation
            #     ),
            # )
            # client.simSetCameraPose(CAPTURE_CAMERA, fake_camera_pose)

            fake_drone_pose = Pose(actual_drone_pose.position, camera_pose.orientation)
            client.simSetVehiclePose(fake_drone_pose, ignore_collision=True)
            client.simContinueForFrames(1)  # NOTE without this, the pose won't change!

            if args.capture_dir and not args.debug:
                name = os.path.join(
                    args.capture_dir, f"{args.prefix}pose{args.suffix}_{i:0{pad}}.png"
                )
                airsim.write_png(name, AirSimImage.get_mono(client, CAPTURE_CAMERA))

                pose_str = f"{i:{pad}} / {n_of_poses}"
                position_str = to_xyz_str(camera_pose.position, show_hints=False)
                ff.log(f"Saved image ({pose_str}) to '{name}' at {position_str}")

                record.append(
                    airsimy.AirSimRecord.make_line_string(
                        camera_pose.position,
                        camera_pose.orientation,
                        time_stamp="0",  # HACK
                        image_file=name,
                    )
                )
            elif args.debug:
                client.simPlotTransforms([camera_pose], scale=100, is_persistent=True)
                client.simPlotArrows(
                    [camera_pose.position], [LOOK_AT_TARGET], Rgba.White, thickness=2.0, duration=10
                )

            client.simSetVehiclePose(actual_drone_pose, ignore_collision=True)

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
    client = connect(ff.SimMode.Multirotor)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done\n")
        ff.log_warning(f"Used scale = {args.scale} and offset = {args.offset}")
        ff.log_warning(f"Used CAPTURE_CAMERA = {CAPTURE_CAMERA}")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Follow a trajectory generated by uavmvs in AirSim's Multirotor mode."
        " The drone position and orientation are printed after reaching every view point."
    )  # NOTE the output can be parsed to generate (TUM) .log trajectory files for ATE/RPE

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

    parser.add_argument("--debug", action="store_true", help="Skip capturing images but plot poses")
    parser.add_argument("--capture_dir", type=str, help="Folder where image captures will be saved")
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

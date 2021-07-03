import os
import sys
import time
import argparse

import ff
import numpy as np
import airsim

from ds.rgba import Rgba
from ff.types import to_xyz_str, to_xyz_tuple
from ie.airsimy import (
    YAW_N,
    AirSimImage,
    AirSimRecord,
    AirSimNedTransform,
    connect,
    viewport_vectors,
    pose_at_simulation_pause,
    quaternion_from_two_vectors,
    quaternion_from_rotation_axis_angle,
    frustum_plot_list_from_viewport_vectors,
    quaternion_orientation_from_eye_to_look_at,
)
from airsim.types import Pose, YawMode, Quaternionr, DrivetrainType

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import (
        TrajectoryCameraKind,
        rot_to_quat,
        parse_uavmvs,
        convert_uavmvs_to_airsim_position,
    )

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise

VELOCITY = 5

SIM_MODE = ff.SimMode.Multirotor
# SIM_MODE = ff.SimMode.ComputerVision

LOOK_AT_TARGET = None
# LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue
# LOOK_AT_TARGET = data_config.Ned.Urban_Building

# CAPTURE_CAMERA = ff.CameraName.front_center
CAPTURE_CAMERA = ff.CameraName.bottom_center

IS_CV_MODE = SIM_MODE == ff.SimMode.ComputerVision
CV_SLEEP_SEC = 0.1

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    path = args.trajectory_path
    _, ext = os.path.splitext(path)
    assert os.path.isfile(path), path
    assert ext in parse_uavmvs.keys(), path

    args.trajectory = parse_uavmvs[ext](path)
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


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
        ff.log(client.simGetCameraInfo(camera_name=CAPTURE_CAMERA))

    if args.flush or (args.capture_dir and not args.debug):
        client.simFlushPersistentMarkers()

    # NOTE see cv_plot_point_cloud.py and
    FORCE_FRONT_XAXIS = True  # XXX :ForceFrontXAxis:
    args.trajectory = args.trajectory[:-5]  # XXX :SkipLastFive:

    half_90 = np.deg2rad(90 / 2)
    positive_90_around_z = Quaternionr(0, 0, np.sin(half_90), w_val=np.cos(half_90))
    negative_90_around_z = Quaternionr(0, 0, -np.sin(half_90), w_val=np.cos(half_90))

    camera_poses = []
    for camera in args.trajectory:
        ## pose = convert_uavmvs_to_airsim_pose(camera, translation=args.offset, scaling=args.scale)
        orientation_matrix = camera._rotation_into(TrajectoryCameraKind.Traj)
        w, x, y, z = rot_to_quat(orientation_matrix)
        pose = Pose(
            convert_uavmvs_to_airsim_position(
                camera.position, translation=args.offset, scaling=args.scale
            ),
            Quaternionr(x, -y, -z, w),
        )

        if FORCE_FRONT_XAXIS:  # XXX fix position
            pose.position.x_val, pose.position.y_val = pose.position.y_val, -pose.position.x_val
            # NOTE equivalent to rotating 90 degrees (centered at the origin) around the Z axis:
            # qposition = airsimy.v2q(pose.position)
            # pose.position = airsimy.q2v(qposition.rotate(negative_90_around_z))

        if LOOK_AT_TARGET is not None:
            pose.orientation = quaternion_orientation_from_eye_to_look_at(
                pose.position, LOOK_AT_TARGET
            )
        else:
            if FORCE_FRONT_XAXIS:  # XXX fix orientation
                assert np.isclose(pose.orientation.get_length(), 1.0)

                pose.orientation = negative_90_around_z * pose.orientation * positive_90_around_z
                x_axis, _, z_axis = AirSimNedTransform.local_axes_frame(pose, flip_z_axis=True)

                pose.orientation = quaternion_from_two_vectors(x_axis, z_axis) * pose.orientation
                x_axis, _, _ = AirSimNedTransform.local_axes_frame(pose, flip_z_axis=True)

                positive_180_around_x = quaternion_from_rotation_axis_angle(x_axis, np.deg2rad(180))
                pose.orientation = positive_180_around_x * pose.orientation
            else:
                assert False  # XXX orientation might be wrong if not FORCE_FRONT_XAXIS

        camera_poses.append(pose)

    if args.debug:
        camera_positions = [pose.position for pose in camera_poses]
        client.simPlotPoints(camera_positions, Rgba.Blue, is_persistent=True)
        client.simPlotLineStrip(camera_positions, Rgba.Cyan, thickness=2.5, is_persistent=True)

    airsim_record = []

    def do_stuff_at_uavmvs_viewpoint(i, pose):
        nonlocal client, camera_poses, airsim_record
        log_string = f"({i}/{len(camera_poses)})"
        p, q = pose.position, pose.orientation
        if args.debug:
            log_string += f" position = {to_xyz_str(p)}"
            client.simPlotTransforms([pose], scale=100, is_persistent=True)
            tl, tr, bl, br = viewport_vectors(pose, hfov_degrees=90, aspect_ratio=(16 / 9))
            viewport_points = frustum_plot_list_from_viewport_vectors(pose, tl, tr, bl, br)
            client.simPlotLineList(viewport_points, Rgba.White, thickness=3, is_persistent=True)
            # client.simPlotArrows([p], [LOOK_AT_TARGET], Rgba.White, thickness=3.0, duration=10)
        elif args.capture_dir:
            path = f"{args.prefix}pose{args.suffix}_{i:0{len(str(len(camera_poses)))}}.png"
            path = os.path.join(args.capture_dir, path)
            airsim.write_png(path, AirSimImage.get_mono(client, CAPTURE_CAMERA))
            log_string += f' saved image to "{path}"'
            record_line = AirSimRecord.make_line_string(p, q, time_stamp=str(i), image_file=path)
            airsim_record.append(record_line)
        ff.log(log_string)

    if IS_CV_MODE:
        for i, camera_pose in enumerate(camera_poses):
            client.simSetVehiclePose(camera_pose, ignore_collision=True)
            do_stuff_at_uavmvs_viewpoint(i, camera_pose)
            time.sleep(CV_SLEEP_SEC)
    else:
        client.moveToZAsync(z=-10, velocity=VELOCITY).join()  # XXX avoid colliding on take off
        client.hoverAsync().join()
        mean_position_error = 0.0

        for i, camera_pose in enumerate(camera_poses):
            client.moveToPositionAsync(
                *to_xyz_tuple(camera_pose.position),
                velocity=VELOCITY,
                drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=YawMode(is_rate=False, yaw_or_rate=YAW_N),
            ).join()

            with pose_at_simulation_pause(client) as real_pose:
                # NOTE when we pre-compute the viewpoint's camera orientation, we use the
                # expected drone position, which (should be close, but) is not the actual
                # drone position. Hence, we could experiment with using fake orientation:
                # quaternion_orientation_from_eye_to_look_at(real_pose.position, LOOK_AT_TARGET)
                fake_pose = Pose(real_pose.position, camera_pose.orientation)

                if CAPTURE_CAMERA == ff.CameraName.bottom_center:
                    fake_pose = Pose(real_pose.position, Quaternionr())  # XXX

                client.simSetVehiclePose(fake_pose, ignore_collision=True)
                client.simContinueForFrames(1)  # NOTE ensures pose change
                do_stuff_at_uavmvs_viewpoint(i, fake_pose)
                client.simSetVehiclePose(real_pose, ignore_collision=True)

                position_error = real_pose.position.distance_to(camera_pose.position)
                mean_position_error += position_error
                ff.log_debug(f"{position_error = }")

        mean_position_error /= len(camera_poses)
        ff.log_debug(f"{mean_position_error = }")

    if airsim_record:
        print_to_file = args.record_path is not None
        file = open(args.record_path, "w") if print_to_file else None
        print(AirSimRecord.make_header_string(), file=(file if print_to_file else sys.stdout))
        for line in airsim_record:
            print(line, file=(file if print_to_file else sys.stdout))
        if print_to_file:
            file.close()
            ff.log_info(f'Saved AirSim record to "{args.record_path}"')


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect(SIM_MODE)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done\n")
        ff.log_info(f"Used scale = {args.scale} and offset = {args.offset}")
        ff.log_info(f"Used VELOCITY = {VELOCITY}")
        ff.log_info(f"Used SIM_MODE = {SIM_MODE}")
        ff.log_info(
            f"Used LOOK_AT_TARGET = {'None' if LOOK_AT_TARGET is None else to_xyz_str(LOOK_AT_TARGET)}"
        )
        ff.log_info(f"Used CAPTURE_CAMERA = {CAPTURE_CAMERA}")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Trace a trajectory generated by uavmvs, "
        "but using different orientations at the viewpoints."
        " Works in either Multirotor or ComputerVision mode."
    )

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")
    parser.add_argument("--record_path", type=str, help="Path to save the recording .TXT file")
    parser.add_argument("--norecord_path", type=str, help="Suppress a use of --record_path (no-op)")

    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument("--debug", action="store_true", help="Skip capturing images but plot poses")

    parser.add_argument("--capture_dir", type=str, help="Folder where image captures will be saved")
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

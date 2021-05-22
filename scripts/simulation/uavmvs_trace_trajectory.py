import os
import time
import argparse

import ff
import airsim

from ds.rgba import Rgba
from ff.types import to_xyz_str, to_xyz_tuple
from ie.airsimy import (
    YAW_N,
    AirSimImage,
    AirSimRecord,
    connect,
    pose_at_simulation_pause,
    quaternion_orientation_from_eye_to_look_at,
)
from airsim.types import Pose, YawMode, DrivetrainType

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import parse_uavmvs, convert_uavmvs_to_airsim_position

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise

VELOCITY = 5

SIM_MODE = ff.SimMode.Multirotor
# SIM_MODE = ff.SimMode.ComputerVision

LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue
# LOOK_AT_TARGET = data_config.Ned.Urban_Building

CAPTURE_CAMERA = ff.CameraName.front_center
# CAPTURE_CAMERA = ff.CameraName.bottom_center

IS_CV_MODE = SIM_MODE == ff.SimMode.ComputerVision
CV_SLEEP_SEC = 2

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

    camera_poses = []
    for camera in args.trajectory:
        position = convert_uavmvs_to_airsim_position(
            camera.position, translation=args.offset, scaling=args.scale
        )
        orientation = quaternion_orientation_from_eye_to_look_at(position, LOOK_AT_TARGET)
        camera_poses.append(Pose(position, orientation))
    if args.debug:
        camera_positions = [pose.position for pose in camera_poses]
        client.simPlotPoints(camera_positions, Rgba.Blue, is_persistent=True)
        client.simPlotLineStrip(camera_positions, Rgba.White, thickness=1.5, is_persistent=True)
    camera_poses_len = len(camera_poses)

    airsim_record = []

    def do_stuff_at_uavmvs_viewpoint(i, pose):
        log_string = f"({i}/{camera_poses_len})"
        if args.debug:
            log_string += f" position = {to_xyz_str(pose.position)}"
            client.simPlotTransforms([pose], scale=100, is_persistent=True)
            client.simPlotArrows(
                [pose.position], [LOOK_AT_TARGET], Rgba.White, thickness=2.0, duration=10
            )
        elif args.capture_dir:
            path = f"{args.prefix}pose{args.suffix}_{i:0{len(str(camera_poses_len))}}.png"
            path = os.path.join(args.capture_dir, path)
            airsim.write_png(path, AirSimImage.get_mono(client, CAPTURE_CAMERA))
            log_string += f' saved image to "{path}"'
            record_line = AirSimRecord.make_line_string(
                position, orientation, time_stamp=str(i), image_file=path
            )
            airsim_record.append(record_line)
        ff.log(log_string)

    if not IS_CV_MODE:
        client.moveToZAsync(z=-10, velocity=VELOCITY).join()  # XXX avoid colliding on take off
        client.hoverAsync().join()

    mean_position_error = 0.0
    for i, camera_pose in enumerate(camera_poses):
        if IS_CV_MODE:
            client.simSetVehiclePose(camera_pose, ignore_collision=True)
            do_stuff_at_uavmvs_viewpoint(i, camera_pose)
            time.sleep(CV_SLEEP_SEC)
        else:
            client.moveToPositionAsync(
                *to_xyz_tuple(camera_pose.position),
                velocity=VELOCITY,
                drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=YawMode(is_rate=False, yaw_or_rate=YAW_N),
            ).join()
            with pose_at_simulation_pause(client) as real_pose:
                fake_pose = Pose(real_pose.position, camera_pose.orientation)
                client.simSetVehiclePose(fake_pose, ignore_collision=True)
                client.simContinueForFrames(1)  # NOTE ensures pose change
                do_stuff_at_uavmvs_viewpoint(i, fake_pose)
                client.simSetVehiclePose(real_pose, ignore_collision=True)

                # NOTE when we pre-compute the viewpoint's camera orientation, we use the
                # expected drone position, which (should be close, but) is not the actual
                # drone position. Hence, we could experiment with using fake orientation:
                # quaternion_orientation_from_eye_to_look_at(real_pose.position, LOOK_AT_TARGET)

                position_error = real_pose.position.distance_to(camera_pose.position)
                mean_position_error += position_error
                ff.log_debug(f"{position_error = }")

    if not IS_CV_MODE:
        ff.log_debug(f"{mean_position_error = }")

    if airsim_record:
        print()
        print(AirSimRecord.make_header_string())
        for line in airsim_record:
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
        ff.log_info(f"Used scale = {args.scale} and offset = {args.offset}")
        ff.log_info(f"Used VELOCITY = {VELOCITY}")
        ff.log_info(f"Used SIM_MODE = {SIM_MODE}")
        ff.log_info(f"Used LOOK_AT_TARGET = {LOOK_AT_TARGET}")
        ff.log_info(f"Used CAPTURE_CAMERA = {CAPTURE_CAMERA}")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fly to the positions of a trajectory generated by uavmvs"
        ", but using different orientations at the viewpoints."
        " Works in either Multirotor or ComputerVision mode."
    )

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

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

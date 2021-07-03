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
from airsim.types import Pose, YawMode, Vector3r, Quaternionr, DrivetrainType

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise

LOOK_AT_TARGET = None
# LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue
# LOOK_AT_TARGET = data_config.Ned.Urban_Building

CV_SLEEP_SEC = 0.0

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    path = args.trajectory_path
    assert os.path.isfile(path), path

    args.trajectory_recording = AirSimRecord.dict_from(path)
    if args.verbose:
        ff.log(f"The trajectory has {len(args.trajectory_recording)} camera poses")

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

    if args.flush or (args.capture_dir and not args.debug):
        client.simFlushPersistentMarkers()

    # NOTE see cv_plot_point_cloud.py and
    FORCE_FRONT_XAXIS = True  # XXX :ForceFrontXAxis:

    camera_poses = [
        Pose(record.position, record.orientation)
        for record in args.trajectory_recording.values()
    ]

    if args.debug:
        camera_positions = [pose.position for pose in camera_poses]
        if args.flush:
            client.simPlotPoints(camera_positions, Rgba.Blue, is_persistent=True)
            client.simPlotLineStrip(camera_positions, Rgba.Cyan, thickness=2.5, is_persistent=True)
        else:
            client.simPlotPoints(camera_positions, Rgba.Red, is_persistent=True)
            client.simPlotLineStrip(camera_positions, Rgba.Magenta, thickness=2.5, is_persistent=True)

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
            if args.flush:
                client.simPlotLineList(viewport_points, Rgba.White, thickness=3, is_persistent=True)
            else:
                client.simPlotLineList(viewport_points, Rgba.Black, thickness=3, is_persistent=True)
            # client.simPlotArrows([p], [LOOK_AT_TARGET], Rgba.White, thickness=3.0, duration=10)
        elif args.capture_dir:
            path = f"{args.prefix}pose{args.suffix}_{i:0{len(str(len(camera_poses)))}}_cv.png"  # XXX added '_cv'
            path = os.path.join(args.capture_dir, path)
            airsim.write_png(path, AirSimImage.get_mono(client, ff.CameraName.front_center))
            log_string += f' saved image to "{path}"'
            record_line = AirSimRecord.make_line_string(p, q, time_stamp=str(i), image_file=path)
            airsim_record.append(record_line)
        ff.log(log_string)

    for i, camera_pose in enumerate(camera_poses):
        client.simSetVehiclePose(camera_pose, ignore_collision=True)
        do_stuff_at_uavmvs_viewpoint(i, camera_pose)
        time.sleep(CV_SLEEP_SEC)

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
    client = connect(ff.SimMode.ComputerVision)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done\n")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Trace a trajectory saved in AirSim's recording .txt format."
    )

    parser.add_argument("trajectory_path", type=str, help="Path to a airsim_rec.txt file")
    parser.add_argument("--record_path", type=str, help="Path to save the recording .TXT file")
    parser.add_argument("--norecord_path", type=str, help="Suppress a use of --record_path (no-op)")

    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument("--debug", action="store_true", help="Skip capturing images but plot poses")

    parser.add_argument("--capture_dir", type=str, help="Folder where image captures will be saved")
    parser.add_argument("--prefix", type=str, help="Prefix added to output image names", default="")
    parser.add_argument("--suffix", type=str, help="Suffix added to output image names", default="")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

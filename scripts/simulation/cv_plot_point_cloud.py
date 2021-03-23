import os
import argparse

from enum import Enum
from typing import List

import ff
import airsim

from ds import Rgba, EditMode
from airsim import Pose, Vector3r, Quaternionr
from ff.helper import input_or_exit
from ie.airsimy import connect, quaternion_look_at

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "io_ply")
    from io_ply import read_ply

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import (
        parse_uavmvs,
        convert_uavmvs_to_airsim_pose,
        convert_uavmvs_to_airsim_position,
    )

    include(FF_PROJECT_ROOT, "scripts", "data", "data_config")
    import data_config as cfg
except:
    raise

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.ply_path), f"Invalid file path: '{args.ply_path}'"

    # Parse the point cloud into a pandas dataframe and extract its points.
    ply_df = read_ply(args.ply_path)
    if args.verbose:
        ff.log(ply_df)

    args.points = ply_df["points"][["x", "y", "z"]].to_numpy()
    n_of_points, _ = args.points.shape
    print(f"The point cloud has {n_of_points} points.")

    # NOTE avoid plotting all points for large clouds, since Unreal can't handle it.
    k = 0 if args.every_k is None else args.every_k
    while not k:
        try:
            k = int(input_or_exit("Do you want to plot one every how many points? "))
        except:
            k = 0
            print("Please input a valid integer. ", end="")
            continue

        answer = input_or_exit(
            f"This will plot a total of {n_of_points // k} points"
            f" (i.e. one every {k}).\nDo you want to continue? [Y/n] "
        )
        k = 0 if answer.lower().strip() in ["n", "no"] else k
    args.every_k = k

    # Check if a uavmvs trajectory was passed in and parse it into points.
    if args.trajectory_path is not None:
        _, ext = os.path.splitext(args.trajectory_path)
        assert os.path.isfile(args.trajectory_path), f"Invalid file path: '{args.trajectory_path}'"
        assert ext in parse_uavmvs.keys(), f"Invalid trajectory extension: '{args.trajectory_path}'"

        args.trajectory = parse_uavmvs[ext](args.trajectory_path)
        if args.verbose:
            ff.log(f"The trajectory has {len(args.trajectory)} camera poses")
    else:
        args.trajectory = None

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


CAMERA_POSE_SIZE = 12.0
TRAJECTORY_THICKNESS = 6.0
POINT_CLOUD_POINT_SIZE = 4.0


class ArrowKey(Enum):
    Up = 72
    Down = 80
    Left = 75
    Right = 77


def enter_edit_mode(client: airsim.MultirotorClient, points: List[Vector3r]) -> None:
    mode = EditMode.TRANSLATING
    step = 1.0
    total = {
        EditMode.TRANSLATING: Vector3r(0, 0, 0),
        # EditMode.ROTATING: Vector3r(0, 0, 0),
        EditMode.SCALING: Vector3r(1, 1, 1),
    }
    try:
        while True:
            key = ord(airsim.wait_key())
            if key == 224:  # arrow keys
                arrow_key = ArrowKey(ord(airsim.wait_key()))
                client.simFlushPersistentMarkers()
                if mode == EditMode.TRANSLATING:
                    delta = {
                        ArrowKey.Up: Vector3r(step, 0, 0),
                        ArrowKey.Down: Vector3r(-step, 0, 0),
                        ArrowKey.Left: Vector3r(0, -step, 0),
                        ArrowKey.Right: Vector3r(0, step, 0),
                    }[arrow_key]
                    points = [point + delta for point in points]
                    total[EditMode.TRANSLATING] += delta
                elif mode == EditMode.SCALING:
                    factor = {
                        ArrowKey.Up: 1 + step * 0.1,
                        ArrowKey.Down: 1 - step * 0.1,
                        ArrowKey.Left: 1 - step * 0.1,
                        ArrowKey.Right: 1 + step * 0.1,
                    }[arrow_key]
                    points = [point * factor for point in points]
                    total[EditMode.SCALING] *= factor
                else:
                    assert False  # TODO handle other edit modes for rotating and scaling
                client.simPlotPoints(points, Rgba.Blue, POINT_CLOUD_POINT_SIZE, is_persistent=True)
            elif key == 27:  # esc
                ff.log("TRANSLATING:", total[EditMode.TRANSLATING])
                ff.log("SCALING:", total[EditMode.SCALING])
                return
            else:
                if key == ord(b"["):
                    step /= 2.0
                elif key == ord(b"]"):
                    step *= 2.0
                elif key == ord(b"\t"):
                    mode = EditMode.next(mode)
                ff.log(f"{mode=} {step=}")
    except KeyboardInterrupt:
        return


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()

    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)

    if args.flush:
        client.simFlushPersistentMarkers()

    def transform(position):
        if args.scale is not None:
            position *= args.scale
        if args.offset is not None:
            position += Vector3r(*args.offset)
        return position

    points = [convert_uavmvs_to_airsim_position(_, transform) for _ in args.points[:: args.every_k]]
    client.simPlotPoints(points, Rgba.Blue, POINT_CLOUD_POINT_SIZE, is_persistent=True)

    if args.trajectory is not None:
        camera_poses = [convert_uavmvs_to_airsim_pose(_, transform) for _ in args.trajectory]
        camera_positions = [pose.position for pose in camera_poses]

        # XXX
        camera_poses = [
            # FIXME point to Cidadela_Statue
            Pose(pose.position, Quaternionr(0, 0, 0, w_val=1))
            # Pose(pose.position, quaternion_look_at(pose.position, cfg.Ned.Cidadela_Statue))
            # Pose(pose.position, (cfg.Ned.Cidadela_Statue - pose.position).to_Quaternionr().sgn())
            for pose in camera_poses
        ]
        for pose in camera_poses:
            print(ff.to_xyzw_str(pose.orientation))
        client.simPlotLineList(
            [_ for pose in camera_poses for _ in (pose.position, cfg.Ned.Cidadela_Statue)],
            Rgba.Magenta,
            TRAJECTORY_THICKNESS,
            duration=60,
        )
        # XXX

        # client.simPlotLineStrip(camera_positions, Rgba.Black, TRAJECTORY_THICKNESS, duration=10)
        # client.simPlotPoints(camera_positions, Rgba.White, CAMERA_POSE_SIZE, is_persistent=True)
        client.simPlotTransforms(camera_poses, 10 * CAMERA_POSE_SIZE, is_persistent=True)

    if args.edit:
        enter_edit_mode(client, points)


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
        ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot a .PLY point cloud in AirSim.")

    parser.add_argument("ply_path", type=str, help="Path to a .PLY file")

    parser.add_argument("--every_k", "-k", type=int, help="Plot one every k points")
    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument("--edit", action="store_true", help="Enter edit mode")

    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Offset added to all points  (e.g. --offset -55 11 1)",
    )
    parser.add_argument("--scale", type=float, help="Scale added to all points  (e.g. --scale 0.2)")

    parser.add_argument("--trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
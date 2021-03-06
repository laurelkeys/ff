import os
import argparse
from airsim.types import Vector3r

import ff
import airsim

from ds.rgba import Rgba
from ff.helper import input_or_exit
from ie.airsimy import connect

try:
    from include_in_path import include, FF_PROJECT_ROOT

    include(FF_PROJECT_ROOT, "misc", "tools", "io_ply")
    from io_ply import read_ply

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import parse_uavmvs
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
TRAJECTORY_THICKNESS = 8.0
POINT_CLOUD_POINT_SIZE = 4.0


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()

    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)

    if args.flush:
        client.simFlushPersistentMarkers()

    def from_numpy(vector):
        vector[2] *= -1.0
        return Vector3r(*map(float, vector))

    points = [from_numpy(point) for point in args.points[:: args.every_k]]
    client.simPlotPoints(points, Rgba.Blue, POINT_CLOUD_POINT_SIZE, is_persistent=True)

    if args.trajectory is not None:
        camera_positions = [from_numpy(camera.position) for camera in args.trajectory]
        client.simPlotPoints(camera_positions, Rgba.White, CAMERA_POSE_SIZE, is_persistent=True)
        client.simPlotLineStrip(camera_positions, Rgba.Black, TRAJECTORY_THICKNESS, duration=10)


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

    parser.add_argument("--trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

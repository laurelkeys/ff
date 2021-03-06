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
except:
    raise

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.ply_path), f"Invalid file path: '{args.ply_path}'"

    ply_data = read_ply(args.ply_path)  # pandas dataframe
    if args.verbose:
        ff.log(ply_data)

    args.points = ply_data["points"][["x", "y", "z"]].to_numpy()
    args.n_of_points, _ = args.points.shape

    print(f"The point cloud has {args.n_of_points} points.")

    k = 0 if args.k is None else args.k

    while not k:
        k = input_or_exit("Do you want to print one every how many points? ")
        try:
            k = int(k)
        except:
            print("Please input a valid integer. ", end="")
            k = 0
            continue
        answer = input_or_exit(
            f"This will print a total of {args.n_of_points // k} points"
            f" (i.e. one every {k}).\nDo you want to continue? [Y/n] "
        )
        if answer.lower().strip() in ["n", "no"]:
            k = 0
        print()

    args.every_k_points = k

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

    if args.flush:
        client.simFlushPersistentMarkers()

    def from_numpy(vector):
        vector[2] *= -1.0
        return Vector3r(*map(float, vector))

    points = [point for point in args.points[:: args.every_k_points]]

    client.simPlotPoints(
        [from_numpy(point) for point in points],
        Rgba.Red,
        size=2.5,
        is_persistent=True,
    )


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
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("ply_path", type=str, help="Path to a .PLY file")
    parser.add_argument("--k", type=int, help="Print one every k points")
    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    # parser.add_argument("--mesh", action="store_true", help="Use a mesh instead of a point cloud")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

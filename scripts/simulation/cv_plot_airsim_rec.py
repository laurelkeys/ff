import os
import argparse

import ff
import numpy as np
import airsim

from ds.rgba import Rgba
from ie.airsimy import AirSimRecord, connect
from airsim.types import Pose, Vector3r

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.rec), f"Invalid file path: '{args.rec}'"

    args.color = Rgba(*((0, 0.651, 0.929) if args.color is None else args.color))

    args.recording = AirSimRecord.dict_from(rec_file=args.rec)

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.flush:
        client.simFlushPersistentMarkers()

    if not args.transformation:
        poses = [Pose(record.position, record.orientation) for record in args.recording.values()]
        positions = [record.position for record in args.recording.values()]
    else:
        matrix = np.loadtxt(args.transformation)  # load the 4x4 transformation matrix
        print(matrix)
        poses = []
        positions = []
        for record in args.recording.values():
            pos = Vector3r(*np.matmul(matrix, np.append(record.position, 1)))
            poses.append(Pose(pos, record.orientation))
            positions = [pos]

    if args.axes:
        client.simPlotTransforms(poses, scale=100.0, thickness=2.5, is_persistent=True)
    else:
        client.simPlotPoints(positions, args.color, size=10, is_persistent=True)

    if args.lines:
        client.simPlotLineStrip(positions, args.color, thickness=2.5, is_persistent=True)


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

    # NOTE this is AirSim's `Recording` output
    parser.add_argument("rec", type=str, help="Path to airsim_rec.txt")
    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument("--lines", action="store_true", help="Plot trajectory lines")
    parser.add_argument("--axes", action="store_true", help="Plot transforms instead of points")
    parser.add_argument("--color", "-rgb", nargs=3, type=float, help="Plot's RGB color")
    parser.add_argument("--transformation", "-align", type=str, help="Path to a 4x4 transformation matrix file to be loaded with numpy")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

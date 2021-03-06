import os
import argparse

import ff

from ds.rgba import Rgba
from wrappers.airsimy import AirSimRecord, connect

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Pose


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.rec), f"Invalid file path: '{args.rec}'"

    args.recording = AirSimRecord.dict_from(rec_file=args.rec)

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

    poses = [Pose(record.position, record.orientation) for record in args.recording.values()]

    if args.plot_points:
        client.simPlotPoints(
            [pose.position for pose in poses], Rgba(0, 0.651, 0.929), size=10, is_persistent=True
        )
    else:
        client.simPlotTransforms(
            poses, scale=10.0, thickness=2.5, is_persistent=True,
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

    # NOTE this is AirSim's `Recording` output
    parser.add_argument("rec", type=str, help="Path to airsim_rec.txt")
    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument(
        "--plot_points", action="store_true", help="Show points instead of transforms"
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

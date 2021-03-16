import os
import time
import argparse

import ff
import airsim

from airsim import Vector3r
from ff.types import to_xyz_str
from ie.airsimy import AirSimImage, connect

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import parse_uavmvs, convert_uavmvs_to_airsim_pose
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
        assert os.path.isdir(args.capture_dir)

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

    transform = None if args.offset is None else lambda position: position + Vector3r(*args.offset)

    camera_poses = [convert_uavmvs_to_airsim_pose(camera, transform) for camera in args.trajectory]
    n_of_poses = len(camera_poses)
    pad = len(str(n_of_poses))

    for i, pose in enumerate(camera_poses):
        client.simSetVehiclePose(pose, ignore_collison=True)

        pose_str = f"{i:{pad}} / {n_of_poses}"
        position_str = to_xyz_str(pose.position, show_hints=False)
        if not args.capture_dir:
            ff.log(f"Going to pose ({pose_str}): {position_str}")
        else:
            name = os.path.join(args.capture_dir, f"{args.prefix}pose{args.suffix}_{i:0{pad}}.png")
            airsim.write_png(name, AirSimImage.get_mono(client, ff.CameraName.bottom_center))
            ff.log(f"Saved image ({pose_str}) to '{name}' at {position_str}")

        time.sleep(args.sleep)


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
    parser = argparse.ArgumentParser(
        description="Follow a trajectory generated by uavmvs in AirSim's CV mode."
    )

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

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
        help="Offset added to all points  (e.g. --offset -55 11 1)",
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

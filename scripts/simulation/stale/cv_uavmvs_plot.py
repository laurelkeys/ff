import os
import argparse

import ff
import numpy as np
import airsim

from airsim import Pose, Vector3r, Quaternionr
from ie.airsimy import connect

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import (
        TrajectoryCamera,
        TrajectoryCameraKind,
        parse_uavmvs,
        convert_uavmvs_to_airsim_pose,
        convert_uavmvs_to_airsim_position,
    )
except:
    raise

# NOTE this is similar to cv_plot_point_cloud.py, but instead of plotting the camera
# positions directly (i.e. a list of 3D points), we do so by starting at the first point
# and from there on out finding the next positions by "walking a distance along a direction".

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

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


CAMERA_POSE_SIZE = 12.0
TRAJECTORY_THICKNESS = 6.0


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    ##initial_pose = client.simGetVehiclePose()
    ##if args.verbose:
    ##    ff.print_pose(initial_pose, airsim.to_eularian_angles)

    ##if args.flush:
    ##    client.simFlushPersistentMarkers()

    # TODO compute distances and orientations in uavmvs' coordinates and then transform
    # them to AirSim, but also try transforming them first and doing the computations directly
    # on AirSim's coordinate system (if everything is correct, they should be the same).

    ##
    def quat_to_rot(q):
        assert q.shape == (4,)
        xxzz = q[1] ** 2 - q[3] ** 2
        rryy = q[0] ** 2 - q[2] ** 2
        yyrrxxzz = q[2] ** 2 + q[0] ** 2 - q[1] ** 2 - q[3] ** 2

        xr2 = q[1] * q[0] * 2
        xy2 = q[1] * q[2] * 2
        xz2 = q[1] * q[3] * 2
        yr2 = q[2] * q[0] * 2
        yz2 = q[2] * q[3] * 2
        zr2 = q[3] * q[0] * 2

        return np.array(
            [
                [(xxzz + rryy), (xy2 - zr2), ( xz2 + yr2 )],
                [( xy2 + zr2 ), (yyrrxxzz ), ( yz2 - xr2 )],
                [( xz2 - yr2 ), (yz2 + xr2), (rryy - xxzz)],
            ]
        )

    C = np.cos(np.pi / 4)  # == np.sqrt(2) / 2
    args.trajectory = [  # args.trajectory.extend(
        TrajectoryCamera(np.array([0, 0, 0]), np.array([1, 0, 0, 0]), kind=TrajectoryCameraKind.Csv),
        TrajectoryCamera(np.array([0, 0, 5]), quat_to_rot(np.array([1, 0, 0, 0])), kind=TrajectoryCameraKind.Traj),
        TrajectoryCamera(np.array([0, 0, 1]), np.array([0, 1, 0, 0]), kind=TrajectoryCameraKind.Csv),
        TrajectoryCamera(np.array([0, 0, 6]), quat_to_rot(np.array([0, 1, 0, 0])), kind=TrajectoryCameraKind.Traj),
        TrajectoryCamera(np.array([0, 0, 2]), np.array([C, C, 0, 0]), kind=TrajectoryCameraKind.Csv),
        TrajectoryCamera(np.array([0, 0, 7]), quat_to_rot(np.array([C, C, 0, 0])), kind=TrajectoryCameraKind.Traj),
        TrajectoryCamera(np.array([0, 0, 3]), np.array([C, 0, C, 0]), kind=TrajectoryCameraKind.Csv),
        TrajectoryCamera(np.array([0, 0, 8]), quat_to_rot(np.array([C, 0, C, 0])), kind=TrajectoryCameraKind.Traj),
        TrajectoryCamera(np.array([0, 0, 4]), np.array([C, 0, 0, C]), kind=TrajectoryCameraKind.Csv),
        TrajectoryCamera(np.array([0, 0, 9]), quat_to_rot(np.array([C, 0, 0, C])), kind=TrajectoryCameraKind.Traj),

    ]
    ##

    camera_poses, camera_positions = [], []
    for i, camera in enumerate(args.trajectory):
        if not camera.spline_interpolated:  # XXX
            pose = convert_uavmvs_to_airsim_pose(
                camera=camera, translation=args.offset, scaling=args.scale
            )
            print(f"---- {i}")
            print(f"{camera.position  = }")
            print(f"{camera.rotation  = }")
            print(f"{pose.position    = }")
            print(f"{pose.orientation = }")
            camera_poses.append(pose)
            camera_positions.append(pose.position)

    ##client.simPlotLineStrip(camera_positions, Rgba.Black, TRAJECTORY_THICKNESS, is_persistent=True)
    ### client.simPlotPoints(camera_positions, Rgba.White, CAMERA_POSE_SIZE, is_persistent=True)
    ##client.simPlotTransforms(camera_poses, 10 * CAMERA_POSE_SIZE, is_persistent=True)


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    ##client = connect(ff.SimMode.ComputerVision)
    client = None
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        ##client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
        pass
    finally:
        ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot a uavmvs trajectory in AirSim.")

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

    parser.add_argument("--flush", action="store_true", help="Flush old plots")

    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Offset added to all points  (e.g. --offset -55 11 1)",
    )
    parser.add_argument("--scale", type=float, help="Scale added to all points  (e.g. --scale 0.2)")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

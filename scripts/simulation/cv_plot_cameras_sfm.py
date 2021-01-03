import os
import argparse

import ff
import numpy as np
import airsim

from ds.rgba import Rgba
from ie.airsimy import connect
from airsim.types import Pose, Vector3r, Quaternionr
from ie.meshroomy import MeshroomParser, MeshroomTransform

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.sfm), f"Invalid file path: '{args.sfm}'"

    # TODO get transforms, i.e. poses (position + orientation) from each camera
    views, poses = MeshroomParser.parse_cameras(cameras_file_path=args.sfm)
    args.views_dict, args.poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

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

    # FIXME move this to wrappers/
    def pose_from_meshroom_to_airsim(meshroom_pose):
        rotation = meshroom_pose.rotation
        center = meshroom_pose.center

        assert len(rotation) == 9 and len(center) == 3, meshroom_pose

        xywz = MeshroomTransform.rotation(rotation, as_xywz_quaternion=True)

        return Pose(
            position_val=Vector3r(center[0], center[1], center[2]),
            orientation_val=Quaternionr(xywz[3], xywz[0], xywz[1], xywz[2]),
        )

    poses = [pose_from_meshroom_to_airsim(pose) for pose in args.poses_dict.values()]

    # TODO plot them and compare with cv_plot_airsim_rec.py
    if args.plot_points:
        client.simPlotPoints(
            [pose.position for pose in poses], Rgba(1, 0.706, 0), size=10, is_persistent=True
        )
    else:
        client.simPlotTransforms(
            poses,
            scale=7.5,
            thickness=2.5,
            is_persistent=True,
        )

    # TODO compute the transformation matrix that aligns Meshroom's reference system with AirSim's
    # TODO compare the aligned camera transforms (i.e. how good is the pose estimation?)
    # TODO test Meshroom's `matchFromKnownCameraPoses`?
    # TODO can we match Meshroom's image filenames with the ones stored in airsim_rec.txt?

    if args.transformation:
        meshroom_to_airsim = np.loadtxt(args.transformation)  # load the 4x4 transformation matrix
        print(meshroom_to_airsim)

        def align_meshroom_to_airsim(meshroom_pose):
            # NOTE this transformation is only based on the positions (and not on the orientations)
            meshroom_pos = np.append(
                meshroom_pose.position.to_numpy_array(), 1
            )  # [x, y, z, 1] homogeneous coordinates
            airsim_pos = np.matmul(
                meshroom_to_airsim, meshroom_pos
            )  # meshroom_to_airsim @ meshroom_pos
            return Pose(
                position_val=Vector3r(airsim_pos[0], airsim_pos[1], airsim_pos[2]),
                orientation_val=meshroom_pose.orientation,
            )

        aligned_poses = [align_meshroom_to_airsim(pose) for pose in poses]

        client.simPlotTransforms(
            aligned_poses,
            scale=7.5,
            thickness=2.5,
            is_persistent=True,
        )

        # FIXME temporary testing to compare with cv_plot_airsim_rec.py
        # client.simFlushPersistentMarkers()
        # client.simPlotPoints([pose.position for pose in aligned_poses], color_rgba=[1, 0.706, 0, 1.0], is_persistent=True)


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

    # NOTE this is Meshroom's `StructureFromMotion` node output
    parser.add_argument("sfm", type=str, help="Path to cameras.sfm")
    parser.add_argument("--flush", action="store_true", help="Flush old plots")
    parser.add_argument("--plot_points", action="store_true", help="Show points instead of transforms")
    parser.add_argument("--transformation", type=str, help="Path to a 4x4 transformation matrix file to be loaded with numpy")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

import os
import argparse

import ff

from wrappers.meshroomy import MeshroomParser, MeshroomTransform

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Pose, Vector3r, Quaternionr


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.sfm), f"Invalid file path: '{args.sfm}'"

    # TODO get transforms (i.e. poses / position + orientation from each camera)
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

    client.simFlushPersistentMarkers()

    # FIXME move this to wrappers/
    def pose_from_meshroom_to_airsim(meshroom_pose):
        rotation = meshroom_pose.rotation
        center = meshroom_pose.center

        assert len(rotation) == 9 and len(center) == 3, meshroom_pose

        xywz = MeshroomTransform.rotation(rotation, as_quaternion=True)  # FIXME

        return Pose(
            position_val=Vector3r(center[0], center[1], center[2]),
            orientation_val=Quaternionr(xywz[3], xywz[0], xywz[1], xywz[2]),
        )

    # TODO plot them and compare with cv_plot_airsim_rec.py
    client.simPlotTransforms(
        poses=[pose_from_meshroom_to_airsim(pose) for pose in args.poses_dict.values()],
        scale=7.5,
        thickness=2.5,
        is_persistent=True,
    )

    # TODO compute the transformation matrix that aligns Meshroom's reference system with AirSim's
    # TODO compare the aligned camera transforms (i.e. how good is the pose estimation?)
    # TODO test Meshroom's `matchFromKnownCameraPoses`?
    # TODO can we match Meshroom's image filenames with the ones stored in airsim_rec.txt?


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done")


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    # NOTE don't `enableApiControl` or `armDisarm` since we are in CV mode
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    # NOTE Meshroom's `StructureFromMotion` node output
    parser.add_argument("sfm", type=str, help="Path to cameras.sfm")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

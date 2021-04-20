import argparse

import ff
import numpy as np
import airsim

from ie import airsimy
from ds.rgba import Rgba
from ie.airsimy import connect, matrix_from_eularian_angles, quaternion_from_two_vectors
from airsim.types import Pose, Vector3r
from airsim.utils import to_quaternion, to_eularian_angles

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


O = Vector3r(0, 0, 0)
X = Vector3r(1, 0, 0)
Y = Vector3r(0, 1, 0)
Z = Vector3r(0, 0, 1)

LOOK_AT_TARGET = data_config.Ned.Cidadela_Statue


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.reset: client.reset()
    client.simFlushPersistentMarkers()

    def plot_transform(p, q, scale=1.0, thickness=1.0, duration = -1.0, is_persistent = False):
        pitch, roll, yaw = to_eularian_angles(q)
        rotation_matrix = matrix_from_eularian_angles(pitch, roll, yaw)
        x_axis = rotation_matrix[0, :]
        y_axis = rotation_matrix[1, :]
        z_axis = rotation_matrix[2, :]
        client.simPlotArrows([p], )
        simPlotTransforms(self, poses, scale = 5.0, thickness = 5.0, duration = -1.0, is_persistent = False):

    with airsimy.paused_simulation(client) as pose:
        p = pose.position
        q = pose.orientation

        client.simPlotTransforms([pose], scale=100, is_persistent=True)
        # client.simPlotArrows([p], [LOOK_AT_TARGET], Rgba.White, is_persistent=True)

        #
        # Position
        #

        client.simPlotArrows([O], [O + X], Rgba.Red, is_persistent=True)
        client.simPlotArrows([O], [O + Y], Rgba.Green, is_persistent=True)
        client.simPlotArrows([O], [O + Z], Rgba.Blue, is_persistent=True)

        # front = client.simGetCameraInfo(ff.CameraName.front_center)
        # bottom = client.simGetCameraInfo(ff.CameraName.bottom_center)
        # client.simSetCameraPose(ff.CameraName.front_center, pose)
        # client.simSetCameraPose(ff.CameraName.bottom_center, bottom.pose)

        #
        # Orientation
        #

        pitch, roll, yaw = to_eularian_angles(q)
        pitch += np.deg2rad(args.pitch)  # rotates X^Z in UE4 coordinate system
        roll += np.deg2rad(args.roll)  # rotates Z^Y in UE4 coordinate system
        yaw += np.deg2rad(args.yaw)  # rotates X^Y in UE4 coordinate system
        pose.orientation = to_quaternion(pitch, roll, yaw)

        pose.orientation = quaternion_from_two_vectors(airsimy.FRONT, LOOK_AT_TARGET - airsimy.FRONT)

        client.simPlotTransforms([pose], scale=200, thickness=2.5, is_persistent=True)
        # client.simSetVehiclePose(pose, ignore_collison=True)


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
        description="Set a specific camera orientation in ComputerVision mode."
    )
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("pitch", nargs="?", default=0.0, type=float)
    parser.add_argument("roll", nargs="?", default=0.0, type=float)
    parser.add_argument("yaw", nargs="?", default=0.0, type=float)
    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

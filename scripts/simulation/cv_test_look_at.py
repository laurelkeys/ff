import argparse
from typing import Tuple

import ff
import numpy as np
import airsim

from ie import airsimy
from ds.rgba import Rgba
from ie.airsimy import connect
from airsim.types import Pose, Vector3r, Quaternionr
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

TEST_POSE = Pose(
    Vector3r(x_val=-13.793405532836914, y_val=2.257002115249634, z_val=-6.261967182159424),
    Quaternionr(
        x_val=-0.020065542310476303,
        y_val=-0.14743053913116455,
        z_val=-0.02142292633652687,
        w_val=0.9886367917060852,
    ),
)


def get_xyz_axis(pose: Pose, flip_z: bool = False) -> Tuple[Vector3r, Vector3r, Vector3r]:
    q = pose.orientation
    x_axis = airsimy.vector_rotated_by_quaternion(X, q)
    y_axis = airsimy.vector_rotated_by_quaternion(Y, q)
    z_axis = airsimy.vector_rotated_by_quaternion(Z, q)
    if flip_z:
        z_axis *= -1
    return x_axis, y_axis, z_axis


def plot_xyz_axis(
    client: airsim.MultirotorClient,
    x_axis: Vector3r,
    y_axis: Vector3r,
    z_axis: Vector3r,
    origin: Vector3r,
    normalize: bool = False,
    thickness: float = 2.5,
) -> None:
    r, g, b = Rgba.Red, Rgba.Green, Rgba.Blue
    if normalize:
        r, g, b = Rgba.Cyan, Rgba.Magenta, Rgba.Yellow
        x_axis = x_axis / x_axis.get_length()
        y_axis = y_axis / y_axis.get_length()
        z_axis = z_axis / z_axis.get_length()
    client.simPlotArrows([origin], [origin + x_axis], r, thickness=thickness, is_persistent=True)
    client.simPlotArrows([origin], [origin + y_axis], g, thickness=thickness, is_persistent=True)
    client.simPlotArrows([origin], [origin + z_axis], b, thickness=thickness, is_persistent=True)


def plot_pose(client: airsim.MultirotorClient, pose: Pose, flip_z: bool = False) -> None:
    # NOTE setting flip_z=True gives the same result as client.simPlotTransforms([pose])
    x_axis, y_axis, z_axis = get_xyz_axis(pose, flip_z)
    plot_xyz_axis(client, x_axis, y_axis, z_axis, origin=pose.position, thickness=2.0)


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.reset:
        client.reset()
    client.simFlushPersistentMarkers()

    plot_xyz_axis(client, X, Y, Z, origin=O)

    # client.simPlotTransforms([TEST_POSE], scale=110, thickness=1.0, is_persistent=True)
    # plot_pose(client, TEST_POSE)

    with airsimy.pose_at_simulation_pause(client) as pose:
        p, q = pose.position, pose.orientation
        print(f"{p = }")
        print(f"{q = }")

        client.simPlotArrows([p], [LOOK_AT_TARGET], Rgba.White, is_persistent=True)
        # client.simPlotTransforms([pose], scale=110, thickness=1.0, is_persistent=True)
        plot_pose(client, pose)

        # NOTE use x' = (LOOK_AT_TARGET - p) as the new x-axis (i.e. front vector),
        # and project the current up/down vector (z-axis in AirSim) into the plane
        # that is normal to x' at point p. This way we can get the remaining right
        # vector by computing cross(down, front).

        x_prime = LOOK_AT_TARGET - p
        _, _, z_axis = get_xyz_axis(pose, flip_z=False)
        z_prime = airsimy.vector_projected_onto_plane(z_axis, plane_normal=x_prime)
        y_prime = z_prime.cross(x_prime)

        plot_xyz_axis(client, x_prime, y_prime, z_prime, origin=p, normalize=True)


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
    # parser.add_argument("pitch", nargs="?", default=0.0, type=float)
    # parser.add_argument("roll", nargs="?", default=0.0, type=float)
    # parser.add_argument("yaw", nargs="?", default=0.0, type=float)
    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

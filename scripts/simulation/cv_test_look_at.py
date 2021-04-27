import argparse

import ff
import airsim

from ie import airsimy
from ds.rgba import Rgba
from ie.airsimy import connect, AirSimNedTransform
from airsim.types import Pose, Vector3r, Quaternionr

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

RGB = (Rgba.Red, Rgba.Green, Rgba.Blue)
CMY = (Rgba.Cyan, Rgba.Magenta, Rgba.Yellow)

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


def plot_xyz_axis(
    client: airsim.MultirotorClient,
    x_axis: Vector3r,
    y_axis: Vector3r,
    z_axis: Vector3r,
    origin: Vector3r,
    colors=RGB,
    thickness=2.5,
) -> None:
    client.simPlotArrows([origin], [origin + x_axis], colors[0], thickness, is_persistent=True)
    client.simPlotArrows([origin], [origin + y_axis], colors[1], thickness, is_persistent=True)
    client.simPlotArrows([origin], [origin + z_axis], colors[2], thickness, is_persistent=True)


def plot_pose(client: airsim.MultirotorClient, pose: Pose) -> None:
    x_axis, y_axis, z_axis = AirSimNedTransform.local_axes_frame(pose)
    plot_xyz_axis(client, x_axis, y_axis, z_axis, origin=pose.position)
    # client.simPlotTransforms([pose], scale=110, thickness=1.0, is_persistent=True)


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.reset:
        client.reset()
    client.simFlushPersistentMarkers()

    plot_pose(client, TEST_POSE)
    plot_xyz_axis(client, X, Y, Z, origin=O)

    with airsimy.pose_at_simulation_pause(client) as pose:
        plot_pose(client, pose)

        client.simPlotArrows([pose.position], [LOOK_AT_TARGET], Rgba.White, is_persistent=True)

        # NOTE use x' = (LOOK_AT_TARGET - p) as the new x-axis (i.e. front vector),
        # and project the current up/down vector (z-axis in AirSim) into the plane
        # that is normal to x' at point p. This way we can get the remaining right
        # vector by computing cross(down, front).

        x_prime = LOOK_AT_TARGET - pose.position
        _, _, z_axis = AirSimNedTransform.local_axes_frame(pose)
        z_prime = airsimy.vector_projected_onto_plane(z_axis, plane_normal=x_prime)

        # NOTE don't forget to normalize! Not doing so will break the orientation below.
        x_prime /= x_prime.get_length()
        z_prime /= z_prime.get_length()

        y_prime = z_prime.cross(x_prime)

        plot_xyz_axis(
            client,
            x_prime * 1.25,
            y_prime * 1.25,
            z_prime * 1.25,
            origin=pose.position,
            colors=CMY,
            thickness=1.5,
        )

        # Now, find the orientation that corresponds to the x'-y'-z' axis frame:
        new_pose = Pose(
            pose.position,
            airsimy.quaternion_that_rotates_axes_frame(
                source_xyz_axes=(X, Y, Z),
                target_xyz_axes=(x_prime, y_prime, z_prime),
            ),
        )
        plot_pose(client, new_pose)  # NOTE this should be the same as the plot_xyz_axis above!
        if args.set:
            client.simSetVehiclePose(new_pose, ignore_collison=True)


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
    parser.add_argument("--set", action="store_true")
    # parser.add_argument("pitch", nargs="?", default=0.0, type=float)
    # parser.add_argument("roll", nargs="?", default=0.0, type=float)
    # parser.add_argument("yaw", nargs="?", default=0.0, type=float)
    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

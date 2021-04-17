import os
import argparse
from airsim.types import DrivetrainType, Vector3r, YawMode

import ff
import airsim
from ff.types import to_xyz_str

from ie.airsimy import connect

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import parse_uavmvs, convert_uavmvs_to_airsim_position

    include(FF_PROJECT_ROOT, "scripts", "data_config")
    import data_config
except:
    raise

# NOTE much from this file is simply copy-pasted from cv_trace_uavmvs_trajectory.py

###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    _, ext = os.path.splitext(args.trajectory_path)
    assert os.path.isfile(args.trajectory_path), f"Invalid file path: '{args.trajectory_path}'"
    assert ext in parse_uavmvs.keys(), f"Invalid trajectory extension: '{args.trajectory_path}'"

    args.trajectory = parse_uavmvs[ext](args.trajectory_path)

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


TEST_AIMING_AT_ROI = True  # HACK improve this after testing


YAW_N = 0
YAW_E = 90
YAW_W = -90
YAW_S = 180


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    client.moveToZAsync(z=-10, velocity=2).join()  # XXX avoid colliding
    client.hoverAsync().join()

    # NOTE (at least for now) don't worry about matching the camera rotation optimized
    # by uavmvs, simply follow the generated viewpoint positions with the drone.
    camera_positions = [
        convert_uavmvs_to_airsim_position(
            camera.position, translation=args.offset, scaling=args.scale
        )
        for camera in args.trajectory
    ]

    def print_record_line(kinematics):
        # Reference: AirSim/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp
        pos_x, pos_y, pos_z = ff.to_xyz_tuple(kinematics.position)
        q_x, q_y, q_z, q_w = ff.to_xyzw_tuple(kinematics.orientation)
        print(pos_x, pos_y, pos_z, q_w, q_x, q_y, q_z, sep="\t")

    print("POS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z")
    for position in camera_positions:
        client.moveToPositionAsync(
            *ff.to_xyz_tuple(position),
            velocity=2,
            drivetrain=DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=YawMode(is_rate=False, yaw_or_rate=-90),
        ).join()

        client.simPause(True)
        kinematics = client.simGetGroundTruthKinematics()
        client.simPause(False)
        print_record_line(kinematics)

        def plot_arrow(unit_vector, r, g, b, scale=1.2):
            client.simPlotArrows(
                [kinematics.position],
                [kinematics.position + unit_vector * scale],
                color_rgba=[r, g, b, 1.0],
                duration=5,
            )

        plot_arrow(Vector3r(1, 0, 0), 1, 0, 0)
        plot_arrow(Vector3r(0, 1, 0), 0, 1, 0)
        plot_arrow(Vector3r(0, 0, 1), 0, 0, 1)

        if TEST_AIMING_AT_ROI:
            body_to_roi = data_config.Ned.Cidadela_Statue - kinematics.position
            unit_body_to_roi = body_to_roi / body_to_roi.get_length()
            # ff.log_info(f"{to_xyz_str(body_to_roi)} (unit = {to_xyz_str(unit_body_to_roi)})")
            plot_arrow(unit_body_to_roi, 1, 0, 1)



###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect(ff.SimMode.Multirotor)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        pass  # ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Follow a trajectory generated by uavmvs in AirSim's Multirotor mode."
        " The drone position and orientation are printed after reaching every view point."
    )  # NOTE the output can be parsed to generate (TUM) .log trajectory files for ATE/RPE

    parser.add_argument("trajectory_path", type=str, help="Path to a .TRAJ, .CSV or .UTJ file")

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

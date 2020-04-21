import os
import sys
import time
import msvcrt
import argparse

import ff

try:
    import airsim
except ModuleNotFoundError:
    airsim_path = ff.Default.AIRSIM_CLIENT_PATH
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))
    import airsim

from ff.types import to_xyz_str, to_xyzw_str, angles_to_str


###############################################################################
# preflight (called before connecting) ########################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    args.flight_velocity = 2
    import viewpoints.default  # FIXME
    args.viewpoints = zip(viewpoints.default.Positions, viewpoints.default.Orientations)


###############################################################################
# fly (called after connecting) ###############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    initial_pose = client.simGetVehiclePose()
    initial_state = client.getMultirotorState()

    if args.verbose:
        print(
            "[ff] VehiclePose \n"
            f"     .position    = {to_xyz_str(initial_pose.position)}\n"
            f"     .orientation = {to_xyzw_str(initial_pose.orientation)}\n"
            f"                    {angles_to_str(airsim.to_eularian_angles(initial_pose.orientation))}\n"
        )

    if initial_state.landed_state == airsim.LandedState.Landed:
        print("[ff] Taking off")
        client.takeoffAsync(timeout_sec=8).join()
    else:
        client.hoverAsync().join()  # airsim.LandedState.Flying

    print("[ff] Flying viewpoints...")
    print("[ff] Press [space] to take pictures (or any other key to stop)")  # TODO
    future = _move_by_path(client, args)

    img_count = 0
    while True:
        if msvcrt.kbhit():
            if msvcrt.getch() != b" ":
                break  # https://stackoverflow.com/a/13207813

            response, *_  = client.simGetImages([airsim.ImageRequest(
                ff.CameraName.front_center,
                airsim.ImageType.Scene,
                False, True  # compressed PNG image
            )])
            img_count += 1
            print(f"     {img_count} pictures taken", end="\r")
            airsim.write_file(os.path.join(args.output_folder, f"out_{img_count}.png"), response.image_data_uint8)
            # TODO save poses to .log file
            print("camera_position:", response.camera_position) # Vector3r
            print("camera_orientation:", response.camera_orientation) # Quaternionr
    print()
    
    print(f"[ff] Waiting for drone to finish path...", end=" ", flush=True)
    future.join()
    print("done")

    time.sleep(4)
    client.reset()
    print("[ff] Drone reset")


def _move_by_path(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    path = [airsim.Vector3r(*position) for position, _orientation in args.viewpoints]
    future = client.moveOnPathAsync(
        path, args.flight_velocity,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=-1.5)  # FIXME
    )
    return future


def _move_by_positions(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    for position, orientation in args.viewpoints:
        _pitch, _roll, yaw = airsim.to_eularian_angles(airsim.Quaternionr(*orientation))
        
        client.moveToPositionAsync(
            *position,
            args.flight_velocity,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw)
        ).join()
        
        client.hoverAsync().join()
        time.sleep(2)


def _move_by_vehicle_poses(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    raise Warning("simSetVehiclePose() is meant for ComputerVision mode")
    # NOTE check https://github.com/microsoft/AirSim/pull/2324
    for position, orientation in args.viewpoints:
        pose = airsim.Pose(airsim.Vector3r(*position), airsim.Quaternionr(*orientation))
        client.simSetVehiclePose(pose, ignore_collison=True)
        client.hoverAsync().join()
        time.sleep(2)


###############################################################################
# main ########################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        path_str = f"'airsim' path: {airsim.__path__[0]}"
        print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    if args.env_name is not None:
        ff.launch_env(args)  # the --launch option was passed
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset


def connect_to_airsim(vehicle_name: str = None) -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    if vehicle_name is not None:
        client.enableApiControl(True, vehicle_name)
        client.armDisarm(True, vehicle_name)
    else:
        client.enableApiControl(True)
        client.armDisarm(True)
    return client


###############################################################################
# argument parsing ############################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fly viewpoints capturing images")

    parser.add_argument(
        "--out",
        dest="output_folder",
        metavar="OUTPUT_FOLDER",
        type=str,
        default="tmp",
        help="Image output folder  (default: %(default)s/)",
    )

    parser.add_argument(
        "--viewpoints_path",
        type=str,
        default=os.path.join("viewpoints", "default.py"),
        help="Path to the viewpoints file  (default: %(default)s)",
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

# See https://github.com/microsoft/AirSim/blob/master/docs/apis.md#apis-for-multirotor
#     https://github.com/microsoft/AirSim/blob/master/docs/image_apis.md#python-1

import os
import sys
import time
import argparse
import subprocess

import numpy as np

try:
    import airsim
except ModuleNotFoundError:
    pass  # don't worry, it'll be imported later

###############################################################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    # NOTE avoid velocities over 5 m/s
    args.velocity = 2


###############################################################################
###############################################################################


def get_NED_coords(
    client: airsim.MultirotorClient, multirotor_state: airsim.MultirotorState = None
) -> airsim.Vector3r:
    ''' `(north, east, down)` values in `m`, following the right-hand rule '''
    if multirotor_state is None:
        multirotor_state = client.getMultirotorState()
    return multirotor_state.kinematics_estimated.position


def get_UE4_coords(
    client: airsim.MultirotorClient, multirotor_state: airsim.MultirotorState = None
) -> airsim.Vector3r:
    ''' `(north, east, up)` values in `cm`, following the left-hand rule '''
    if multirotor_state is None:
        multirotor_state = client.getMultirotorState()
    pos = multirotor_state.gps_location  # GeoPoint
    return airsim.Vector3r(pos.latitude, pos.longitude, pos.altitude)


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.z is not None and args.z > 0:
        print("Warning: AirSim uses NED coordinates, meaning +z is down")
        print("         (to fly upwards use negative z values)")

    state = client.getMultirotorState()
    home_position = get_NED_coords(client, state)  # NED starting point coordinates ~ (0, 0, 0)
    player_start = get_UE4_coords(client, state)   # UE4 PlayerStart coordinates

    if args.verbose:
        print("home_position (NED):", home_position, "\n")
        print("player_start (UE4):", player_start, "\n")

    if args.relative:
        xyz = (
            home_position.x_val + args.x,
            home_position.y_val + args.y,
            home_position.z_val + (0 if args.z is None else args.z),
        )
    else:
        if args.z is None:
            args.z = home_position.z_val  # keep the same altitude
        xyz = (args.x, args.y, args.z)

    if state.landed_state == airsim.LandedState.Landed:
        print(f"landed_state = {state.landed_state} (Landed)")
        print("[ff] Taking off.. ", end="", flush=True)
        client.takeoffAsync().join()
        print("done.")
    else:  # airsim.LandedState.Flying
        print(f"landed_state = {state.landed_state} (Flying)")
        print("[ff] Hovering", flush=True)
        client.hoverAsync().join()

    if args.teleport:
        print(f"[ff] Teleporting to {xyz}.. ", end="", flush=True)
        pose = airsim.Pose()
        pose.position = airsim.Vector3r(*xyz)
        client.simSetVehiclePose(pose, ignore_collison=True)
        time.sleep(4)  # wait a few seconds after teleporting
    else:
        print(f"[ff] Moving to {xyz}.. ", end="", flush=True)
        client.moveToPositionAsync(*xyz, args.velocity).join()
    print("done.")

    state = client.getMultirotorState()
    final_xyz = get_NED_coords(client, state)
    final_xyz = (final_xyz.x_val, final_xyz.y_val, final_xyz.z_val)
    print(f"[ff] Reached {final_xyz}.")

    if args.verbose:
        print("\nKinematics estimated position:", state.kinematics_estimated.position)
        print("GPS location (UE4 coordinates):", state.gps_location, "\n")

    client.hoverAsync().join()
    print(f"[ff] Hovering for {args.wait_sec} seconds.. ", end="", flush=True)
    time.sleep(args.wait_sec)  # hover for some time before teleporting back
    print("done.")

    if args.verbose:
        print("\nKinematics estimated position:", state.kinematics_estimated.position)
        print("GPS location (UE4 coordinates):", state.gps_location, "\n")


###############################################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Change the drone's position (uses NED coordinates)")

    parser.add_argument(
        "x", type=float, help="Coordinate value in meters, where +x is north"
    )
    parser.add_argument(
        "y", type=float, help="Coordinate value in meters, where +y is east"
    )
    parser.add_argument(
        "z", type=float, help="Coordinate value in meters, where +z is down (kept the same by default)",
        nargs="?",
        default=None
    )

    parser.add_argument(
        "--relative",
        action="store_true",
        help="Move relative to the current drone position,"
             " instead of the PlayerStart position"  # ~ (0, 0, 0) in AirSim's NED system
    )

    parser.add_argument(
        "--wait_sec",
        type=float,
        default=4.0,
        help="Time hovering in the final position  (default: %(default)ds)"
    )

    parser.add_argument(
        "--teleport",
        action="store_true",
        help="Teleport to specified position, instead of waiting for the drone to fly"
             " (NOTE this may lead to errors with .exe environments, see pull#2324)"
    )

    parser.add_argument(
        "--airsim_root",
        type=str,
        default=os.path.join("D:", "dev", "AirSim"),
        help="AirSim directory  (default: %(default)s)"
    )

    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory."
    )

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    return parser


def import_airsim(airsim_path: str, create_symbolic_link: bool = False) -> None:
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        assert os.path.exists(
            client_path
        ), f"\nexpected '{client_path}' doesn't exist\n"

        if create_symbolic_link:
            symlink_cmd = ["ln", "-s", airsim_path, "airsim"]
            if args.verbose:
                symlink_cmd.append("--verbose")
            subprocess.run(symlink_cmd)

            airsim_client_root = os.getcwd()
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # ModuleNotFoundError will be raised if the path is incorrect


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


def main(args: argparse.Namespace) -> None:
    try:
        airsim_path = airsim.__path__
    except NameError:
        airsim_path = os.path.join(args.airsim_root, "PythonClient", "airsim")
        import_airsim(airsim_path, create_symbolic_link=args.symbolic_link)
    finally:
        if args.verbose:
            path_str = f"'airsim' path: {airsim.__path__[0]}"
            print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

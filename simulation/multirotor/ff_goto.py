import os
import sys
import time
import argparse
import subprocess

import numpy as np

from typing import Tuple

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
) -> Tuple[float, float, float]:
    ''' `(north, east, down)` values in `m`, following the right-hand rule '''
    if multirotor_state is None:
        multirotor_state = client.getMultirotorState()
    pos = multirotor_state.kinematics_estimated.position  # Vector3r
    return pos.x_val, pos.y_val, pos.z_val


def get_UE4_coords(
    client: airsim.MultirotorClient, multirotor_state: airsim.MultirotorState = None
) -> Tuple[float, float, float]:
    ''' `(north, east, up)` values in `cm`, following the left-hand rule '''
    if multirotor_state is None:
        multirotor_state = client.getMultirotorState()
    pos = multirotor_state.gps_location  # GeoPoint
    return pos.latitude, pos.longitude, pos.altitude


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.z is not None and args.z > 0:
        print("Warning: AirSim uses NED coordinates, meaning +z is down")
        print("         (to fly upwards use negative z values)\n")

    state = client.getMultirotorState()
    start_pos = {
        'NED': get_NED_coords(client, state),
        'UE4': get_UE4_coords(client, state)
    }  # obs.: UE4's PlayerStart coordinates correspond to ~(0, 0, 0) in AirSim's NED system

    print(f"Starting position (NED): {start_pos['NED']}\n")
    if args.verbose:
        print(f"Starting position (UE4): {start_pos['UE4']}\n")

    if args.relative:
        if args.z is None:
            args.z = 0
        xyz = tuple(sum(axis) for axis in zip(start_pos['NED'], (args.x, args.y, args.z)))
    else:
        if args.z is None:
            args.z = start_pos['NED'][2]  # keep the same altitude
        xyz = (args.x, args.y, args.z)

    if state.landed_state == airsim.LandedState.Landed:
        print(f"landed_state = {state.landed_state} (Landed)")
        print("[ff] Taking off.. ", end="", flush=True)
        client.takeoffAsync().join()
        print("done.")
    else:
        # NOTE .exe environments seem to always return Landed
        print(f"landed_state = {state.landed_state} (Flying)")
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
    print(f"Ending position (NED): {get_NED_coords(client, state)}\n")
    if args.verbose:
        print(f"Ending position (UE4): {get_UE4_coords(client, state)}\n")

    print(f"[ff] Hovering for {args.wait_sec} seconds.. ", end="", flush=True)
    client.hoverAsync().join()
    time.sleep(args.wait_sec)
    print("done.")

    if args.verbose:
        state = client.getMultirotorState()
        print(f"Final position (NED): {get_NED_coords(client, state)}\n")
        print(f"Final position (UE4): {get_UE4_coords(client, state)}\n")


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

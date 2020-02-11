import os
import sys
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
    pass


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    def get_NED_coords(
        multirotor_state: airsim.MultirotorState = None,
    ) -> airsim.Vector3r:
        if multirotor_state is None:
            multirotor_state = client.getMultirotorState()
        # return multirotor_state.position
        return multirotor_state.kinematics_estimated.position

    def get_UE4_coords(
        multirotor_state: airsim.MultirotorState = None,
    ) -> airsim.Vector3r:
        if multirotor_state is None:
            multirotor_state = client.getMultirotorState()
        pos = multirotor_state.gps_location  # GeoPoint
        return airsim.Vector3r(pos.latitude, pos.longitude, pos.altitude)

    state = client.getMultirotorState()

    # NED starting point coordinates ~(0, 0, 0)
    home_position = get_NED_coords(state)

    # UE4 PlayerStart coordinates
    player_start = get_UE4_coords(state)

    print("home_position (NED):", home_position, "\nplayer_start (UE4):", player_start)

    print("\n[ff] Taking off.. ", end="")
    client.takeoffAsync().join()
    print("done.")

    state = client.getMultirotorState()
    print("NED:", get_NED_coords(state), "\nUE4:", get_UE4_coords(state))

    print("\n[ff] Landing.. ", end="")
    client.landAsync().join()
    print("done.")

    state = client.getMultirotorState()
    print("NED:", get_NED_coords(state), "\nUE4:", get_UE4_coords(state))

    print("\n[ff] Reset")
    client.reset()
    state = client.getMultirotorState()
    print("NED:", get_NED_coords(), "\nUE4:", get_UE4_coords())
    pass


###############################################################################
###############################################################################


# FIXME PlayerStart in NED is not exactly (0, 0, 0)


def convert_to_UE4(coords_NED):
    coords_UE4 = coords_NED * 100
    coords_UE4.z_val *= -1
    coords_UE4 += player_start
    return coords_UE4  # UE4 is in cm, with +z pointing up


def convert_to_NED(coords_UE4):
    coords_NED = (coords_UE4 - player_start) / 100
    coords_NED.z_val *= -1
    return coords_NED  # NED is in meters, with +z pointing down


###############################################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument(
        "z",
        type=float,
        nargs="?",
        default=None,
        help="Z coordinate (kept the same by default)",
    )

    parser.add_argument(
        "--airsim_root",
        type=str,
        default=os.path.join("D:", "dev", "AirSim"),
        help="AirSim directory  (default: %(default)s)",
    )

    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory.",
    )

    parser.add_argument(
        "--disable_api_on_exit",
        action="store_true",
        help="Disable API control on exit by calling client.enableApiControl(False).",
    )

    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
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
    finally:
        if args.disable_api_on_exit:
            client.enableApiControl(False)


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

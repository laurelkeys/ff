import os, sys, time, argparse

import ff
from ff_types import Vec3

try:
    import airsim
except ModuleNotFoundError:
    airsim_path = ff.Default.AIRSIM_CLIENT_PATH
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))
    import airsim


###############################################################################
# constants ###################################################################
###############################################################################


# test paths for different maps (in NED coordinates)
BLOCKS_PATH = [
    Vec3(15, 15, -40),
    Vec3(55, 15, -40),
    Vec3(55, -20, -40),
    Vec3(15, -20, -40),
]


###############################################################################
# preflight (called before connecting) ########################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    args.flight_velocity = 5
    args.flight_path = BLOCKS_PATH  # TODO add more options


###############################################################################
# fly (called after connecting) ###############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.verbose:
        print(f"[ff] HomeGeoPoint: {Vec3.from_GeoPoint(client.getHomeGeoPoint())}\n")
        print(
            f"[ff] VehiclePose: position={Vec3.from_Vector3r(client.simGetVehiclePose().position)}\n"
        )

    initial_state = client.getMultirotorState()

    if initial_state.landed_state == airsim.LandedState.Landed:
        print(f"[ff] Taking off")
        client.takeoffAsync(timeout_sec=8).join()

    print(f"[ff] Moving on path...", end=" ", flush=True)
    client.moveOnPathAsync(
        path=[airsim.Vector3r(coord.x, coord.y, coord.z) for coord in args.flight_path],
        velocity=args.flight_velocity,
    ).join()
    print(f"done.")

    time.sleep(4)
    print(f"[ff] Drone reset")
    client.reset()


###############################################################################
# airsim (general) ############################################################
###############################################################################


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
# main ########################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        path_str = f"'airsim' path: {airsim.__path__[0]}"
        print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    if args.env_name is not None:
        ff.launch_env(args)  # the --launch option was passed

    if not args.no_wait:
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset


###############################################################################
# argument parsing ############################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "--airsim_root",
        type=str,
        default=ff.Default.ARGS["airsim_root"],
        help="AirSim directory  (default: %(default)s)",
    )

    parser.add_argument(
        "--launch",
        dest="env_name",
        metavar="ENV_NAME",
        type=str,
        nargs="?",
        const="",  # tries to select a valid environment folder in env_root
        help="Name of the folder that contains the environment (.exe or .uproject file) to run.",
    )

    parser.add_argument(
        "--from",
        dest="env_root",
        metavar="ENV_ROOT",
        type=str,
        default=ff.Default.ARGS["env_root"],
        help="Directory that contains the environment folder  (default: %(default)s)."
        "\nAliases: {"
        + ", ".join(
            [f"'{alias}': {env_root}" for alias, env_root in ff.Default.ENV_ROOT_ALIASES.items()]
        )
        + "}.",
    )

    parser.add_argument(
        "--edit",
        action="store_true",
        help="Launch the specified environment's .sln in Visual Studio (instead of running its .uproject).",
    )

    parser.add_argument(
        "--no_wait",
        action="store_true",
        help="Don't wait for a key press before connecting to AirSim.",
    )

    parser.add_argument(
        "--devenv_exe",
        type=str,
        default=ff.Default.ARGS["devenv_exe"],
        help="Path to Visual Studio's devenv.exe, needed to launch .sln environments  (default: %(default)s)",
    )

    parser.add_argument(
        "--unreal_editor_exe",
        type=str,
        default=ff.Default.ARGS["unreal_editor_exe"],
        help="Path to UE4Editor.exe, needed to launch .uproject environments  (default: %(default)s)",
    )

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

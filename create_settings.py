import json
import argparse

from os.path import join


def env_settings(args) -> str:
    settings_path = join(args.envs_root, "settings.json")
    with open(settings_path) as settings_file:
        settings = settings_file.read()
    return settings


def get_parser():
    parser = argparse.ArgumentParser(
        description="Generate settings.json for AirSim environments."
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    parser.add_argument(
        "--envs_root",
        type=str,
        default="D:\\Documents\\AirSim",
        help="Downloaded environments directory  (default: %(default)s)",
    )
    # SimMode
    parser.add_argument(
        "--sim_mode",
        type=str,
        choices=["ComputerVision", "Multirotor", "Car"],
        default="Multirotor",
        help="Which simulation mode will be used  (default: %(default)s)",
    )
    # ViewMode
    parser.add_argument(
        "--view_mode",
        type=str,
        choices=[
            "FlyWithMe",       # Chase the vehicle from behind with 6 degrees of freedom
            "GroundObserver",  # Chase the vehicle from 6' above the ground but with full freedom in XY plane
            "Fpv",             # View the scene from the vehicle's vehicle front camera
            "Manual",          # Use the arrow keys and WASD to move the camera manually
            "SpringArmChase",  # Chase the vehicle with a camera that has some latency in movement
            "NoDisplay",       # Freeze rendering for main screen but keep subwindows, recording and APIs active
        ],
        default="FlyWithMe",
        help="Which camera to use and how it will follow the vehicle  (default: %(default)s)",
    )
    return parser


if __name__ == "__main__":
    args = get_parser().parse_args()

    settings = {}
    settings["SettingsVersion"] = 1.2
    settings["SimMode"] = args.sim_mode

    settings_json = json.dumps(settings, indent=2)
    if args.verbose:
        print("Current settings.json:")
        print(env_settings(args))
        print()
        print("Updated settings.json:")
    print(settings_json)

import json
import argparse

from os.path import join


CAMERA_NAME = [
    "front_center",
    "front_right",
    "front_left",
    "bottom_center",
    "back_center",
]

IMAGE_TYPE = [
    "Scene",
    "DepthPlanner",
    "DepthPerspective",
    "DepthVis",
    "DisparityNormalized",
    "Segmentation",
    "SurfaceNormals",
    "Infrared",
]


def env_settings(args) -> str:
    settings_path = join(args.settings_root, "settings.json")
    with open(settings_path) as settings_file:
        settings = settings_file.read()
    return settings


def build_settings(args, default_args):
    _args = args.__dict__
    _default_args = default_args.__dict__

    json_dict = {"SettingsVersion": 1.2}

    def set_property(airsim_property):
        import re  # convert CamelCase to snake_case with RegEx

        argument = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", airsim_property)
        argument = re.sub("([a-z0-9])([A-Z])", r"\1_\2", argument).lower()
        if _args[argument] != _default_args[argument]:
            json_dict[airsim_property] = _args[argument]

    set_property("SimMode")
    set_property("LocalHostIp")
    set_property("ClockSpeed")
    set_property("ViewMode")

    return json.dumps(json_dict)


def get_parser():
    parser = argparse.ArgumentParser(
        description="Generate settings.json for AirSim environments."
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    parser.add_argument(
        "--settings_root",
        type=str,
        default="D:\\Documents\\AirSim",
        help="Path to settings.json's parent folder  (default: %(default)s)",
    )
    parser.add_argument(
        "--envs_root",
        type=str,
        default="D:\\Documents\\AirSim",
        help="Downloaded environments directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--sim_mode",  # SimMode
        type=str,
        choices=["ComputerVision", "Multirotor"],
        default="Multirotor",
        help="Which simulation mode will be used  (default: %(default)s)",
    )
    parser.add_argument(
        "--local_host_ip",  # LocalHostIp
        type=str,
        default="127.0.0.1",
        help="Set this if you'll be connecting to remote machines  (default: %(default)s)",
    )
    parser.add_argument(
        "--clock_speed",  # ClockSpeed
        type=float,
        default=1.0,  # NOTE higher values may decrease simulation quality
        help="Speed of simulation clock with respect to wall clock  (default: %(default)s)",
    )
    parser.add_argument(
        "--view_mode",  # ViewMode
        type=str,
        choices=[
            "FlyWithMe",  # Chase the vehicle from behind with 6 degrees of freedom
            "GroundObserver",  # Chase the vehicle from 6' above the ground but with full freedom in XY plane
            "Fpv",  # View the scene from the vehicle's vehicle front camera
            "Manual",  # Use the arrow keys and WASD to move the camera manually
            "SpringArmChase",  # Chase the vehicle with a camera that has some latency in movement
            "NoDisplay",  # Freeze rendering for main screen but keep subwindows, recording and APIs active
        ],
        default="FlyWithMe",
        help="Which camera to use and how it will follow the vehicle  (default: %(default)s)",
    )
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    settings_json = build_settings(args, default_args=parser.parse_args([]))

    if args.verbose:
        print("Current settings.json:")
        print(env_settings(args))
        print()
        print("Updated settings.json:")
    print(settings_json)


# TODO
#   - enable Sun position changes (docs/apis.md#time-of-day-api)
#   - configure subwindows (docs/settings.md#subwindows)
#   - specify cameras used for recording (docs/settings.md#recording)
#   - configure CaptureSettings (docs/settings.md#capturesettings)
#   - allow RecordUIVisible and LogMessagesVisible to be set false

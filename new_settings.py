import json
import argparse

from os.path import join


CAMERA_NAME = [
    "front_center",
    "front_right",
    "front_left",
    "bottom_center",  # NOTE "fpv" if the SimMode is Car
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
    parser.add_argument(
        "--sim_mode",  # SimMode
        type=str,
        choices=["ComputerVision", "Multirotor", "Car"],
        default="Multirotor",
        help="Which simulation mode will be used  (default: %(default)s)",
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
    parser.add_argument(
        "--record_on_move",  # RecordOnMove,
        action="store_true",
        help="Do not register the frame if the vehicle hasn't moved",
    )
    parser.add_argument(
        "--record_interval",  # RecordInterval
        type=float,
        default=0.05,
        help="Minimum interval in seconds between capturing two images  (default: %(default)s)",
    )
    return parser


if __name__ == "__main__":
    args = get_parser().parse_args()

    settings = {}
    settings["SettingsVersion"] = 1.2
    settings["SimMode"] = args.sim_mode
    settings["ClockSpeed"] = args.clock_speed
    settings["ViewMode"] = args.view_mode
    # FIXME specify 'Cameras' before setting 'Recording'

    settings_json = json.dumps(settings, indent=2)
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

# Format example with default values:
# {
#     "SimMode": "",
#     "ClockSpeed": 1,
#     "RecordUIVisible": true, // ?
#     "LogMessagesVisible": true, // ?
#     "ViewMode": "",
#     "Recording": {
#         "RecordOnMove": false,
#         "RecordInterval": 0.05,
#         "Cameras": [
#             {
#                 "CameraName": "0",
#                 "ImageType": 0,
#                 "PixelsAsFloat": false,
#                 "Compress": true
#             }
#         ]
#     },
#     "CameraDefaults": {
#         "CaptureSettings": [
#             {
#                 "ImageType": 0,
#                 "Width": 256,
#                 "Height": 144,
#                 "FOV_Degrees": 90,
#                 "AutoExposureSpeed": 100,
#                 "AutoExposureBias": 0,
#                 "AutoExposureMaxBrightness": 0.64,
#                 "AutoExposureMinBrightness": 0.03,
#                 "MotionBlurAmount": 0,
#                 "TargetGamma": 1.0,
#                 "ProjectionMode": "",
#                 "OrthoWidth": 5.12
#             }
#         ],
#         "X": NaN,
#         "Y": NaN,
#         "Z": NaN,
#         "Pitch": NaN,
#         "Roll": NaN,
#         "Yaw": NaN
#     },
#     "OriginGeopoint": {
#         "Latitude": 47.641468,
#         "Longitude": -122.140165,
#         "Altitude": 122
#     },
#     "TimeOfDay": {
#         "Enabled": false,
#         "StartDateTime": "",
#         "CelestialClockSpeed": 1,
#         "StartDateTimeDst": false,
#         "UpdateIntervalSecs": 60
#     },
#     "SubWindows": [
#         {
#             "WindowID": 0,
#             "CameraName": "0",
#             "ImageType": 3,
#             "Visible": false
#         },
#         {
#             "WindowID": 1,
#             "CameraName": "0",
#             "ImageType": 5,
#             "Visible": false
#         },
#         {
#             "WindowID": 2,
#             "CameraName": "0",
#             "ImageType": 0,
#             "Visible": false
#         }
#     ]
# }
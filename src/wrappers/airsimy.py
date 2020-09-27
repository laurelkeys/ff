import numpy as np
from __future__ import annotations

from typing import Dict, List

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim

###############################################################################
###############################################################################


class AirSimImage:
    @staticmethod
    def array_from_uncompressed(image, height, width):
        return np.flipud(np.fromstring(image, dtype=np.uint8).reshape(height, width, -1))

    @staticmethod
    def get_mono(client, camera_name=ff.CameraName.front_center, vehicle_name=None):
        request = {
            "requests": [
                airsim.ImageRequest(
                    camera_name, airsim.ImageType.Scene, pixels_as_float=False, compress=False,
                )
            ]
        }

        if vehicle_name is not None:
            request["vehicle_name"] = vehicle_name

        response, *_ = client.simGetImages(**request)

        return AirSimImage.array_from_uncompressed(
            response.image_data_uint8, response.height, response.width
        )

    @staticmethod
    def get_stereo(client, vehicle_name=None):
        request = {
            "requests": [
                airsim.ImageRequest(
                    ff.CameraName.front_left,
                    airsim.ImageType.Scene,
                    pixels_as_float=False,
                    compress=False,
                ),
                airsim.ImageRequest(
                    ff.CameraName.front_right,
                    airsim.ImageType.Scene,
                    pixels_as_float=False,
                    compress=False,
                ),
            ]
        }

        if vehicle_name is not None:
            request["vehicle_name"] = vehicle_name

        response_left, response_right, *_ = client.simGetImages(**request)

        return (
            AirSimImage.array_from_uncompressed(
                response_left.image_data_uint8, response_left.height, response_left.width
            ),
            AirSimImage.array_from_uncompressed(
                response_right.image_data_uint8, response_right.height, response_right.width
            ),
        )


###############################################################################
###############################################################################


class AirSimRotation:
    def __init__(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    @staticmethod
    def nanRotation() -> AirSimRotation:
        return AirSimRotation(np.nan, np.nan, np.nan)


###############################################################################
###############################################################################


class AirSimSettings:
    def __init__(
        self,
        # TODO add more settings
        sim_mode: str = ff.SimMode.Multirotor,
        view_mode: str = ff.ViewMode.Default,
        subwindows: List[AirSimSettings.Subwindow] = None,  # FIXME handle
    ):
        self.settings_version = 1.2  # TODO keep this synced with AirSim
        self.sim_mode = sim_mode
        self.view_mode = view_mode

        # return {
        #     "SettingsVersion": 1.2,
        #     "SimMode": sim_mode,
        #     "ViewMode": view_mode,
        #     # TODO test specifying the drone's initial position, then
        #     #      make this configurable through an argument
        #     # "Vehicles": {
        #     #     "SimpleFlight": {
        #     #         "VehicleType": "SimpleFlight",
        #     #         "X": 0.0, "Y": 0.0, "Z": -10.0,
        #     #     }
        #     # }
        # }

    class Subwindow:
        def __init__(
            self,
            window_id: int = 0,
            image_type: int = airsim.ImageType.Scene,
            visible: bool = False,
            camera_name: str = None,  # ff.CameraName
            vehicle_name: str = None,
        ):
            assert 0 <= window_id <= 2, window_id
            self.window_id = window_id
            self.image_type = image_type
            self.visible = visible
            self.camera_name = camera_name or ""
            self.vehicle_name = vehicle_name or ""

    class CaptureSettings:
        def __init__(
            self,
            width: int = 960,
            height: int = 540,
            fov_degrees: int = 90,
            image_type: int = airsim.ImageType.Scene,
        ):
            self.width = width
            self.height = height
            self.fov_degrees = fov_degrees
            self.image_type = image_type

    # class Gimbal:
    #     def __init__(
    #         self,
    #         stabilization: float = 0.0,
    #         rotation: AirSimRotation = None,
    #     ):
    #         self.stabilization = stabilization
    #         self.rotation = rotation

    class Camera:
        def __init__(
            self,
            position: airsim.Vector3r = None,
            # rotation: AirSimRotation = None,
            # gimbal: Gimbal = None,
            capture_settings: List[AirSimSettings.CaptureSettings] = None,
            image_type: int = airsim.ImageType.Scene,
        ):
            self.position = position
            self.capture_settings = capture_settings or []
            self.image_type = image_type

    class Vehicle:
        class VehicleType:
            PhysXCar = "PhysXCar"
            SimpleFlight = "SimpleFlight"
            ComputerVision = "ComputerVision"

        class DefaultVehicleState:
            Armed = "Armed"
            Disarmed = "Disarmed"

        def __init__(
            self,
            name: str,
            vehicle_type: str = VehicleType.SimpleFlight,
            default_vehicle_state: str = DefaultVehicleState.Armed,
            position: airsim.Vector3r = None,
            rotation: AirSimRotation = None,
            cameras: Dict[str, AirSimSettings.Camera] = None,
        ):
            assert all([_ in ff.CameraName._list_all for _ in cameras.keys()]), cameras
            self.name = name
            self.vehicle_type = vehicle_type
            self.default_vehicle_state = default_vehicle_state
            self.position = position
            self.rotation = rotation
            self.cameras = cameras or []


###############################################################################
###############################################################################

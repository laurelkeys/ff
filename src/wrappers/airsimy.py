from __future__ import annotations

from typing import Any, Dict, List

import ff
import numpy as np

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
        sim_mode: str = ff.SimMode.Multirotor,
        view_mode: str = ff.ViewMode.Default,
        ## origin_geopoint: airsim.GeoPoint = None,
        subwindows: List[AirSimSettings.Subwindow] = None,  # FIXME handle
        vehicles: List[AirSimSettings.Vehicle] = None,
        camera_defaults: AirSimSettings.Camera = None,  # FIXME handle
    ):
        self.settings_version = 1.2  # NOTE keep this synced with AirSim
        self.sim_mode = sim_mode
        self.view_mode = view_mode
        self.subwindows = subwindows
        self.vehicles = vehicles
        self.camera_defaults = camera_defaults

    def as_dict(self) -> dict:
        settings: Dict[str, Any] = {"SettingsVersion": self.settings_version}

        if self.sim_mode != ff.SimMode.Default:
            settings["SimMode"] = self.sim_mode

        if self.view_mode != ff.ViewMode.Default:
            settings["ViewMode"] = self.view_mode

        if self.vehicles is not None:
            settings["Vehicles"] = {_.name: _.as_dict() for _ in self.vehicles}

        return settings

    class Subwindow:
        def __init__(
            self,
            window_id: int = 0,
            image_type: int = airsim.ImageType.Scene,
            visible: bool = False,
            camera_name: str = None,
            vehicle_name: str = None,
        ):
            assert 0 <= window_id <= 2, window_id
            self.window_id = window_id
            self.image_type = image_type
            self.visible = visible
            self.camera_name = camera_name
            self.vehicle_name = vehicle_name

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

        def as_dict(self) -> dict:
            capture_settings: Dict[str, Any] = {"ImageType": self.image_type}

            if self.width is not None:
                capture_settings["Width"] = self.width
            if self.height is not None:
                capture_settings["Height"] = self.height
            if self.fov_degrees is not None:
                capture_settings["FOV_Degrees"] = self.fov_degrees

            return capture_settings

    ## class Gimbal:
    ##     def __init__(
    ##         self,
    ##         stabilization: float = 0.0,
    ##         rotation: AirSimRotation = None,
    ##     ):
    ##         self.stabilization = stabilization
    ##         self.rotation = rotation

    class Camera:
        def __init__(
            self,
            position: airsim.Vector3r = None,
            ## rotation: AirSimRotation = None,
            ## gimbal: Gimbal = None,
            capture_settings: List[AirSimSettings.CaptureSettings] = None,
        ):
            self.position = position
            self.capture_settings = capture_settings

        def as_dict(self) -> dict:
            camera = {}

            if self.position is not None:
                camera["X"] = self.position.x_val
                camera["Y"] = self.position.y_val
                camera["Z"] = self.position.z_val

            if self.capture_settings is not None:
                camera["CaptureSettings"] = [_.as_dict() for _ in self.capture_settings]

            return camera

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
            default_vehicle_state: str = None,
            position: airsim.Vector3r = None,  # NOTE in global NED
            rotation: AirSimRotation = None,
            cameras: Dict[str, AirSimSettings.Camera] = None,
        ):
            if cameras is not None:
                assert all([_ in ff.CameraName._list_all for _ in cameras.keys()]), cameras
            self.name = name
            self.vehicle_type = vehicle_type
            self.default_vehicle_state = default_vehicle_state
            self.position = position
            self.rotation = rotation
            self.cameras = cameras

        def as_dict(self) -> dict:
            # NOTE there's no default VehicleType, so it must be specified
            vehicle: Dict[str, Any] = {"VehicleType": self.vehicle_type}

            if self.default_vehicle_state is not None:
                vehicle["DefaultVehicleState"] = self.default_vehicle_state

            if self.position is not None:
                vehicle["X"] = self.position.x_val
                vehicle["Y"] = self.position.y_val
                vehicle["Z"] = self.position.z_val

            if self.rotation is not None:
                vehicle["Yaw"] = self.rotation.yaw
                vehicle["Pitch"] = self.rotation.pitch
                vehicle["Roll"] = self.rotation.roll

            if self.cameras is not None:
                vehicle["Cameras"] = {name: _.as_dict() for name, _ in self.cameras.items()}

            return vehicle


###############################################################################
###############################################################################

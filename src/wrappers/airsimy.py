from __future__ import annotations

from typing import Dict, List, Union

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


class Rotation:
    def __init__(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0):
        """ Represents a 3D rotation along the normal / vertical axis (`yaw`), transverse
            / lateral axis (`pitch`) and longitudinal axis (`roll`).

            Note: some of AirSim's API calls expect values in the order `pitch, roll, yaw`.
        """
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    @staticmethod
    def nanRotation() -> Rotation:
        return Rotation(np.nan, np.nan, np.nan)

    def as_dict(self) -> dict:
        return {"Yaw": self.yaw, "Pitch": self.pitch, "Roll": self.roll}

    def as_quaternion(self) -> airsim.Quaternionr:
        """ Returns a quaternion representing the rotation angles.

            Note: AirSim's `Quaternionr` expresses coordinates in WXYZ order.
        """
        return airsim.to_quaternion(
            pitch=self.pitch,
            roll=self.roll,
            yaw=self.yaw,
        )


###############################################################################
###############################################################################


class Rgba(tuple):
    White   = (1.0, 1.0, 1.0, 1.0)
    Black   = (0.0, 0.0, 0.0, 1.0)
    Red     = (1.0, 0.0, 0.0, 1.0)
    Green   = (0.0, 1.0, 0.0, 1.0)
    Blue    = (0.0, 0.0, 1.0, 1.0)
    Cyan    = (0.0, 1.0, 1.0, 1.0)
    Magenta = (1.0, 0.0, 1.0, 1.0)
    Yellow  = (1.0, 1.0, 0.0, 1.0)

    def __new__(cls, r: float, g: float, b: float, alpha: float = 1.0):
        rgba = (r, g, b, alpha)
        assert all(0.0 <= ch <= 1.0 for ch in rgba), rgba
        return super(Rgba, cls).__new__(cls, rgba)

    def __init__(self, r: float, g: float, b: float, alpha: float = 1.0):
        """ Construct a new color, with values RGBA in `[0.0, 1.0]`. """
        self.r = self[0]
        self.g = self[1]
        self.b = self[2]
        self.a = self[3]

    def from255(self, r: int, g: int, b: int, alpha: float = 1.0) -> Rgba:
        """ Construct a new color by converting RGB values from `[0, 255]` to `[0.0, 1.0]`. """
        return Rgba(r / 255, g / 255, b / 255, alpha)


###############################################################################
###############################################################################


class AirSimSettings:
    """ Creates a `dict` representation of AirSim's `settings.json` file. """

    DictType = Dict[str, Union[str, int, float, dict, list]]

    def __init__(
        self,
        sim_mode: str = ff.SimMode.Multirotor,
        view_mode: str = ff.ViewMode.Default,
        clock_speed: float = 1.0,
        camera_defaults: AirSimSettings.Camera = None,
        origin_geopoint: airsim.GeoPoint = None,
        subwindows: List[AirSimSettings.Subwindow] = None,
        vehicles: List[AirSimSettings.Vehicle] = None,
    ):
        self.settings_version = 1.2  # NOTE keep this synced with AirSim
        self.sim_mode = sim_mode
        self.view_mode = view_mode
        self.clock_speed = clock_speed
        self.camera_defaults = camera_defaults
        self.origin_geopoint = origin_geopoint
        self.subwindows = subwindows
        self.vehicles = vehicles

    def as_dict(self) -> AirSimSettings.DictType:
        settings: AirSimSettings.DictType = {"SettingsVersion": self.settings_version}

        if self.sim_mode != ff.SimMode.Default:
            settings["SimMode"] = self.sim_mode

        if self.view_mode != ff.ViewMode.Default:
            settings["ViewMode"] = self.view_mode

        if self.clock_speed != 1.0:
            settings["ClockSpeed"] = self.clock_speed

        if self.camera_defaults is not None:
            settings["CameraDefaults"] = self.camera_defaults.as_dict()

        if self.origin_geopoint is not None:
            settings["OriginGeopoint"] = {
                "Latitude": self.origin_geopoint.latitude,
                "Longitude": self.origin_geopoint.longitude,
                "Altitude": self.origin_geopoint.altitude,
            }

        if self.subwindows is not None:
            settings["SubWindows"] = [_.as_dict() for _ in self.subwindows]

        if self.vehicles is not None:
            settings["Vehicles"] = {_.name: _.as_dict() for _ in self.vehicles}

        return settings

    class CaptureSettings:
        def __init__(
            self,
            width: int = 960,
            height: int = 540,
            image_type: int = airsim.ImageType.Scene,
            fov_degrees: int = 90,
        ):
            self.width = width
            self.height = height
            self.image_type = image_type
            self.fov_degrees = fov_degrees

        def as_dict(self) -> AirSimSettings.DictType:
            capture_settings: AirSimSettings.DictType = {"ImageType": self.image_type}
            if self.width is not None: capture_settings["Width"] = self.width
            if self.height is not None: capture_settings["Height"] = self.height
            if self.fov_degrees is not None: capture_settings["FOV_Degrees"] = self.fov_degrees
            return capture_settings

    class Gimbal:
        def __init__(
            self, stabilization: float = 0.0, rotation: Rotation = None,
        ):
            self.stabilization = stabilization
            self.rotation = rotation

        def as_dict(self) -> AirSimSettings.DictType:
            gimbal: AirSimSettings.DictType = {"Stabilization": self.stabilization}
            if self.rotation is not None: gimbal.update(self.rotation.as_dict())
            return gimbal

    class Camera:
        def __init__(
            self,
            capture_settings: List[AirSimSettings.CaptureSettings] = None,
            position: airsim.Vector3r = None,
            rotation: Rotation = None,
            gimbal: AirSimSettings.Gimbal = None,
        ):
            self.capture_settings = capture_settings
            self.position = position
            self.rotation = rotation
            self.gimbal = gimbal

        def as_dict(self) -> AirSimSettings.DictType:
            camera: AirSimSettings.DictType = {}
            if self.capture_settings is not None: camera["CaptureSettings"] = [_.as_dict() for _ in self.capture_settings]
            if self.position is not None: camera.update({"X": self.position.x_val, "Y": self.position.y_val, "Z": self.position.z_val})
            if self.rotation is not None: camera.update(self.rotation.as_dict())
            if self.gimbal is not None: camera["Gimbal"] = self.gimbal.as_dict()
            return camera

    class Subwindow:
        def __init__(
            self,
            window_id: int = 0,
            visible: bool = False,
            image_type: int = airsim.ImageType.Scene,
            camera_name: str = None,
            vehicle_name: str = None,
        ):
            assert 0 <= window_id <= 2, window_id
            self.window_id = window_id
            self.visible = visible
            self.image_type = image_type
            self.camera_name = camera_name
            self.vehicle_name = vehicle_name

        def as_dict(self) -> AirSimSettings.DictType:
            subwindow: AirSimSettings.DictType = {"WindowID": self.window_id}
            if self.visible: subwindow["Visible"] = self.visible
            if self.image_type is not None: subwindow["ImageType"] = self.image_type
            if self.camera_name is not None: subwindow["CameraName"] = self.camera_name
            if self.vehicle_name is not None: subwindow["VehicleName"] = self.vehicle_name
            return subwindow

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
            position: airsim.Vector3r = None,  # NOTE in global NED coordinates, SI units and origin at PlayerStart
            rotation: Rotation = None,  # NOTE in degrees
            cameras: Dict[str, AirSimSettings.Camera] = None,
        ):
            assert cameras is None or all([_ in ff.CameraName._list_all for _ in cameras.keys()]), cameras
            self.name = name
            self.vehicle_type = vehicle_type
            self.default_vehicle_state = default_vehicle_state
            self.position = position
            self.rotation = rotation
            self.cameras = cameras

        def as_dict(self) -> AirSimSettings.DictType:
            vehicle: AirSimSettings.DictType = {"VehicleType": self.vehicle_type}
            if self.default_vehicle_state is not None: vehicle["DefaultVehicleState"] = self.default_vehicle_state
            if self.position is not None: vehicle.update({"X": self.position.x_val, "Y": self.position.y_val, "Z": self.position.z_val})
            if self.rotation is not None: vehicle.update(self.rotation.as_dict())
            if self.cameras is not None: vehicle["Cameras"] = {name: _.as_dict() for name, _ in self.cameras.items()}
            return vehicle


###############################################################################
###############################################################################

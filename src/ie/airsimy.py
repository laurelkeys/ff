from __future__ import annotations

from enum import Enum
from typing import Dict, List, Union, cast
from contextlib import contextmanager

import ff
import numpy as np
import airsim

from airsim.types import Vector3r, Quaternionr

###############################################################################
###############################################################################


def connect(sim_mode: str) -> airsim.MultirotorClient:
    """ Returns a (Multirotor or ComputerVision) client connected to AirSim. """
    assert sim_mode in [ff.SimMode.Multirotor, ff.SimMode.ComputerVision], sim_mode

    if sim_mode == ff.SimMode.Multirotor:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
    else:
        client = airsim.VehicleClient()
        client.confirmConnection()

    return cast(airsim.MultirotorClient, client)


def reset(client: airsim.MultirotorClient) -> None:
    """ Resets a client connected to AirSim and re-enables API control. """
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(True)


###############################################################################
###############################################################################


@contextmanager
def pose_at_simulation_pause(client: airsim.MultirotorClient):
    client.simPause(is_paused=True)
    try:
        yield client.simGetVehiclePose()
    finally:
        client.simPause(is_paused=False)


###############################################################################
###############################################################################


def compute_camera_intrinsics(image_width, image_height, fov, is_degrees=False):
    if is_degrees:
        fov = np.deg2rad(fov)

    f = image_width / (2 * np.tan(fov))
    cu = image_width / 2
    cv = image_height / 2

    return np.array([[f, 0, cu], [0, f, cv], [0, 0, 1]])


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

    @staticmethod
    def get_mono_and_pose(client, camera_name=ff.CameraName.front_center, vehicle_name=None):
        client.simPause(True)
        image = AirSimImage.get_mono(client, camera_name, vehicle_name)
        pose = client.simGetVehiclePose() if vehicle_name is None else client.simGetVehiclePose(vehicle_name)
        client.simPause(False)
        return image, pose

    @staticmethod
    def get_stereo_and_pose(client, vehicle_name=None):
        client.simPause(True)
        image = AirSimImage.get_stereo(client, vehicle_name)
        pose = client.simGetVehiclePose() if vehicle_name is None else client.simGetVehiclePose(vehicle_name)
        client.simPause(False)
        return image, pose


###############################################################################
###############################################################################


# NOTE these values can be used with YawMode(is_rate=False) to change orientation
YAW_N =   0  # North
YAW_E =  90  # East
YAW_W = -90  # West
YAW_S = 180  # South


# NOTE see AirLib/include/common/VectorMath.hpp
FRONT = Vector3r( 1,  0,  0)  # North
BACK  = Vector3r(-1,  0,  0)  # South
DOWN  = Vector3r( 0,  0,  1)
UP    = Vector3r( 0,  0, -1)
RIGHT = Vector3r( 0,  1,  0)  # East
LEFT  = Vector3r( 0, -1,  0)  # West


###############################################################################
###############################################################################


class AirSimNedTransform:
    """ References:
        https://github.com/microsoft/AirSim/blob/master/docs/apis.md#coordinate-system
        https://github.com/microsoft/AirSim/blob/master/Unreal/Plugins/AirSim/Source/NedTransform.h
    """

    class CoordinateSystem(Enum):
        """ Vehicles are spawned at position specified in settings in global NED. """
        Unreal    = 1  # UU or Unreal Units or Unreal Coordinate system
        GlobalNed = 2  # NED transformation of UU with origin set to 0,0,0
        LocalNed  = 3  # NED transformation of UU with origin set to vehicle's spawning UU location
        Geo       = 4  # GPS coordinate where UU origin is assigned some geo-coordinate

    # NOTE UU  is left  handed, with units in centimeters, and +X = Front, +Y = Right, +Z = Up
    # NOTE NED is right handed, with units in      meters, and +X = North, +Y = East,  +Z = Down

    UU_TO_NED_SCALE = 0.01
    NED_TO_UU_SCALE = 100

    @staticmethod
    def vector_from_uu_to_ned(v: Vector3r, scale: float = UU_TO_NED_SCALE):
        return Vector3r(v.x_val * scale, v.y_val * scale, -v.z_val * scale)

    @staticmethod
    def vector_from_ned_to_uu(v: Vector3r, scale: float = NED_TO_UU_SCALE):
        return Vector3r(v.x_val * scale, v.y_val * scale, -v.z_val * scale)

    @staticmethod
    def quaternion_from_uu_to_ned(q: Quaternionr):
        return Quaternionr(-q.x_val, -q.y_val, q.z_val, w_val=q.w_val)

    @staticmethod
    def quaternion_from_ned_to_uu(q: Quaternionr):
        return Quaternionr(-q.x_val, -q.y_val, q.z_val, w_val=q.w_val)


###############################################################################
###############################################################################


def matrix_from_eularian_angles(pitch: float, roll: float, yaw: float, is_degrees: bool = False) -> np.ndarray:
    # ref.: https://github.com/microsoft/AirSim/blob/master/PythonClient/computer_vision/capture_ir_segmentation.py
    if is_degrees:
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)
        roll = np.deg2rad(roll)

    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    sr, cr = np.sin(roll), np.cos(roll)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    # NOTE roll is applied first, then pitch, then yaw.
    RyRx = np.matmul(Ry, Rx)
    return np.matmul(Rz, RyRx)


def matrix_from_rotation_axis_angle(axis: Vector3r, angle: float, is_degrees: bool = False) -> np.ndarray:
    # ref.: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    if is_degrees:
        angle = np.deg2rad(angle)

    x, y, z = axis.to_numpy_array()

    ca = np.cos(angle)
    sa = np.sin(angle)
    omca = 1 - ca

    return np.array(
        [
            [ca + (x ** 2) * omca, x * y * omca - z * sa, x * z * omca + y * sa],
            [y * x * omca + z * sa, ca + (y ** 2) * omca, y * z * omca - x * sa],
            [z * x * omca - y * sa, z * y * omca + x * sa, ca + (z ** 2) * omca],
        ]
    )


def quaternion_from_rotation_axis_angle(axis: Vector3r, angle: float, is_degrees: bool = False) -> Quaternionr:
    if is_degrees:
        angle = np.deg2rad(angle)

    half_angle = angle / 2
    axis /= axis.get_length()  # normalize
    axis *= np.sin(half_angle)
    return Quaternionr(axis.x_val, axis.y_val, axis.z_val, w_val=np.cos(half_angle))


def quaternion_look_at(source_point: Vector3r, target_point: Vector3r) -> Quaternionr:
    """ Assuming you are looking at `FRONT` vector, what rotation you need to look at `target_point`?. """
    # ref.: https://github.com/microsoft/AirSim/blob/master/AirLibUnitTests/QuaternionTest.hpp (lookAt)
    to_vector = target_point - source_point
    to_vector /= to_vector.get_length()  # normalize

    axis = FRONT.cross(to_vector)
    axis = UP if axis.get_length() == 0 else axis / axis.get_length()  # normalize

    return quaternion_from_rotation_axis_angle(axis, angle=np.arccos(FRONT.dot(to_vector)))

    # def quaternion_from_two_vectors(a: Vector3r, b: Vector3r) -> Quaternionr:
    #     """ What rotation (around the intersection of the two vectors) we need to rotate `a` to `b`? """
    #     # ref.: https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h (FromTwoVectors)
    #     v0 = a / a.get_length()
    #     v1 = b / b.get_length()
    #     c = v1.dot(v0)
    #
    #     assert c > -1  # FIXME handle the case when the vectors are nearly opposites
    #
    #     s = np.sqrt((1 + c) * 2)
    #     axis = v0.cross(v1) / s
    #     return Quaternionr(axis.x_val, axis.y_val, axis.z_val, w_val=(s / 2))
    #
    # return quaternion_from_two_vectors(FRONT, to_vector)


def quaternion_from_yaw(yaw: float, is_degrees: bool = False) -> Quaternionr:
    # ref.: https://github.com/microsoft/AirSim/blob/master/AirLib/include/common/VectorMath.hpp (quaternionFromYaw)
    return quaternion_from_rotation_axis_angle(DOWN, yaw, is_degrees)


def yaw_from_quaternion(q: Quaternionr) -> float:
    """ Extracts the yaw part from `q`, using RPY / euler (z-y'-x'') angles. """
    # ref.: https://github.com/microsoft/AirSim/blob/master/AirLib/include/common/VectorMath.hpp (yawFromQuaternion)
    return np.arctan2(
        2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
        1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val),
    )


# ref.: https://stackoverflow.com/questions/32438252/efficient-way-to-apply-mirror-effect-on-quaternion-rotation/40334755#40334755
def quaternion_flip_z_axis(q: Quaternionr) -> Quaternionr: return Quaternionr(-q.x_val, -q.y_val, q.z_val, w_val=q.w_val)
def quaternion_flip_y_axis(q: Quaternionr) -> Quaternionr: return Quaternionr(-q.x_val, q.y_val, -q.z_val, w_val=q.w_val)
def quaternion_flip_x_axis(q: Quaternionr) -> Quaternionr: return Quaternionr(q.x_val, -q.y_val, -q.z_val, w_val=q.w_val)


def rotate_vector_by_quaternion(v: Vector3r, q: Quaternionr) -> Vector3r:
    if q.get_length() != 1.0:
        q /= q.get_length()  # normalize

    # Extract the vector and scalar parts of q:
    u, s = Vector3r(q.x_val, q.y_val, q.z_val), q.w_val

    # NOTE the results from these two methods are the same up to 7 decimal places.

    # ref.: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    # v_prime = u * (2 * u.dot(v)) + v * (s * s - u.dot(u)) + u.cross(v) * (2 * s)

    # ref.: https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h (_transformVector)
    uv = u.cross(v)
    uv += uv
    v_prime = v + uv * s + u.cross(uv)

    return v_prime


if False:
    def matrix_from_rotator(pitch: float, yaw: float, roll: float, is_degrees: bool = False) -> np.ndarray:
        """ Reference: https://docs.unrealengine.com/en-US/API/Runtime/Core/Math/FRotator/index.html

            `pitch`: Rotation around the right axis (around Y axis), Looking up and down (0=Straight Ahead, +Up, -Down).
            `yaw`:   Rotation around the up axis (around Z axis), Running in circles 0=East, +North, -South.
            `roll`:  Rotation around the forward axis (around X axis), Tilting your head, 0=Straight, +Clockwise, -CCW.
        """
        if is_degrees:
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)
            roll = np.deg2rad(roll)

        sp, cp = np.sin(pitch), np.cos(pitch)
        sy, cy = np.sin(yaw), np.cos(yaw)
        sr, cr = np.sin(roll), np.cos(roll)

        x_axis = [(cp * cy), (cp * sy), sp]
        y_axis = [(sr * sp * cy - cr * sy), (sr * sp * sy + cr * cy), -(sr * cp)]
        z_axis = [-(cr * sp * cy + sr * sy), (cy * sr - cr * sp * sy), (cr * cp)]

        rotation_matrix = np.hstack((np.vstack((x_axis, y_axis, z_axis, [0, 0, 0])), [0, 0, 0, 1]))

        return rotation_matrix


    def quaternion_from_rotator(pitch: float, yaw: float, roll: float, is_degrees: bool = False) -> Quaternionr:
        """ Reference: https://docs.unrealengine.com/en-US/API/Runtime/Core/Math/FRotator/index.html

            `pitch`: Rotation around the right axis (around Y axis), Looking up and down (0=Straight Ahead, +Up, -Down).
            `yaw`:   Rotation around the up axis (around Z axis), Running in circles 0=East, +North, -South.
            `roll`:  Rotation around the forward axis (around X axis), Tilting your head, 0=Straight, +Clockwise, -CCW.
        """
        if is_degrees:
            pitch = np.deg2rad(np.fmod(pitch, 360))
            yaw = np.deg2rad(np.fmod(yaw, 360))
            roll = np.deg2rad(np.fmod(roll, 360))

        pitch *= 0.5
        yaw *= 0.5
        roll *= 0.5

        sp, cp = np.sin(pitch), np.cos(pitch)
        sy, cy = np.sin(yaw), np.cos(yaw)
        sr, cr = np.sin(roll), np.cos(roll)

        x = +(cr * sp * sy) - (sr * cp * cy)
        y = -(cr * sp * cy) - (sr * cp * sy)
        z = +(cr * cp * sy) - (sr * sp * cy)
        w = +(cr * cp * cy) + (sr * sp * sy)

        # NOTE calling airsim.to_quaternion(pitch, roll, yaw) is equivalent to
        # passing this return to AirSimNedTransform.quaternion_from_uu_to_ned.
        return Quaternionr(x, y, z, w_val=w)


###############################################################################
###############################################################################


class AirSimRecord:
    def __init__(
        self,
        time_stamp: int,
        pos_x: float, pos_y: float, pos_z: float,
        q_w: float, q_x: float, q_y: float, q_z: float,
        image_file: str,
    ):
        """ Represents an item from AirSim's recording file (i.e. a row in `airsim_rec.txt`). """
        # assert os.path.isfile(image_file), image_file
        self.time_stamp = time_stamp
        self.image_file = image_file
        self.position = Vector3r(pos_x, pos_y, pos_z)
        self.orientation = Quaternionr(q_x, q_y, q_z, q_w)

    @staticmethod
    def _parse(
        time_stamp: str,
        pos_x: str, pos_y: str, pos_z: str,
        q_w: str, q_x: str, q_y: str, q_z: str,
        image_file: str
    ) -> AirSimRecord:
        return AirSimRecord(
            int(time_stamp),
            float(pos_x), float(pos_y), float(pos_z),
            float(q_w), float(q_x), float(q_y), float(q_z),
            image_file
        )

    @staticmethod
    def list_from(rec_file: str) -> List[AirSimRecord]:
        """ Parses `airsim_rec.txt` into a list of records. """
        with open(rec_file, "r") as f:
            next(f)  # skip the column header "TimeStamp POS_X POS_Y POS_Z Q_W Q_X Q_Y Q_Z ImageFile"
            record_list = []
            for record_row in f:
                record_list.append(AirSimRecord._parse(*record_row.rstrip('\n').split('\t')))
        return record_list

    @staticmethod
    def dict_from(rec_file: str) -> Dict[str, AirSimRecord]:
        """ Parses `airsim_rec.txt` into a dictionary mapping image file to records. """
        with open(rec_file, "r") as f:
            next(f)  # skip the column header "TimeStamp POS_X POS_Y POS_Z Q_W Q_X Q_Y Q_Z ImageFile"
            record_dict = {}
            for record_row in f:
                record = AirSimRecord._parse(*record_row.rstrip('\n').split('\t'))
                record_dict[record.image_file] = record
        return record_dict


###############################################################################
###############################################################################


class AirSimRotation:
    def __init__(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0):
        """ Represents a 3D rotation along the normal / vertical axis (`yaw`), transverse
            / lateral axis (`pitch`) and longitudinal axis (`roll`).

            Note: some of AirSim's API calls expect values in the order `pitch, roll, yaw`.
        """
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    @staticmethod
    def nanRotation() -> AirSimRotation:
        return AirSimRotation(np.nan, np.nan, np.nan)

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


class AirSimSettings:
    """ Creates a `dict` representation of AirSim's `settings.json` file. """

    DictType = Dict[str, Union[str, int, float, dict, list]]

    def __init__(
        self,
        sim_mode: str = ff.SimMode.Multirotor,
        view_mode: str = ff.ViewMode.Default,
        clock_speed: float = 1.0,
        # TODO include Recording to adjust the RecordInterval
        camera_defaults: AirSimSettings.Camera = None,
        origin_geopoint: airsim.GeoPoint = None,
        subwindows: List[AirSimSettings.Subwindow] = None,
        vehicles: List[AirSimSettings.Vehicle] = None,
    ):
        if subwindows is not None:
            assert len((ids := [_.window_id for _ in subwindows])) == len(set(ids)), "duplicate window ids"
        if vehicles is not None:
            assert len((names := [_.name for _ in vehicles])) == len(set(names)), "duplicate vehicle names"

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

    class Recording:
        def __init__(
            self,
            record_on_move: bool = False,
            record_interval: float = 0.05,
            # FIXME add a cameras parameter (see blob/master/docs/settings.md#recording)
            # TODO add new options once they are merged (https://github.com/microsoft/AirSim/pull/2861)
        ):
            self.record_on_move = record_on_move
            self.record_interval = record_interval

        def as_dict(self) -> AirSimSettings.DictType:
            recording: AirSimSettings.DictType = {
                "RecordOnMove": self.record_on_move,
                "RecordInterval": self.record_interval,
            }
            return recording

    class CaptureSettings:
        def __init__(
            self,
            width: int = 960,
            height: int = 540,
            image_type: int = airsim.ImageType.Scene,
            fov_degrees: int = 90,  # NOTE horizontal (obs.: vertical fov = height / width * horizontal fov)
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
            self, stabilization: float = 0.0, rotation: AirSimRotation = None,
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
            rotation: AirSimRotation = None,
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
            rotation: AirSimRotation = None,  # NOTE in degrees
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

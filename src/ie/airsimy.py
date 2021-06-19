from __future__ import annotations

from enum import Enum
from typing import Dict, List, Tuple, Union, Optional, cast
from contextlib import contextmanager

import ff
import numpy as np
import airsim

from airsim.types import Pose, Vector3r, Quaternionr

###############################################################################
###############################################################################


def connect(sim_mode: str):
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

    return client


def reset(client) -> None:
    """ Resets a client connected to AirSim and re-enables API control. """
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(True)


###############################################################################
###############################################################################


@contextmanager
def pose_at_simulation_pause(client):
    """ Pauses the AirSim simulation and returns the client's current pose. """
    client.simPause(is_paused=True)
    try:
        yield client.simGetVehiclePose()
    finally:
        client.simPause(is_paused=False)


###############################################################################
###############################################################################


class AirSimImage:
    @staticmethod
    def compute_camera_intrinsics(image_width, image_height, fov, is_degrees=False):
        if is_degrees:
            fov = np.deg2rad(fov)

        f = image_width / (2 * np.tan(fov))
        cu = image_width / 2
        cv = image_height / 2

        return np.array([[f, 0, cu], [0, f, cv], [0, 0, 1]])

    @staticmethod
    def _array_from_uncompressed(image, height, width, flip):
        if not flip:
            return np.fromstring(image, dtype=np.uint8).reshape(height, width, -1)
        return np.flipud(np.fromstring(image, dtype=np.uint8).reshape(height, width, -1))

    @staticmethod
    def get_mono(client, camera_name=ff.CameraName.front_center, vehicle_name=None, flip=False):
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

        return AirSimImage._array_from_uncompressed(
            response.image_data_uint8, response.height, response.width, flip
        )

    @staticmethod
    def get_stereo(client, vehicle_name=None, flip=False):
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
            AirSimImage._array_from_uncompressed(
                response_left.image_data_uint8, response_left.height, response_left.width, flip
            ),
            AirSimImage._array_from_uncompressed(
                response_right.image_data_uint8, response_right.height, response_right.width, flip
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

    @staticmethod
    def local_axes_frame(pose: Pose, normalized: bool = True, flip_z_axis: bool = False):
        """ Returns AirSim's coordinate system axes rotated by the pose's orientation. """
        # NOTE if flip_z_axis=True, then client.simPlotTransforms([pose]) is equivalent to:
        # client.simPlotArrows([pose.position], [pose.position + x_axis])
        # client.simPlotArrows([pose.position], [pose.position + y_axis])
        # client.simPlotArrows([pose.position], [pose.position + z_axis])

        q = pose.orientation
        if normalized:
            q /= q.get_length()

        x_axis = vector_rotated_by_quaternion(FRONT, q)
        y_axis = vector_rotated_by_quaternion(RIGHT, q)
        z_axis = vector_rotated_by_quaternion(UP if flip_z_axis else DOWN, q)

        return x_axis, y_axis, z_axis


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
NED_AXES_FRAME = (FRONT, RIGHT, DOWN)


###############################################################################
###############################################################################


def matrix_from_eularian_angles(roll: float, pitch: float, yaw: float, is_degrees: bool = False) -> np.ndarray:
    # ref.: https://github.com/microsoft/AirSim/blob/master/PythonClient/computer_vision/capture_ir_segmentation.py
    if is_degrees:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])  # roll
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])  # pitch
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])  # yaw

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


def quaternion_to_rotation_axis_angle(q: Quaternionr) -> Tuple[Vector3r, float]:
    axis = Vector3r(q.x_val, q.y_val, q.z_val)
    axis /= axis.get_length()  # normalize
    angle = 2 * np.arccos(q.w_val)
    return axis, angle


def quaternion_from_rotation_axis_angle(axis: Vector3r, angle: float, is_degrees: bool = False) -> Quaternionr:
    if is_degrees:
        angle = np.deg2rad(angle)

    half_angle = angle / 2
    axis /= axis.get_length()  # normalize
    axis *= np.sin(half_angle)
    return Quaternionr(axis.x_val, axis.y_val, axis.z_val, w_val=np.cos(half_angle))


# def quaternion_look_at(source_point: Vector3r, target_point: Vector3r) -> Quaternionr:
#     """ Assuming you are looking at `FRONT` vector, what rotation you need to look at `target_point`?. """
#     # ref.: https://github.com/microsoft/AirSim/blob/master/AirLibUnitTests/QuaternionTest.hpp (lookAt)
#     to_vector = target_point - source_point
#     to_vector /= to_vector.get_length()  # normalize
#
#     axis = FRONT.cross(to_vector)
#     axis = UP if axis.get_length() == 0 else axis / axis.get_length()  # normalize
#
#     return quaternion_from_rotation_axis_angle(axis, angle=np.arccos(FRONT.dot(to_vector)))
#     # return quaternion_from_two_vectors(FRONT, to_vector)


def quaternion_from_two_vectors(a: Vector3r, b: Vector3r) -> Quaternionr:
    """ What rotation (around the intersection of the two vectors) we need to rotate `a` to `b`? """
    # ref.: https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h (FromTwoVectors)
    v0 = a / a.get_length()
    v1 = b / b.get_length()
    c = v1.dot(v0)

    assert c > -1  # FIXME handle the case when the vectors are nearly opposites

    s = np.sqrt((1 + c) * 2)
    axis = v0.cross(v1) / s
    return Quaternionr(axis.x_val, axis.y_val, axis.z_val, w_val=(s / 2))


def quaternion_from_yaw(yaw: float, is_degrees: bool = False) -> Quaternionr:
    # ref.: https://github.com/microsoft/AirSim/blob/master/AirLib/include/common/VectorMath.hpp (quaternionFromYaw)
    return quaternion_from_rotation_axis_angle(DOWN, yaw, is_degrees)


def quaternion_to_yaw(q: Quaternionr) -> float:
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


def quaternion_that_rotates_axes_frame(
    source_xyz_axes: Tuple[Vector3r, Vector3r, Vector3r],
    target_xyz_axes: Tuple[Vector3r, Vector3r, Vector3r],
    assume_normalized: bool = False,  # warn if it isn't
) -> Quaternionr:
    """ Returns the quaternion that rotates vectors from the `source` coordinate system to the `target` axes frame. """
    if not assume_normalized:
        assert all(np.isclose(_.get_length(), 1) for _ in source_xyz_axes + target_xyz_axes)

    # ref.: https://math.stackexchange.com/a/909245
    i, j, k = source_xyz_axes
    a, b, c = target_xyz_axes

    def quaternion_from_vector(v: Vector3r) -> Quaternionr:
        return Quaternionr(v.x_val, v.y_val, v.z_val, w_val=0)

    qx = quaternion_from_vector(a) * quaternion_from_vector(i)
    qy = quaternion_from_vector(b) * quaternion_from_vector(j)
    qz = quaternion_from_vector(c) * quaternion_from_vector(k)

    rx = qx.x_val + qy.x_val + qz.x_val
    ry = qx.y_val + qy.y_val + qz.y_val
    rz = qx.z_val + qy.z_val + qz.z_val
    rw = qx.w_val + qy.w_val + qz.w_val

    rotation = Quaternionr(-rx, -ry, -rz, w_val=(1 - rw))
    length = rotation.get_length()
    assert not np.isclose(length, 0)

    return rotation / length  # normalize


def quaternion_that_rotates_orientation(
    from_orientation: Quaternionr,
    to_orientation: Quaternionr,
) -> Quaternionr:
    """ Returns the quaternion that rotates `from_orientation` into `to_orientation`. """
    return to_orientation * from_orientation.inverse()


def quaternion_orientation_from_eye_to_look_at(
    eye_position: Vector3r, look_at_position: Vector3r
) -> Quaternionr:
    """ Returns the quaternion representing the orientation from `eye_position`
        aiming at `look_at_position`, in AirSim's reference NED axes frame.
    """
    # Compute the forward axis as the vector which points
    # from the camera eye to the region of interest (ROI):
    x_axis = look_at_position - eye_position
    x_axis /= x_axis.get_length()  # normalize

    z_axis = vector_projected_onto_plane(DOWN, plane_normal=x_axis)
    z_axis /= z_axis.get_length()  # normalize

    y_axis = z_axis.cross(x_axis)

    return quaternion_that_rotates_axes_frame(
        source_xyz_axes=NED_AXES_FRAME,
        target_xyz_axes=(x_axis, y_axis, z_axis),
    )


def vector_projected_onto_vector(v: Vector3r, u: Vector3r) -> Vector3r:
    """ Returns the projection of `v` onto `u`. """
    return u * v.dot(u) / u.dot(u)


def vector_projected_onto_plane(v: Vector3r, plane_normal: Vector3r) -> Vector3r:
    """ Returns the projection of `v` onto the plane defined by `plane_normal`. """
    n = plane_normal / plane_normal.get_length()
    return v - vector_projected_onto_vector(v, n)


def vector_rotated_by_quaternion(v: Vector3r, q: Quaternionr) -> Vector3r:
    q /= q.get_length()  # normalize

    # Extract the vector and scalar parts of q:
    u, s = Vector3r(q.x_val, q.y_val, q.z_val), q.w_val

    # NOTE the results from these two methods are the same up to 7 decimal places.

    # ref.: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    # return u * (2 * u.dot(v)) + v * (s * s - u.dot(u)) + u.cross(v) * (2 * s)

    # ref.: https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h (_transformVector)
    uv = u.cross(v)
    uv += uv
    return v + uv * s + u.cross(uv)


def vector_rotated_by_quaternion_reversed(v: Vector3r, q: Quaternionr) -> Vector3r:
    # NOTE we could have used q.conjugate(), if we assumed q was a unit quaternion
    return vector_rotated_by_quaternion(v, q.inverse())


def point_to_plane_distance(p: Vector3r, n: Vector3r, d: float):
    """ Returns the distance from a point `p` to a plane with normal `n`
        and distance `d` from the coordinate system's origin (along `n`).
    """
    return p.dot(n) - d


if False:
    # ref.: https://docs.unrealengine.com/en-US/API/Runtime/Core/Math/FRotator/index.html
    #
    # `pitch`: Rotation around the right axis (around Y axis), Looking up and down (0=Straight Ahead, +Up, -Down).
    # `yaw`:   Rotation around the up axis (around Z axis), Running in circles 0=East, +North, -South.
    # `roll`:  Rotation around the forward axis (around X axis), Tilting your head, 0=Straight, +Clockwise, -CCW.

    def matrix_from_rotator(pitch: float, yaw: float, roll: float, is_degrees: bool = False) -> np.ndarray:
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


def viewport_vectors(
    pose: Pose, hfov_degrees: float, aspect_ratio: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    front_axis, right_axis, up_axis = AirSimNedTransform.local_axes_frame(pose)

    front_axis = (front_axis / front_axis.get_length()).to_numpy_array()
    right_axis = (right_axis / right_axis.get_length()).to_numpy_array()
    up_axis = (up_axis / up_axis.get_length()).to_numpy_array()

    half_hfov = 0.5 * np.deg2rad(hfov_degrees)
    half_vfov = half_hfov / aspect_ratio
    hcos, hsin = np.cos(half_hfov), np.sin(half_hfov)
    vcos, vsin = np.cos(half_vfov), np.sin(half_vfov)

    # ref.: https://steve.hollasch.net/cgindex/math/rotvec.html

    up_matrix = np.array(
        [
            [0, up_axis[2], -up_axis[1]],
            [-up_axis[2], 0, up_axis[0]],
            [up_axis[1], -up_axis[0], 0],
        ]
    )
    right_matrix = np.array(
        [
            [0, right_axis[2], -right_axis[1]],
            [-right_axis[2], 0, right_axis[0]],
            [right_axis[1], -right_axis[0], 0],
        ]
    )

    right_matrix1 = vsin * right_matrix
    right_matrix2 = (1 - vcos) * right_matrix @ right_matrix

    up_matrix1 = hsin * up_matrix
    up_matrix2 = (1 - hcos) * up_matrix @ up_matrix

    eye_to_top = front_axis @ (np.eye(3) + right_matrix1 + right_matrix2)
    eye_to_top_left = eye_to_top @ (np.eye(3) - up_matrix1 + up_matrix2)
    eye_to_top_right = eye_to_top @ (np.eye(3) + up_matrix1 + up_matrix2)

    eye_to_bottom = front_axis @ (np.eye(3) - right_matrix1 + right_matrix2)
    eye_to_bottom_left = eye_to_bottom @ (np.eye(3) - up_matrix1 + up_matrix2)
    eye_to_bottom_right = eye_to_bottom @ (np.eye(3) + up_matrix1 + up_matrix2)

    # eye = pose.position.to_numpy_array()

    # # viewport corners
    # top_left = Vector3r(*(eye + eye_to_top_left))
    # top_right = Vector3r(*(eye + eye_to_top_right))
    # bottom_left = Vector3r(*(eye + eye_to_bottom_left))
    # bottom_right = Vector3r(*(eye + eye_to_bottom_right))

    # def cross_norm(v1, v2):
    #     v1_cross_v2 = np.cross(v1, v2)
    #     return v1_cross_v2 / np.linalg.norm(v1_cross_v2)

    # # frustum plane normals
    # top = cross_norm(eye_to_top_left, eye_to_top_right)
    # right = cross_norm(eye_to_top_right, eye_to_bottom_right)
    # bottom = cross_norm(eye_to_bottom_right, eye_to_bottom_left)
    # left = cross_norm(eye_to_bottom_left, eye_to_top_left)
    # near = np.array([0, 0, 1])
    # far = np.array([0, 0, -1])

    return eye_to_top_left, eye_to_top_right, eye_to_bottom_left, eye_to_bottom_right


def frustum_plot_list_from_viewport_vectors(
    pose: Pose,
    eye_to_top_left: np.ndarray,
    eye_to_top_right: np.ndarray,
    eye_to_bottom_left: np.ndarray,
    eye_to_bottom_right: np.ndarray,
    scale: float = 2.0
) -> List[Vector3r]:
    eye = pose.position

    top_left = eye + Vector3r(*(eye_to_top_left * scale))
    top_right = eye + Vector3r(*(eye_to_top_right * scale))
    bottom_left = eye + Vector3r(*(eye_to_bottom_left * scale))
    bottom_right = eye + Vector3r(*(eye_to_bottom_right * scale))

    return [
        eye, top_left,
        eye, top_right,
        eye, bottom_left,
        eye, bottom_right,

        top_left, top_right,
        top_right, bottom_right,
        bottom_right, bottom_left,
        bottom_left, top_left,
    ]


def frustum_plane_normals_from_viewport_vectors(
    eye_to_top_left: np.ndarray,
    eye_to_top_right: np.ndarray,
    eye_to_bottom_left: np.ndarray,
    eye_to_bottom_right: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    def cross_norm(v1, v2):
        v1_cross_v2 = np.cross(v1, v2)
        return v1_cross_v2 / np.linalg.norm(v1_cross_v2)

    top = cross_norm(eye_to_top_left, eye_to_top_right)
    left = cross_norm(eye_to_bottom_left, eye_to_top_left)
    right = cross_norm(eye_to_top_right, eye_to_bottom_right)
    bottom = cross_norm(eye_to_bottom_right, eye_to_bottom_left)

    return top, left, right, bottom


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
        """ Represents an item from AirSim's recording file (i.e. a row in `airsim_rec.txt`).
            Reference: https://github.com/microsoft/AirSim/blob/master/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp
        """
        # assert os.path.isfile(image_file), image_file
        self.time_stamp = time_stamp
        self.image_file = image_file
        self.position = Vector3r(pos_x, pos_y, pos_z)
        self.orientation = Quaternionr(q_x, q_y, q_z, q_w)

    def __repr__(self) -> str:
        return f"AirSimRecord(time_stamp={self.time_stamp}, image_file={self.image_file}, position={self.position}, orientation={self.orientation})"

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

    @staticmethod
    def make_header_string(skip_time_stamp: bool = False, skip_image_file: bool = False) -> str:
        # return "TimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z\tImageFile"
        return (
            ("" if skip_time_stamp else "TimeStamp\t")
            + "POS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z"
            + ("" if skip_image_file else "\tImageFile")
        )

    @staticmethod
    def make_line_string(
        position: Vector3r,
        orientation: Quaternionr,
        time_stamp: Optional[str],
        image_file: Optional[str],
    ) -> str:
        pos = position
        q = orientation

        pos_string = "\t".join([str(_) for _ in [pos.x_val, pos.y_val, pos.z_val]])
        q_string = "\t".join([str(_) for _ in [q.w_val, q.x_val, q.y_val, q.z_val]])

        return (
            ("" if time_stamp is None else f"{time_stamp}\t")
            + f"{pos_string}\t{q_string}"
            + ("" if image_file is None else f"\t{image_file}")
        )


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

    def as_quaternion(self) -> Quaternionr:
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
            position: Vector3r = None,
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
            position: Vector3r = None,  # NOTE in global NED coordinates, SI units and origin at PlayerStart
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

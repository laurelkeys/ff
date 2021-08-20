from __future__ import annotations

import os
import json
import glob
import enum
import contextlib

from typing import Dict, List, Tuple, Union, Optional, cast

import numpy as np
import airsim
import open3d as o3d
from airsim import client

import fff_config

###############################################################################
#### Environment Variables ####################################################
###############################################################################


# e.g.: "C:/path/to/UE_4.25/Engine/Binaries/Win64/UE4Editor.exe"
UE4EDITOR_EXE_PATH = os.getenv(
    "fff_UE4EDITOR_EXE_PATH", default=fff_config.DEFAULT_UE4EDITOR_EXE_PATH
)

# e.g.: "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/Common7/IDE/devenv.exe"
VS_DEVENV_EXE_PATH = os.getenv(
    "fff_VS_DEVENV_EXE_PATH", default=fff_config.DEFAULT_VS_DEVENV_EXE_PATH
)

# e.g.: "C:/Documents/AirSim/settings.json"
AIRSIM_SETTINGS_JSON_PATH = os.getenv(
    "fff_AIRSIM_SETTINGS_JSON_PATH", default=fff_config.DEFAULT_AIRSIM_SETTINGS_JSON_PATH
)

# e.g.: "C:/path/to/git/AirSim/PythonClient/airsim"
AIRSIM_PYCLIENT_FOLDER_PATH = os.getenv(
    "fff_AIRSIM_PYCLIENT_FOLDER_PATH", default=fff_config.DEFAULT_AIRSIM_PYCLIENT_FOLDER_PATH
)

# e.g.: {"doc": "C:/Documents/AirSim/", "dev": "C:/dev/Environments/"}
ENVIRONMENT_FOLDER_ALIASES_DICT = {**fff_config.DEFAULT_ENVIRONMENT_FOLDER_ALIASES_DICT}


###############################################################################
#### Helper / Utility Functions ###############################################
###############################################################################


def input_or_exit(prompt: str) -> str:
    """Simple wrapper around `input()` to catch `KeyboardInterrupt` and `exit()`."""
    try:
        return input(prompt)
    except KeyboardInterrupt:
        exit()


###############################################################################
#### Console Logger ###########################################################
###############################################################################


# fmt: off
class Color(enum.Enum):
    RESET   = "\033[0m"
    GREY    = "\033[90m"
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
# fmt: on


def log_colored(color: Color, *args, **kwargs):
    print(color.value, end="")
    print(*args, Color.RESET.value, **kwargs)


def log(*args, **kwargs):
    log_colored(Color.GREEN, *args, **kwargs)


def log_info(*args, log_prefix="[INFO]", **kwargs):
    log_colored(Color.YELLOW, log_prefix, *args, **kwargs)


def log_error(*args, log_prefix="[ERROR]", **kwargs):
    log_colored(Color.RED, log_prefix, *args, **kwargs)


def log_debug(*args, log_prefix="[DEBUG]", **kwargs):
    log_colored(Color.GREY, log_prefix, *args, **kwargs)


def log_warning(*args, log_prefix="[WARNING]", **kwargs):
    log_colored(Color.MAGENTA, log_prefix, *args, **kwargs)


###############################################################################
#### Rgba Colors ##############################################################
###############################################################################


class Rgba(tuple):
    White = (1.0, 1.0, 1.0, 1.0)
    Black = (0.0, 0.0, 0.0, 1.0)

    Red = (1.0, 0.0, 0.0, 1.0)
    Green = (0.0, 1.0, 0.0, 1.0)
    Blue = (0.0, 0.0, 1.0, 1.0)

    Cyan = (0.0, 1.0, 1.0, 1.0)
    Magenta = (1.0, 0.0, 1.0, 1.0)
    Yellow = (1.0, 1.0, 0.0, 1.0)

    def __new__(cls, r: float, g: float, b: float, alpha: float = 1.0):
        rgba = (r, g, b, alpha)
        assert all(0.0 <= ch <= 1.0 for ch in rgba), rgba
        return super(Rgba, cls).__new__(cls, rgba)

    def __init__(self, r: float, g: float, b: float, alpha: float = 1.0):
        """Construct a new color from RGBA values in `[0.0, 1.0]`."""
        self.r = self[0]
        self.g = self[1]
        self.b = self[2]
        self.a = self[3]

    @staticmethod
    def from255(r: int, g: int, b: int, alpha: float = 1.0) -> Rgba:
        """Construct a new color by converting RGB values from `[0, 255]` to `[0.0, 1.0]`."""
        return Rgba(r / 255, g / 255, b / 255, alpha)


###############################################################################
#### AirSim Math ##############################################################
###############################################################################


# fmt: off
def v2v(vector_xyz: np.ndarray): assert vector_xyz.size == 3; return airsim.Vector3r(*vector_xyz)
def q2q(quaternion_xyzw: np.ndarray): assert quaternion_xyzw.size == 4; return airsim.Quaternionr(*quaternion_xyzw)
def q2v(quaternion: airsim.Quaternionr): return airsim.Vector3r(quaternion.x_val, quaternion.y_val, quaternion.z_val)
def v2q(vector: airsim.Vector3r, w: float = 0): return airsim.Quaternionr(vector.x_val, vector.y_val, vector.z_val, w)
# fmt: on


TypeOfVector = Union[airsim.Vector3r, np.ndarray]
TypeOfQuaternion = Union[airsim.Quaternionr, np.ndarray]
TypeOfVectorOrQuaternion = Union[airsim.Vector3r, airsim.Quaternionr, np.ndarray]


def coordinates_str(
    vector_or_quaternion: TypeOfVectorOrQuaternion,
    decimal_points: int = 2,
    show_prefixes: bool = True,
    prefixes: List[str] = None,
) -> str:
    """Returns a string representation of all coordinates."""
    prefixes = ["x=", "y=", "z=", "w="] if show_prefixes else ["", "", "", ""]
    return ", ".join(
        [f"{prefix}{_:.{decimal_points}f}" for prefix, _ in zip(prefixes, [*vector_or_quaternion])]
    )


def xyz_xyzw_of_client(client_state: airsim.MultirotorState) -> Tuple[np.ndarray, np.ndarray]:
    position = client_state.kinematics_estimated.position
    orientation = client_state.kinematics_estimated.orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


def xyz_xyzw_of_camera(camera_info: airsim.CameraInfo) -> Tuple[np.ndarray, np.ndarray]:
    position = camera_info.pose.position
    orientation = camera_info.pose.orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


def xyz_xyzw_of_image(image_response: airsim.ImageResponse) -> Tuple[np.ndarray, np.ndarray]:
    position = image_response.camera_position
    orientation = image_response.camera_orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


def all_close(
    lhs_vector_or_quaternion: TypeOfVectorOrQuaternion,
    rhs_vector_or_quaternion: TypeOfVectorOrQuaternion,
    eps=1e-7,
) -> bool:
    """Returns true iff `lhs` and `rhs` are element-wise equal within `eps` tolerance."""
    assert type(lhs_vector_or_quaternion) == type(rhs_vector_or_quaternion)
    return all(
        abs(lhs - rhs) <= eps
        for lhs, rhs in zip([*lhs_vector_or_quaternion], [*rhs_vector_or_quaternion])
    )


def vector_angle_between(v1: TypeOfVector, v2: TypeOfVector, in_degrees=False) -> float:
    """Returns the angle between vectors `v1` and `v2`."""
    v1 = v1.to_numpy_array() if isinstance(v1, airsim.Vector3r) else v1
    v2 = v2.to_numpy_array() if isinstance(v2, airsim.Vector3r) else v2

    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))

    if cos_angle > 1.0:
        return 0.0  # arccos(1.0)
    elif cos_angle < -1.0:
        return np.pi  # arccos(-1.0)
    else:
        return np.arccos(cos_angle)


def vector_flip_z(v: TypeOfVector) -> TypeOfVector:
    """Returns a copy of the vector, but with its Z-coordinate negated."""
    if isinstance(v, airsim.Vector3r):
        return airsim.Vector3r(v.x_val, v.y_val, -v.z_val)
    else:
        return np.negative(v, where=np.ndarray([False, False, True]))


#
# Reference:
#   https://stackoverflow.com/questions/32438252/efficient-way-to-apply-mirror-effect-on-quaternion-rotation/40334755#40334755
#

# fmt: off
def quaternion_flip_z(q: TypeOfQuaternion) -> TypeOfQuaternion:
    if isinstance(q, airsim.Quaternionr):
        return airsim.Quaternionr(-q.x_val, -q.y_val, q.z_val, w_val=q.w_val)
    return np.negative(q, where=np.ndarray([True, True, False, False]))

def quaternion_flip_y(q: TypeOfQuaternion) -> TypeOfQuaternion:
    if isinstance(q, airsim.Quaternionr):
        return airsim.Quaternionr(-q.x_val, q.y_val, -q.z_val, w_val=q.w_val)
    return np.negative(q, where=np.ndarray([True, False, True, False]))

def quaternion_flip_x(q: TypeOfQuaternion) -> TypeOfQuaternion:
    if isinstance(q, airsim.Quaternionr):
        return airsim.Quaternionr(q.x_val, -q.y_val, -q.z_val, w_val=q.w_val)
    return np.negative(q, where=np.ndarray([False, True, True, False]))
# fmt: on


# fmt: off

# NOTE these values can be used with YawMode(is_rate=False) to change orientation
YAW_N =   0  # North
YAW_E =  90  # East
YAW_W = -90  # West
YAW_S = 180  # South

# NOTE see AirLib/include/common/VectorMath.hpp
FRONT = airsim.Vector3r( 1,  0,  0)  # North
BACK  = airsim.Vector3r(-1,  0,  0)  # South
DOWN  = airsim.Vector3r( 0,  0,  1)
UP    = airsim.Vector3r( 0,  0, -1)
RIGHT = airsim.Vector3r( 0,  1,  0)  # East
LEFT  = airsim.Vector3r( 0, -1,  0)  # West

# NOTE AirSim uses the North-East-Down (NED) coordinate system, while
# Unreal Units (UU) have Z-axis flipped, and use cm instead of meters.
NED_AXIS_FRAME = FRONT, RIGHT, DOWN  # right-handed
UE4_AXIS_FRAME = FRONT, RIGHT, UP    # left-handed
UE4_TO_NED_SCALE = 0.01
NED_TO_UE4_SCALE = 100

# NOTE this is used internally, alongside the TypeOfVectorOrQuaternion type alias
class VectorOrQuaternion(enum.Enum):
    Vector         = 1  # airsim.Vector3r
    Quaternion     = 2  # airsim.Quaternionr
    VectorXYZ      = 3  # np.ndarray ([x, y, z])
    QuaternionXYZW = 4  # np.ndarray ([x, y, z, w])

    @staticmethod
    def type_of(vector_or_quaternion: TypeOfVectorOrQuaternion):
        if isinstance(vector_or_quaternion, airsim.Vector3r):
            return VectorOrQuaternion.Vector
        if isinstance(vector_or_quaternion, airsim.Quaternionr):
            return VectorOrQuaternion.Quaternion
        array = cast(np.ndarray, vector_or_quaternion)
        assert isinstance(vector_or_quaternion, np.ndarray), type(vector_or_quaternion)
        return {3: VectorOrQuaternion.VectorXYZ, 4: VectorOrQuaternion.QuaternionXYZW}[array.size]

    @staticmethod
    def is_vector_type(typ: VectorOrQuaternion):
        return typ == VectorOrQuaternion.Vector or typ == VectorOrQuaternion.VectorXYZ

    @staticmethod
    def is_quaternion_type(typ: VectorOrQuaternion):
        return typ == VectorOrQuaternion.Quaternion or typ == VectorOrQuaternion.QuaternionXYZW

# fmt: on


def ue4_to_ned(vector_or_quaternion: TypeOfVectorOrQuaternion) -> TypeOfVectorOrQuaternion:
    """Converts Unreal Units to AirSim's NED system (NOTE: vector coordinates are scaled)."""
    typ = VectorOrQuaternion.type_of(vector_or_quaternion)
    return {
        VectorOrQuaternion.Vector: lambda v: vector_flip_z(v) * UE4_TO_NED_SCALE,
        VectorOrQuaternion.VectorXYZ: lambda v: vector_flip_z(v) * UE4_TO_NED_SCALE,
        VectorOrQuaternion.Quaternion: lambda q: quaternion_flip_z(q),
        VectorOrQuaternion.QuaternionXYZW: lambda q: quaternion_flip_z(q),
    }[typ]


def ned_to_ue4(vector_or_quaternion: TypeOfVectorOrQuaternion) -> TypeOfVectorOrQuaternion:
    """Converts AirSim's NED system to Unreal Units (NOTE: vector coordinates are scaled)."""
    typ = VectorOrQuaternion.type_of(vector_or_quaternion)
    return {
        VectorOrQuaternion.Vector: lambda v: vector_flip_z(v) * NED_TO_UE4_SCALE,
        VectorOrQuaternion.VectorXYZ: lambda v: vector_flip_z(v) * NED_TO_UE4_SCALE,
        VectorOrQuaternion.Quaternion: lambda q: quaternion_flip_z(q),
        VectorOrQuaternion.QuaternionXYZW: lambda q: quaternion_flip_z(q),
    }[typ]


def local_axis_frame(pose: airsim.Pose, normalize: bool = True, flip_z_axis: bool = False):
    """Returns AirSim's NED coordinate system axis rotated by the pose's orientation."""
    # NOTE if flip_z_axis=True, then client.simPlotTransforms([pose]) is equivalent to:
    # client.simPlotArrows([pose.position], [pose.position + x_axis])
    # client.simPlotArrows([pose.position], [pose.position + y_axis])
    # client.simPlotArrows([pose.position], [pose.position + z_axis])

    q = pose.orientation
    if normalize:
        q /= q.get_length()

    x_axis = vector_rotated_by_quaternion(FRONT, q)
    y_axis = vector_rotated_by_quaternion(RIGHT, q)
    z_axis = vector_rotated_by_quaternion(UP if flip_z_axis else DOWN, q)

    return x_axis, y_axis, z_axis


###############################################################################
#### AirSim Extensions ########################################################
###############################################################################


# fmt: off
class CameraName(enum.Enum):
    front_center  = "front_center"   # 0
    front_right   = "front_right"    # 1
    front_left    = "front_left"     # 2
    bottom_center = "bottom_center"  # 3
    back_center   = "back_center"    # 4
    _0, _1, _2, _3, _4 = "0", "1", "2", "3", "4"

class SimMode(enum.Enum):
    Default        = ""
    Car            = "Car"
    Multirotor     = "Multirotor"
    ComputerVision = "ComputerVision"


class ViewMode(enum.Enum):
    Default        = ""
    FlyWithMe      = "FlyWithMe"       # Multirotor's default
    GroundObserver = "GroundObserver"
    Fpv            = "Fpv"
    Manual         = "Manual"
    SpringArmChase = "SpringArmChase"  # Car's default
    NoDisplay      = "NoDisplay"
# fmt: on


TypeOfAirSimClient = Union[airsim.MultirotorClient, airsim.VehicleClient]


def connect(sim_mode: str, vehicle_name: str = None) -> TypeOfAirSimClient:
    """Returns a (Multirotor or ComputerVision) client connected to AirSim."""
    assert sim_mode in [SimMode.Multirotor, SimMode.ComputerVision], sim_mode
    vehicle_name = "" if vehicle_name is None else vehicle_name

    if sim_mode == SimMode.Multirotor:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True, vehicle_name)
        client.armDisarm(True, vehicle_name)
    else:
        client = airsim.VehicleClient()
        client.confirmConnection()

    return client


def reset(client: TypeOfAirSimClient, vehicle_name: str = None) -> None:
    """Resets a client connected to AirSim and re-enables API control."""
    vehicle_name = "" if vehicle_name is None else vehicle_name

    client.reset()
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)


@contextlib.contextmanager
def paused_simulation(client: TypeOfAirSimClient):
    """Pauses the AirSim simulation loop."""
    client.simPause(is_paused=True)
    try:
        yield
    finally:
        client.simPause(is_paused=False)


def airsim_settings_str_from_dict(settings: dict) -> str:
    """Converts a `dict` representing AirSim's settings.json to a JSON
    string that can be passed to it, through the `--settings` argument.
    """
    # NOTE for some reason, at least on Windows, passing the settings
    # as a string directly in the command line does not seem to work.
    # So, this is used to correctly create one (see AirSim/issues/2824).
    return json.dumps(settings).replace('"', '\\"')


def airsim_possible_environment_paths(
    env_root: str, exts: List[str] = ["uproject", "sln", "exe"]
) -> List[str]:
    """Searches for valid environment files with the following patterns:
    - `{env_root}/*.{ext}`
    - `{env_root}/*/*.{ext}`
    """
    env_paths = []
    for ext in exts:
        env_paths.extend(glob.glob(os.path.join(env_root, f"*.{ext}")))
        env_paths.extend(glob.glob(os.path.join(env_root, "*", f"*.{ext}")))
    return env_paths


###############################################################################
#### Meshroom #################################################################
###############################################################################


###############################################################################
#### Open3D ###################################################################
###############################################################################

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
#### AirSim Math ##############################################################
###############################################################################


TypeOfAirSimClient = Union[airsim.MultirotorClient, airsim.VehicleClient]

TypeOfVector = Union[airsim.Vector3r, np.ndarray]
TypeOfQuaternionr = Union[airsim.Quaternionr, np.ndarray]
TypeOfVectorOrQuaternionr = Union[airsim.Vector3r, airsim.Quaternionr, np.ndarray]

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

NED_AXIS_FRAME = FRONT, RIGHT, DOWN  # North, West, Down

# fmt: on


def coordinates_str(
    vector_or_quaternion: TypeOfVectorOrQuaternionr,
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


def xyz_xyzw_of_camera(camera_info: airsim.CameraInfo):
    position = camera_info.pose.position
    orientation = camera_info.pose.orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


def xyz_xyzw_of_image(image_response: airsim.ImageResponse):
    position = image_response.camera_position
    orientation = image_response.camera_orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


def all_close(
    lhs_vector_or_quaternion: TypeOfVectorOrQuaternionr,
    rhs_vector_or_quaternion: TypeOfVectorOrQuaternionr,
    eps=1e-7,
) -> bool:
    """Returns true iff `lhs` and `rhs` are element-wise equal within `eps` tolerance."""
    assert type(lhs_vector_or_quaternion) == type(rhs_vector_or_quaternion)
    return all(
        abs(lhs - rhs) <= eps
        for lhs, rhs in zip([*lhs_vector_or_quaternion], [*rhs_vector_or_quaternion])
    )


def flipped_z(v: TypeOfVector) -> TypeOfVector:
    """Returns a copy of the vector, but with its Z-coordinate negated."""
    if isinstance(v, airsim.Vector3r):
        return airsim.Vector3r(v.x_val, v.y_val, -v.z_val)
    else:
        return np.negative(v, where=np.ndarray([False, False, True]))


def angle_between(v1: TypeOfVector, v2: TypeOfVector, in_degrees=False) -> float:
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

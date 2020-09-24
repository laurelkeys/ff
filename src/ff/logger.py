from enum import Enum

from .types import to_xyz_str, to_xyzw_str, angles_to_str

###############################################################################
###############################################################################

class Color(Enum):
    RESET   = '\033[0m'
    GREY    = '\033[90m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'

def log_colored(color: Color, *args, **kwargs):
    print(color.value, end="")
    print(*args, Color.RESET.value, **kwargs)

def log(*args, **kwargs):
    log_colored(Color.GREEN, "[ff]", *args, **kwargs)

def log_info(*args, **kwargs):
    log_colored(Color.YELLOW, "[INFO]", *args, **kwargs)

def log_error(*args, **kwargs):
    log_colored(Color.RED, "[ERROR]", *args, **kwargs)

def log_debug(*args, **kwargs):
    log_colored(Color.GREY, "[DEBUG]", *args, **kwargs)

def log_warning(*args, **kwargs):
    log_colored(Color.MAGENTA, "[WARNING]", *args, **kwargs)


###############################################################################
###############################################################################


def print_airsim_path(airsim_path):
    path_str = f"'airsim' path: {airsim_path[0]}"
    print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")


def print_pose(vehicle_pose, to_eularian_angles=None):
    print_str = [
        " VehiclePose \n",
        f"     .position    = {to_xyz_str(vehicle_pose.position)}\n",
        f"     .orientation = {to_xyzw_str(vehicle_pose.orientation)}\n",
        f"                    {angles_to_str(to_eularian_angles(vehicle_pose.orientation))}\n",
    ]
    log(*(print_str if to_eularian_angles is not None else print_str[:-1]))


###############################################################################
###############################################################################

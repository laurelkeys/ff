from .types import to_xyz_str, to_xyzw_str, angles_to_str

###############################################################################
###############################################################################


def log_info(*args, **kwargs):
    print("[INFO]", *args, **kwargs)

def log_error(*args, **kwargs):
    print("[ERROR]", *args, **kwargs)

def log_debug(*args, **kwargs):
    print("[DEBUG]", *args, **kwargs)

def log_warning(*args, **kwargs):
    print("[WARNING]", *args, **kwargs)


###############################################################################
###############################################################################


def print_airsim_path(airsim_path):
    path_str = f"'airsim' path: {airsim_path[0]}"
    print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")


def print_pose(vehicle_pose, to_eularian_angles=None):
    print_str = [
        "[ff] VehiclePose \n",
        f"     .position    = {to_xyz_str(vehicle_pose.position)}\n",
        f"     .orientation = {to_xyzw_str(vehicle_pose.orientation)}\n",
        f"                    {angles_to_str(to_eularian_angles(vehicle_pose.orientation))}\n",
    ]
    print(*(print_str if to_eularian_angles is not None else print_str[:-1]))


###############################################################################
###############################################################################

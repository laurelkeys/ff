
###############################################################################
###############################################################################


class CameraName:
    front_center  = "front_center"  # 0
    front_right   = "front_right"   # 1
    front_left    = "front_left"    # 2
    bottom_center = "bottom_center" # 3
    back_center   = "back_center"   # 4


class SimMode:
    Default        = ""
    Car            = "Car"
    Multirotor     = "Multirotor"
    ComputerVision = "ComputerVision"
    _list_all      = [Default, Car, Multirotor, ComputerVision]


class ViewMode:
    Default        = ""
    FlyWithMe      = "FlyWithMe"
    GroundObserver = "GroundObserver"
    Fpv            = "Fpv"
    Manual         = "Manual"
    SpringArmChase = "SpringArmChase"
    NoDisplay      = "NoDisplay"
    _list_all      = [Default, FlyWithMe, GroundObserver, Fpv, Manual, SpringArmChase, NoDisplay]


###############################################################################
###############################################################################


def xyz_xyzw_of_client(client_state):
    position = client_state.kinematics_estimated.position
    orientation = client_state.kinematics_estimated.orientation
    return position.to_numpy_array(), orientation.to_numpy_array()

def xyz_xyzw_of_camera(camera_info):
    position = camera_info.pose.position
    orientation = camera_info.pose.orientation
    return position.to_numpy_array(), orientation.to_numpy_array()

def xyz_xyzw_of_image(image_response):
    position = image_response.camera_position
    orientation = image_response.camera_orientation
    return position.to_numpy_array(), orientation.to_numpy_array()


###############################################################################
###############################################################################

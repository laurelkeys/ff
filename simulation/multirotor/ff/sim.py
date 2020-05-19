import time


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


###############################################################################
###############################################################################

# TODO:
# - https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696
# - https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212

# def move_to_position(client, x, y, z, velocity):
#     '''  '''
#     current_pos = client.simGetVehiclePose().position
#     # current_pos = client.getMultirotorState().kinematics_estimated.position

#     dx, dy, dz = x - current_pos.x_val, y - current_pos.y_val, z - current_pos.z_val
#     dt = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5 / velocity
#     vx, vy, vz = dx / dt, dy / dt, dz / dt

#     client.moveByVelocityAsync(vx, vy, vz, duration=dt)
#     time.sleep(dt)
#     client.moveByVelocityAsync(0, 0, 0, duration=2)
#     client.hoverAsync().join()

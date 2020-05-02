import time


def move_to_position(client, x, y, z, velocity):
    ''' https://github.com/microsoft/AirSim/issues/1677#issuecomment-469999696 '''
    current_pos = client.simGetVehiclePose().position   # client.getMultirotorState().kinematics_estimated.position

    dx, dy, dz = x - current_pos.x_val, y - current_pos.y_val, z - current_pos.z_val
    dt = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5 / velocity
    vx, vy, vz = dx / dt, dy / dt, dz / dt

    client.moveByVelocityAsync(vx, vy, vz, duration=dt)
    time.sleep(dt)
    client.moveByVelocityAsync(0, 0, 0, duration=2)
    client.hoverAsync().join()


# TODO https://github.com/microsoft/AirSim/issues/1677#issuecomment-605440212

from __future__ import annotations

import time

from typing import List

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Vector3r, Pose


###############################################################################
## Config #####################################################################
###############################################################################


# https://microsoft.github.io/AirSim/apis/#apis-for-multirotor
USE_AIRSIM_HIGH_LEVEL_CONTROL = True

CONFIRMATION_DISTANCE = 3.0
WAIT_TIME = 0.1


###############################################################################
## Flight controller ##########################################################
###############################################################################


# TODO make a "builder" way to use the controller, e.g.:
#      |
#      |  Controller.teleport(client, to=Pose())
#      |  Controller.fly_path(client, path=[])
#
#      would become something like:
#      |
#      |  Controller.with(client)
#      |            .teleport(to=Pose())
#      |            .fly_path(path=[])


class Controller:
    """ A (currently) blocking flight controller which abstracts the flying method. """

    @staticmethod
    def teleport(client: airsim.MultirotorClient, to: Pose, ignore_collison: bool = True) -> None:
        # HACK see https://github.com/Microsoft/AirSim/issues/1618#issuecomment-689152817
        client.simSetVehiclePose(to, ignore_collison)
        client.moveToPositionAsync(*ff.to_xyz_tuple(to.position), velocity=1)

    @staticmethod
    def fly_path(
        client: airsim.MultirotorClient,
        path: List[Vector3r],
        velocity: float = 2.0,
        timeout_sec: float = 3e38,  # FIXME make this a constant
    ) -> None:
        ff.log_warning(f"{USE_AIRSIM_HIGH_LEVEL_CONTROL=}")

        if USE_AIRSIM_HIGH_LEVEL_CONTROL:
            client.moveOnPathAsync(path, velocity, timeout_sec).join()
        else:
            _fly_path(client, path, velocity, timeout_sec)


###############################################################################
## Internal functions #########################################################
###############################################################################


def _fly_path(
    client: airsim.MultirotorClient, path: List[Vector3r], velocity: float, timeout_sec: float
):

    for i, point in enumerate(path):
        ff.log_debug(f"Going to {ff.to_xyz_str(point)}")

        try:
            next_point = path[i + 1]
        except:
            next_point = None  # NOTE final point

        future = client.moveToPositionAsync(*ff.to_xyz_tuple(point), velocity, timeout_sec)
        curr_pos = client.simGetVehiclePose().position

        # https://github.com/Microsoft/AirSim/issues/1677
        # https://github.com/Microsoft/AirSim/issues/2974

        if next_point is not None:
            if i == 0:
                future.join()  # NOTE first point
            else:
                # "spin lock" untill we are close to the next point
                while not ff.Vec3.all_close(
                    ff.Vec3.from_Vector3r(curr_pos),
                    ff.Vec3.from_Vector3r(point),
                    eps=CONFIRMATION_DISTANCE,
                ):
                    curr_pos = client.simGetVehiclePose().position
                    time.sleep(WAIT_TIME)

                future.join()
                time.sleep(0.5)  # FIXME testing

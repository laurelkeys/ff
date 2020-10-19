from __future__ import annotations

import time

from typing import List

import ff

from ff.types import Vec3

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Pose, Vector3r


###############################################################################
## Config #####################################################################
###############################################################################


# https://microsoft.github.io/AirSim/apis/#apis-for-multirotor
USE_AIRSIM_HIGH_LEVEL_CONTROL = False
ff.log_warning(f"{USE_AIRSIM_HIGH_LEVEL_CONTROL=}")

CONFIRMATION_DISTANCE = 3.0
WAIT_TIME = 0.1  # FIXME using `time.sleep` won't go well with changing clock speeds


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
    waypoint_count = len(path)
    assert waypoint_count >= 2  # FIXME handle corner cases

    first_point, *middle_points, final_point = [Vec3.from_Vector3r(waypoint) for waypoint in path]

    client.moveToPositionAsync(*first_point, velocity, timeout_sec).join()

    for next_pos in middle_points:
        # https://github.com/Microsoft/AirSim/issues/1677
        # https://github.com/Microsoft/AirSim/issues/2974

        future = client.moveToPositionAsync(*next_pos, velocity, timeout_sec)
        curr_pos = Vec3.from_Vector3r(client.simGetVehiclePose().position)

        # "spin lock" until we are close to the next point
        while not Vec3.all_close(curr_pos, next_pos, eps=CONFIRMATION_DISTANCE):
            curr_pos = Vec3.from_Vector3r(client.simGetVehiclePose().position)
            time.sleep(WAIT_TIME)

        future.join()  # wait for AirSim's API to also recognize we've arrived
        time.sleep(0.5)  # FIXME stopping for some time minimizes overshooting

    client.moveToPositionAsync(*final_point, velocity, timeout_sec).join()

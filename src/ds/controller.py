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
            _fly_path2(client, path, velocity, timeout_sec)

    ###########################################################################
    ## Auxiliary methods (i.e. don't receive a `client`) ######################
    ###########################################################################

    @staticmethod
    def augment_path(path: List[Vector3r], max_dist: float) -> List[Vector3r]:
        """ Adds new (intermediate) waypoints to `path` so that no two consecutive
            points are more than `max_dist` apart.

            Note that `max_dist` is assumed to be a positive distance value in meters.
        """
        augmented_path = [path[0]]
        for next_point in path[0::]:
            curr_point = augmented_path[-1]

            dist_vector = next_point - curr_point  # vector pointing to next waypoint
            dist = dist_vector.get_length()  # same as `curr_point.distance_to(next_point)`
            ff.log_debug(f"{dist=}")

            while dist > max_dist:
                curr_point = curr_point + dist_vector * (max_dist / dist)
                augmented_path.append(curr_point)

                dist_vector = next_point - curr_point
                dist = dist_vector.get_length()

            augmented_path.append(next_point)

        return augmented_path


###############################################################################
## Internal functions #########################################################
###############################################################################


def _fly_path(
    client: airsim.MultirotorClient, path: List[Vector3r], velocity: float, timeout_sec: float
) -> None:
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

        # FIXME slow down instead of calling `.join()`... otherwise we're just
        #       using the high level controller in fact (and remove `.sleep()`)
        future.join()  # wait for AirSim's API to also recognize we've arrived
        time.sleep(0.5)  # FIXME stopping for some time minimizes overshooting

    client.moveToPositionAsync(*final_point, velocity, timeout_sec).join()


def _fly_path2(
    client: airsim.MultirotorClient, path: List[Vector3r], velocity: float, timeout_sec: float
) -> None:
    # NOTE testing changes to _fly_path (the same FIXMEs apply)

    assert len(path) >= 2

    first_point, *middle_points, final_point = [Vec3.from_Vector3r(waypoint) for waypoint in path]

    client.moveToPositionAsync(*first_point, velocity, timeout_sec).join()

    for next_pos, next_next_pos in zip(middle_points, middle_points[1:] + [final_point]):
        future = client.moveToPositionAsync(*next_pos, velocity, timeout_sec)
        curr_pos = Vec3.from_Vector3r(client.simGetVehiclePose().position)
        while not Vec3.all_close(curr_pos, next_pos, eps=CONFIRMATION_DISTANCE):
            curr_pos = Vec3.from_Vector3r(client.simGetVehiclePose().position)
            time.sleep(WAIT_TIME)

        # TODO Interpolate between no slowdown before the next waypoint and a
        #      "full stop" if the turning angle is too high:
        #
        #            No stop              ...       Full stop
        #
        #  curr       next     next_next        curr       next
        #    o -------> o -------> o              o -------> o
        #             \___/               ...            \__ |
        #            θ = 180                         θ = 90  |
        #                                                    v
        #                                                    o
        #                                                next_next

        curr_to_next = next_pos - curr_pos
        next_to_next_next = next_next_pos - next_pos

        theta = Vec3.angle_between(curr_to_next, next_to_next_next)
        ff.log_debug(f"θ = {theta}")

        if theta > 1.0:
            client.cancelLastTask()
            future = client.moveToPositionAsync(*next_pos, 1, timeout_sec)
            future.join()
            time.sleep(0.2)

    client.moveToPositionAsync(*final_point, velocity, timeout_sec).join()

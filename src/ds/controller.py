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
    from airsim.types import Vector3r, Pose


USE_AIRSIM_HL_CONTROLLER = True
CONFIRMATION_DISTANCE = 3.0
WAIT_TIME = 0.1


class Controller:
    """ A (currently) blocking flight controller which abstracts the flying method """

    @staticmethod
    def teleport(
        client: airsim.MultirotorClient, to: Pose, ignore_collison: bool = True, name: str = ""
    ) -> None:
        # HACK see https://github.com/Microsoft/AirSim/issues/1618#issuecomment-689152817
        client.simSetVehiclePose(pose=to, ignore_collison=ignore_collison)
        client.moveToPositionAsync(
            x=to.position.x_val,
            y=to.position.y_val,
            z=to.position.z_val,
            velocity=1,
            vehicle_name=name,
        )

    @staticmethod
    def fly_path(
        client: airsim.MultirotorClient, path: List[Vector3r], velocity: float = 2.0
    ) -> None:
        # TODO expose other arguments for more customization (e.g. `timeout_sec`)
        ff.log_warning(f"USE_AIRSIM_HL_CONTROLLER={USE_AIRSIM_HL_CONTROLLER}")

        if USE_AIRSIM_HL_CONTROLLER:
            client.moveOnPathAsync(path, velocity).join()

        else:
            for i, point in enumerate(path):
                ff.log_debug(f"Going to {ff.to_xyz_str(point)}")
                try:
                    next_point = path[i + 1]
                except:
                    next_point = None  # NOTE final point

                future = client.moveToPositionAsync(*ff.to_xyz_tuple(point), velocity)
                curr_pos = client.simGetVehiclePose().position

                # ref.: Issue#1677, Issue#2974
                if next_point is not None:
                    if i == 0:
                        future.join()
                    else:
                        while not Vec3.all_close(
                            Vec3.from_Vector3r(curr_pos),
                            Vec3.from_Vector3r(point),
                            eps=CONFIRMATION_DISTANCE,
                        ):
                            # "spin lock" untill we are close to the next point
                            curr_pos = client.simGetVehiclePose().position
                            time.sleep(WAIT_TIME)

                        future.join()
                        time.sleep(0.5)  # FIXME testing

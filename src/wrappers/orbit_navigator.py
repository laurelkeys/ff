import os
import sys
import math
import time

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim


class OrbitNavigator:
    """ Makes the drone fly in a circle.
        See https://github.com/microsoft/AirSim/blob/master/PythonClient/multirotor/orbit.py
    """

    def __init__(
        self, client, radius=2, altitude=10, speed=2, iterations=1, center=[1, 0], snapshots=None
    ):
        assert len(center) == 2, "Expecting '[x,y]' for the center direction vector"

        self.client = client
        self.z = None
        self.did_takeoff = False

        self.radius = radius
        self.altitude = altitude
        self.speed = speed
        self.iterations = max(1, iterations)

        self.snapshots = snapshots
        self.snapshot_index = 0
        self.next_snapshot = None
        self.snapshot_delta = None
        if self.snapshots is not None and self.snapshots > 0:
            self.snapshot_delta = 360 / self.snapshots

        # center is just a direction vector, so we normalize it to compute the actual cx, cy locations
        cx, cy = center
        length = math.sqrt((cx * cx) + (cy * cy))
        cx = (cx / length) * self.radius
        cy = (cy / length) * self.radius

        initial_state = self.client.getMultirotorState()
        self.home = initial_state.kinematics_estimated.position
        self.center = self.home
        self.center.x_val += cx
        self.center.y_val += cy
        ff.log_debug(f"{(self.home is self.center)=}")

        self._print = lambda *args, **kwargs: print(*args, **kwargs)

    def start(self, verbose=True):
        if not verbose:
            self._print = lambda *args, **kwargs: None

        self._print("arming the drone...")
        self.client.armDisarm(True)

        # AirSim uses NED coordinates so negative axis is up
        state = self.client.getMultirotorState()
        if not self.did_takeoff and state.landed_state == airsim.LandedState.Landed:
            self.did_takeoff = True
            self._print("taking off...")
            self.client.takeoffAsync(timeout_sec=10).join()
            start = self.client.getMultirotorState().kinematics_estimated.position
            z = -self.altitude + self.home.z_val
        else:
            start = state.kinematics_estimated.position
            z = start.z_val  # use current altitude then
            self._print(f"already flying so we will orbit at current altitude {z=}")

        self._print(f"climbing to position: {start.x_val}, {start.y_val}, {z}")
        self.client.moveToPositionAsync(start.x_val, start.y_val, z, self.speed).join()
        self.z = z

        self._print("ramping up to speed...")
        count = 0
        self.start_angle = None
        self.next_snapshot = None

        # ramp up time
        ramptime = self.radius / 10
        self.start_time = time.time()

        while count < self.iterations:
            if self.snapshot_index >= self.snapshots > 0:
                break
            # ramp up to full speed in smooth increments so we don't start too aggressively
            now = time.time()
            speed = self.speed
            diff = now - self.start_time
            if diff < ramptime:
                speed = self.speed * diff / ramptime
            elif ramptime > 0:
                self._print("reached full speed...")
                ramptime = 0

            lookahead_angle = speed / self.radius

            # compute current angle
            pos = self.client.getMultirotorState().kinematics_estimated.position
            dx = pos.x_val - self.center.x_val
            dy = pos.y_val - self.center.y_val
            # actual_radius = math.sqrt((dx * dx) + (dy * dy))
            angle_to_center = math.atan2(dy, dx)

            camera_heading = math.degrees(angle_to_center - math.pi)

            # compute lookahead
            lookahead_x = self.center.x_val + self.radius * math.cos(
                angle_to_center + lookahead_angle
            )
            lookahead_y = self.center.y_val + self.radius * math.sin(
                angle_to_center + lookahead_angle
            )

            vx = lookahead_x - pos.x_val
            vy = lookahead_y - pos.y_val

            if self._track_orbits(math.degrees(angle_to_center)):
                count += 1
                self._print("completed {} orbits".format(count))

            self.camera_heading = camera_heading
            self.client.moveByVelocityZAsync(
                vx,
                vy,
                z,
                duration=1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, camera_heading),
            )  # FIXME shouldn't we wait with .join()?

        self.client.moveToPositionAsync(start.x_val, start.y_val, z, velocity=2).join()

        if self.did_takeoff:
            # if we did the takeoff then also do the landing.
            if z < self.home.z_val:
                self._print("descending")
                self.client.moveToPositionAsync(
                    start.x_val, start.y_val, self.home.z_val - 5, velocity=2,
                ).join()

            self._print("landing...")
            self.client.landAsync(timeout_sec=40).join()

            self._print("disarming.")
            self.client.armDisarm(False)

    def _track_orbits(self, angle):
        # tracking the number of completed orbits is surprisingly tricky to get right
        # in order to handle random wobbles about the starting point,
        # so we watch for complete 1/2 orbits to avoid that problem
        if angle < 0:
            angle += 360

        if self.start_angle is None:
            self.start_angle = angle
            if self.snapshot_delta:
                self.next_snapshot = angle + self.snapshot_delta
            self.previous_angle = angle
            self.shifted = False
            self.previous_sign = None
            self.previous_diff = None
            self.quarter = False
            return False

        # now we just have to watch for a smooth crossing from negative diff to positive diff
        if self.previous_angle is None:
            self.previous_angle = angle
            return False

        # ignore the click over from 360 back to 0
        if self.previous_angle > 350 and angle < 10:
            if self.snapshot_delta and self.next_snapshot >= 360:
                self.next_snapshot -= 360
            return False

        diff = self.previous_angle - angle
        crossing = False
        self.previous_angle = angle

        if self.snapshot_delta and angle > self.next_snapshot:
            self._print(f"Taking snapshot at {angle=}")
            self._take_snapshot()
            self.next_snapshot += self.snapshot_delta

        diff = abs(angle - self.start_angle)
        if diff > 45:
            self.quarter = True

        if self.quarter and self.previous_diff is not None and diff != self.previous_diff:
            # watch direction this diff is moving
            # if it switches from shrinking to growing then we passed the starting point
            direction = -1 if self.previous_diff < diff else 1
            if self.previous_sign is None:
                self.previous_sign = direction
            elif self.previous_sign > 0 and direction < 0:
                if diff < 45:
                    self.quarter = False
                    if self.snapshots <= self.snapshot_index + 1:
                        crossing = True
            self.previous_sign = direction
        self.previous_diff = diff

        return crossing

    def _take_snapshot(self):
        # first hold our current position so drone doesn't try and keep flying while we take the picture.
        pos = self.client.getMultirotorState().kinematics_estimated.position
        self.client.moveToPositionAsync(
            pos.x_val,
            pos.y_val,
            self.z,
            velocity=0.5,
            timeout_sec=10,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(False, self.camera_heading),
        ).join()
        responses = self.client.simGetImages(
            [airsim.ImageRequest(ff.CameraName.front_right, airsim.ImageType.Scene)]
        )  # scene vision image in png format
        response = responses[0]
        filename = f"photo_{self.snapshot_index}"
        self.snapshot_index += 1
        airsim.write_file(os.path.normpath(filename + ".png"), response.image_data_uint8)
        self._print(f"Saved snapshot: {filename}")
        self.start_time = time.time()  # cause smooth ramp up to happen again after photo is taken

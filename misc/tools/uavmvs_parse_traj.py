from enum import Enum
from typing import Optional, NamedTuple

import numpy as np

###############################################################################
###############################################################################


class TrajectoryCameraKind(Enum):
    Traj = 0
    Csv = 1
    Utj = 2

    # NOTE while the units and coordinate system for .traj
    # and .csv files are the same, in .utj files the x sign
    # is inverted, and x, y, z are in centimeters.


TRAJ = TrajectoryCameraKind.Traj
CSV = TrajectoryCameraKind.Csv
UTJ = TrajectoryCameraKind.Utj


CSV_HEADER = "x,y,z,qw,qx,qy,qz,key"
INTO_UTJ = np.array([-100, 100, 100])  # m -> cm and invert x
FROM_UTJ = np.array([-0.01, 0.01, 0.01])  # cm -> m and invert x


class TrajectoryCamera(NamedTuple):
    """ Represents a camera pose entry in a .traj, .csv or .utj file. """

    position: np.ndarray
    rotation: np.ndarray
    kind: TrajectoryCameraKind
    focal_length: Optional[float] = None  # .traj
    spline_interpolated: bool = False  # .csv
    id: Optional[int] = None  # .utj

    # NOTE `rotation` is:
    # - (3, 3) rotation matrix, if kind == TRAJ
    # - (4,) XYZW quaternion, if kind == CSV
    # - (3,) pitch,roll,yaw, if kind == UTJ

    def _rotation_into(self, other_kind):
        if self.kind == other_kind:
            return self.rotation
        # def matrix_to_quat(): raise NotImplementedError
        # def matrix_to_euler(): raise NotImplementedError
        # def quat_to_matrix(): raise NotImplementedError
        # def quat_to_euler(): raise NotImplementedError
        # def euler_to_matrix(): raise NotImplementedError
        # def euler_to_quat(): raise NotImplementedError
        # return {
        #     (TRAJ, CSV): matrix_to_quat,
        #     (TRAJ, UTJ): matrix_to_euler,
        #     (CSV, TRAJ): quat_to_matrix,
        #     (CSV, UTJ): quat_to_euler,
        #     (UTJ, TRAJ): euler_to_matrix,
        #     (UTJ, CSV): euler_to_quat,
        # }[(self.kind, other_kind)]()
        raise NotImplementedError

    def into(self, other_kind):
        if self.kind in [TRAJ, CSV] and other_kind in [TRAJ, CSV]:
            return self
        elif self.kind != UTJ:
            return TrajectoryCamera(
                position=self.position * FROM_UTJ,
                rotation=self._rotation_into(other_kind),
                kind=other_kind,
            )
        elif other_kind != UTJ:
            return TrajectoryCamera(
                position=self.position * INTO_UTJ,
                rotation=self._rotation_into(other_kind),
                kind=other_kind,
            )
        return self


###############################################################################
###############################################################################


def parse_uavmvs_traj(filepath):
    trajectory = []
    with open(filepath, "r") as f:
        n_of_cameras = int(f.readline())
        for _ in range(n_of_cameras):
            position = np.array([float(_) for _ in f.readline().split()])
            rotation = np.array(
                [
                    [float(_) for _ in f.readline().split()],
                    [float(_) for _ in f.readline().split()],
                    [float(_) for _ in f.readline().split()],
                ]
            )
            focal_length = float(f.readline())
            assert position.shape == (3,)
            assert rotation.shape == (3, 3)
            trajectory.append(
                TrajectoryCamera(
                    position,
                    rotation,
                    kind=TRAJ,
                    focal_length=focal_length,
                )
            )
    return trajectory


def parse_uavmvs_csv(filepath):
    trajectory = []
    with open(filepath, "r") as f:
        assert f.readline().rstrip().lower() == CSV_HEADER
        for line in f.readlines():
            x, y, z, qw, qx, qy, qz, key = line.split(",")
            position = np.array([float(_) for _ in [x, y, z]])
            rotation = np.array([float(_) for _ in [qx, qy, qz, qw]])
            trajectory.append(
                TrajectoryCamera(
                    position,
                    rotation,
                    kind=CSV,
                    spline_interpolated=(int(key) == 0),
                )
            )
    return trajectory


def parse_uavmvs_utj(filepath):
    trajectory = []
    with open(filepath, "r") as f:
        for line in f.readlines():
            id, x, y, z, pitch, roll, yaw = line.split(",")
            # NOTE angles are in euler degrees (standard for UE4),
            # x, y and z are in centimeters and x sign is inverted.
            position = np.array([float(_) for _ in [x, y, z]])
            rotation = np.array([float(_) for _ in [pitch, roll, yaw]])
            trajectory.append(
                TrajectoryCamera(
                    position,
                    rotation,
                    kind=UTJ,
                    id=int(id),
                )
            )
    return trajectory


###############################################################################
###############################################################################

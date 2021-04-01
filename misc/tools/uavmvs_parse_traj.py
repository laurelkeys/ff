from enum import Enum
from typing import Dict, List, Tuple, Callable, Optional, NamedTuple, cast

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
    # - (4,) WXYZ quaternion, if kind == CSV
    # - (3,) pitch,roll,yaw, if kind == UTJ

    def _rotation_into(self, other_kind: TrajectoryCameraKind):
        if self.kind == other_kind:
            return self.rotation

        # ref.: uavmvs/apps/trajectory_tools/interpolate.cpp
        def rot_to_quat(R):
            assert R.shape == (3, 3)
            v0 = 1.0 + R[0, 0] + R[1, 1] + R[2, 2]
            v1 = 1.0 + R[0, 0] - R[1, 1] - R[2, 2]
            v2 = 1.0 - R[0, 0] + R[1, 1] - R[2, 2]
            v3 = 1.0 - R[0, 0] - R[1, 1] + R[2, 2]
            if v0 >= v1 and v0 >= v2 and v0 >= v3:
                tmp = 0.5 * np.sqrt(v0)
                return np.array(
                    [
                        tmp,
                        (R[2, 1] - R[1, 2]) / (4 * tmp),
                        (R[0, 2] - R[2, 0]) / (4 * tmp),
                        (R[1, 0] - R[0, 1]) / (4 * tmp),
                    ]
                )
            elif v1 >= v0 and v1 >= v2 and v1 >= v3:
                tmp = 0.5 * np.sqrt(v1)
                return np.array(
                    [
                        (R[2, 1] - R[1, 2]) / (4 * tmp),
                        tmp,
                        (R[0, 1] + R[1, 0]) / (4 * tmp),
                        (R[0, 2] + R[2, 0]) / (4 * tmp),
                    ]
                )
            elif v2 >= v0 and v2 >= v1 and v2 >= v3:
                tmp = 0.5 * np.sqrt(v2)
                return np.array(
                    [
                        (R[0, 2] - R[2, 0]) / (4 * tmp),
                        (R[0, 1] + R[1, 0]) / (4 * tmp),
                        tmp,
                        (R[1, 2] + R[2, 1]) / (4 * tmp),
                    ]
                )
            else:
                assert v3 >= v0 and v3 >= v1 and v3 >= v2
                tmp = 0.5 * np.sqrt(v3)
                return np.array(
                    [
                        (R[1, 0] - R[0, 1]) / (4 * tmp),
                        (R[0, 2] + R[2, 0]) / (4 * tmp),
                        (R[1, 2] + R[2, 1]) / (4 * tmp),
                        tmp,
                    ]
                )

        def rot_to_euler(R):
            assert R.shape == (3, 3)
            return quat_to_euler(rot_to_quat(R))

        # ref.: https://github.com/nmoehrle/mve/blob/master/libs/math/quaternion.h
        def quat_to_rot(q):
            assert q.shape == (4,)
            xxzz = q[1] ** 2 - q[3] ** 2
            rryy = q[0] ** 2 - q[2] ** 2
            yyrrxxzz = q[2] ** 2 + q[0] ** 2 - q[1] ** 2 - q[3] ** 2

            xr2 = q[1] * q[0] * 2
            xy2 = q[1] * q[2] * 2
            xz2 = q[1] * q[3] * 2
            yr2 = q[2] * q[0] * 2
            yz2 = q[2] * q[3] * 2
            zr2 = q[3] * q[0] * 2

            return np.array(
                [
                    [(xxzz + rryy), (xy2 - zr2), ( xz2 + yr2 )],
                    [( xy2 + zr2 ), (yyrrxxzz ), ( yz2 - xr2 )],
                    [( xz2 - yr2 ), (yz2 + xr2), (rryy - xxzz)],
                ]
            )

        # ref.: uavmvs/apps/trajectory_tools/interpolate.cpp
        def quat_to_euler(q):
            assert q.shape == (4,)
            phi = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            theta = np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0))
            psi = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            return np.array([phi, theta, psi])  # roll, pitch, yaw  (in radians)

        def euler_to_rot(e):
            assert e.shape == (3,)
            raise NotImplementedError

        def euler_to_quat(e):
            assert e.shape == (3,)
            raise NotImplementedError

        return cast(
            Dict[
                Tuple[TrajectoryCameraKind, TrajectoryCameraKind],
                Callable[[np.ndarray], np.ndarray],
            ],
            {
                (TRAJ, CSV): rot_to_quat,
                (TRAJ, UTJ): rot_to_euler,
                (CSV, TRAJ): quat_to_rot,
                (CSV, UTJ): quat_to_euler,
                # TODO
                (UTJ, TRAJ): euler_to_rot,
                (UTJ, CSV): euler_to_quat,
            },
        )[(self.kind, other_kind)](self.rotation)

    def into(self, other_kind):
        if self.kind == other_kind:
            return self
        elif other_kind == UTJ:
            return TrajectoryCamera(
                position=self.position * FROM_UTJ,
                rotation=self._rotation_into(other_kind),
                kind=other_kind,
            )
        elif self.kind == UTJ:
            return TrajectoryCamera(
                position=self.position * INTO_UTJ,
                rotation=self._rotation_into(other_kind),
                kind=other_kind,
            )
        else:
            assert self.kind in [TRAJ, CSV] and other_kind in [TRAJ, CSV]
            return TrajectoryCamera(
                position=self.position,
                rotation=self._rotation_into(other_kind),
                kind=other_kind,
            )


###############################################################################
###############################################################################


def parse_uavmvs_traj(filepath: str) -> List[TrajectoryCamera]:
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


def parse_uavmvs_csv(filepath: str) -> List[TrajectoryCamera]:
    trajectory = []
    with open(filepath, "r") as f:
        assert f.readline().rstrip().lower() == CSV_HEADER
        for line in f.readlines():
            x, y, z, qw, qx, qy, qz, key = line.split(",")
            position = np.array([float(_) for _ in [x, y, z]])
            rotation = np.array([float(_) for _ in [qw, qx, qy, qz]])
            trajectory.append(
                TrajectoryCamera(
                    position,
                    rotation,
                    kind=CSV,
                    spline_interpolated=(int(key) == 0),
                )
            )
    return trajectory


def parse_uavmvs_utj(filepath: str) -> List[TrajectoryCamera]:
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


parse_uavmvs: Dict[str, Callable[[str], List[TrajectoryCamera]]] = {
    ".traj": parse_uavmvs_traj,
    ".csv": parse_uavmvs_csv,
    ".utj": parse_uavmvs_utj,
}


###############################################################################
###############################################################################


def convert_uavmvs_to_airsim_position(camera_position, translation=None, scaling=None):
    import airsim

    x, y, z = map(float, camera_position)
    y = -y
    z = -z
    position = airsim.Vector3r(x, y, z)

    if translation is not None:
        position += airsim.Vector3r(*map(float, translation))
    if scaling is not None:
        position *= float(scaling)

    del airsim
    return position


def convert_uavmvs_to_airsim_pose(camera: TrajectoryCamera, translation=None, scaling=None):
    import airsim

    assert camera.kind in [TRAJ, CSV]

    position = convert_uavmvs_to_airsim_position(camera.position, translation, scaling)

    qw, qx, qy, qz = map(float, camera._rotation_into(CSV))
    qy = -qy
    qz = -qz
    orientation = airsim.Quaternionr(qx, -qy, -qz, qw)

    length = orientation.get_length()
    assert 0.99 <= length <= 1.01, orientation
    # orientation /= length

    # NOTE ignore translation and scaling for orientation (we'd only care for rotation)

    pose = airsim.Pose(position, orientation)

    del airsim
    return pose


###############################################################################
###############################################################################

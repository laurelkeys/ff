import os
import argparse

from enum import Enum
from typing import Optional, NamedTuple

import numpy as np
import open3d as o3d


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
                kind=other_kind
            )
        elif other_kind != UTJ:
            return TrajectoryCamera(
                position=self.position * INTO_UTJ,
                rotation=self._rotation_into(other_kind),
                kind=other_kind
            )
        return self


###############################################################################
###############################################################################


def parse_uavmvs_traj(filepath):
    trajectory = []
    with open(filepath, "r") as f:
        n_of_cameras = int(f.readline())
        for _ in range(n_of_cameras):
            position = np.array([float(_) for _ in f.readline()])
            rotation = np.array(
                [
                    [float(_) for _ in f.readline()],
                    [float(_) for _ in f.readline()],
                    [float(_) for _ in f.readline()],
                ]
            )
            focal_length = float(f.readline())
            assert position.shape == (3,)
            assert rotation.shape == (3, 3)
            trajectory.append(
                TrajectoryCamera(
                    position, rotation, kind=TRAJ, focal_length=focal_length
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
                    position, rotation, kind=CSV, spline_interpolated=(int(key) == 0),
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
                    position, rotation, kind=UTJ, id=int(id),
                )
            )
    return trajectory


###############################################################################
###############################################################################


RGB = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
CMY = [[0, 1, 1], [1, 0, 1], [1, 1, 0]]
BLACK = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
WHITE = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]


def coordinate_axes_line_set(axes_origins, size=1.0, colors=None):
    points, lines = [], []

    # NOTE Open3D can't really handle rendering more than 50 meshes, so we merge
    #      multiple coordinate axes into a single LineSet for it to run smoothly.

    for origin in axes_origins:
        o = len(points)
        lines.extend([[o, o + 1], [o, o + 2], [o, o + 3]])  # o-x o-y o-z
        points.extend(
            [
                np.asarray(origin, dtype=np.float32) + size * np.asarray(_, dtype=np.float32)
                for _ in [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]  # o x y z
            ]
        )

    if colors is None:
        colors = [RGB for _ in range(len(axes_origins))]

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(np.asarray(colors).reshape((-1, 3)))

    return line_set  # TODO rotate this based on the camera's rotation


###############################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    parse_uavmvs = {
        ".traj": parse_uavmvs_traj,
        ".csv": parse_uavmvs_csv,
        ".utj": parse_uavmvs_utj,
    }

    _, ext = os.path.splitext(args.trajectory)
    assert os.path.isfile(args.trajectory) and ext in parse_uavmvs.keys()

    trajectory = parse_uavmvs[ext](args.trajectory)

    camera_positions = [camera.position for camera in trajectory]

    if args.export is not None:
        pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(camera_positions))
        o3d.io.write_point_cloud(filename=args.export, pointcloud=pcd, print_progress=args.verbose)

    geometry_list = [
        coordinate_axes_line_set(
            size=0.5,
            axes_origins=camera_positions,
            colors=[RGB if not camera.interpolated else BLACK for camera in trajectory],
        )
    ]

    if args.mesh:
        mesh = o3d.io.read_triangle_mesh(args.mesh)
        geometry_list.append(mesh)
        if args.verbose:
            print(mesh)
            # print(np.asarray(mesh.vertices))
            # print(np.asarray(mesh.triangles))
            print()

    if args.cloud:
        cloud = o3d.io.read_point_cloud(args.cloud)
        geometry_list.append(cloud)
        if args.verbose:
            print(cloud)
            # print(np.asarray(cloud.points))
            print()

    o3d.visualization.draw_geometries(geometry_list)


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize a trajectory generated by uavmvs with Open3D."
    )

    parser.add_argument("trajectory", type=str, help="Path to the input .TRAJ, .CSV or .UTJ file")
    parser.add_argument("--export", type=str, help="Path to export the trajectory as .PLY to")

    parser.add_argument("--mesh", type=str, help="Path to a .PLY mesh file")
    parser.add_argument("--cloud", type=str, help="Path to a .PLY point cloud file")

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    main(args=parser.parse_args())


###############################################################################
###############################################################################

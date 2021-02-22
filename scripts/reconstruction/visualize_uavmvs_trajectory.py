import os
import argparse

from typing import Optional, NamedTuple

import numpy as np
import open3d as o3d


###############################################################################
###############################################################################


def _parse_float_list(line):
    return [float(_) for _ in line.split()]


def _eat_position(f):
    position = np.array(_parse_float_list(f.readline()))
    assert position.shape == (3,)
    return position


def _eat_rotation(f):
    rotation = np.array(
        [
            _parse_float_list(f.readline()),
            _parse_float_list(f.readline()),
            _parse_float_list(f.readline()),
        ]
    )
    assert rotation.shape == (3, 3)
    return rotation


class TrajectoryCamera(NamedTuple):
    """ Represents a camera pose entry in a .traj (or .csv) file. """

    position: np.ndarray
    rotation: np.ndarray  # NOTE (3, 3) rotation matrix or (4,) xyzw quaternion
    interpolated: bool = False  # NOTE spline interpolated position if True
    focal_length: Optional[float] = None


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

    _, ext = os.path.splitext(args.trajectory)
    assert os.path.isfile(args.trajectory) and ext in [".traj", ".csv"]

    with open(args.trajectory, "r") as f:
        trajectory = []
        if ext == ".traj":
            n_of_cameras = int(f.readline())
            for _ in range(n_of_cameras):
                position = _eat_position(f)
                rotation = _eat_rotation(f)
                trajectory.append(
                    TrajectoryCamera(position, rotation, focal_length=float(f.readline()))
                )
            assert len(trajectory) == n_of_cameras
        else:
            f.readline()  # skip header
            for line in f.readlines():
                x, y, z, qw, qx, qy, qz, key = line.split(",")
                trajectory.append(
                    TrajectoryCamera(
                        position=np.array([x, y, z]),
                        rotation=np.array([qx, qy, qz, qw]),
                        interpolated=(int(key) == 0),
                    )
                )

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

    parser.add_argument("trajectory", type=str, help="Path to the input .TRAJ (or .CSV) file")
    parser.add_argument("--export", type=str, help="Path to export the trajectory as .PLY to")

    parser.add_argument("--mesh", type=str, help="Path to a .PLY mesh file")
    parser.add_argument("--cloud", type=str, help="Path to a .PLY point cloud file")

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    main(args=parser.parse_args())


###############################################################################
###############################################################################

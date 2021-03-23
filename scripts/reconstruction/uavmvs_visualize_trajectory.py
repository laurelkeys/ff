import os
import argparse

from glob import iglob
from typing import List

import numpy as np
import open3d as o3d

try:
    from include_in_path import include, FF_PROJECT_ROOT

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    from uavmvs_parse_traj import parse_uavmvs, TrajectoryCamera
except:
    raise


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
                     # http://www.open3d.org/docs/latest/tutorial/Basic/transformation.html


###############################################################################
###############################################################################


def draw_trajectory(args: argparse.Namespace, trajectory: List[TrajectoryCamera]) -> None:
    camera_positions = [camera.position for camera in trajectory]

    if args.export is not None:
        pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(camera_positions))
        o3d.io.write_point_cloud(filename=args.export, pointcloud=pcd, print_progress=args.verbose)

    geometry_list = [
        coordinate_axes_line_set(
            size=args.size,
            axes_origins=camera_positions,
            colors=[RGB if not camera.spline_interpolated else BLACK for camera in trajectory],
        )
    ]

    if args.trace:
        points = [np.asarray(p, dtype=np.float32) for p in camera_positions]
        lines = [[i - 1, i] for i, _ in enumerate(camera_positions)]
        geometry_list.append(
            o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
        )

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


def main(args: argparse.Namespace) -> None:
    # if args.verbose:
    #     o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    if len(args.trajectories) == 1 and os.path.isdir(args.trajectories[0]):
        args.trajectories, trajectories_dir = [], args.trajectories[0]
        args.trajectories.extend(sorted(iglob(os.path.join(trajectories_dir, "*.traj"))))
        args.trajectories.extend(sorted(iglob(os.path.join(trajectories_dir, "*.csv"))))
        args.trajectories.extend(sorted(iglob(os.path.join(trajectories_dir, "*.utj"))))

    for trajectory_path in args.trajectories:
        root, ext = os.path.splitext(trajectory_path)
        assert os.path.isfile(trajectory_path), trajectory_path
        assert ext in parse_uavmvs.keys(), trajectory_path

        trajectory = parse_uavmvs[ext](trajectory_path)
        if args.verbose:
            print(f"({len(trajectory)} cameras in trajectory '{os.path.basename(root)}{ext}')")

        draw_trajectory(args, trajectory)


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize a trajectory file generated by uavmvs with Open3D."
    )

    parser.add_argument(
        "trajectories", type=str, nargs="+", help="Path to the input .TRAJ, .CSV or .UTJ file(s)"
    )
    parser.add_argument("--export", type=str, help="Path to export the trajectory as .PLY to")

    parser.add_argument("--mesh", type=str, help="Path to a .PLY mesh file")
    parser.add_argument("--cloud", type=str, help="Path to a .PLY point cloud file")

    parser.add_argument("--size", type=float, default=2.0, help="Coordinate axis size")
    parser.add_argument("--trace", action="store_true", help="Plot trajectory lines")

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    main(args=parser.parse_args())


###############################################################################
###############################################################################
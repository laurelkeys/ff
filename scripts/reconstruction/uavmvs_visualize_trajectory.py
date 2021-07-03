import os
import argparse

from glob import iglob
from typing import List

import numpy as np
import open3d as o3d

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    import uavmvs_parse_traj as uavmvs
except:
    raise


###############################################################################
###############################################################################


RGB = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
CMY = [[0, 1, 1], [1, 0, 1], [1, 1, 0]]
BLACK = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
WHITE = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]


def coordinate_axes_line_set(
    axes_origins, axes_rotations=None, size=1.0, colors=None, frustum=False, row_major=True
):
    points, lines = [], []

    # NOTE Open3D can't really handle rendering more than 50 meshes, so we merge
    #      multiple coordinate axes into a single LineSet for it to run smoothly.

    def update_points_and_lines(origin, x_axis, y_axis, z_axis):
        scale_and_translate = lambda _: np.asarray(_, dtype=np.float32) * size + np.asarray(origin)

        o = len(points)
        lines.extend([[o, o + 1], [o, o + 2], [o, o + 3]])  # o-x o-y o-z
        points.extend([scale_and_translate(_) for _ in [[0, 0, 0], x_axis, y_axis, z_axis]])

        if frustum:
            br, tr, tl, bl = o + 4, o + 5, o + 6, o + 7
            lines.extend([[o, br], [o, tr], [o, tl], [o, bl]])  # o-br o-tr o-tl o-bl
            lines.extend([[br, tr], [tr, tl], [tl, bl], [bl, br]])  # br-tr tr-tl tl-bl bl-br
            # NOTE scale z_axis by some small constant factor so that the frustum looks nicer
            points.append(scale_and_translate(2 * z_axis + x_axis - y_axis))  # bottom right (br)
            points.append(scale_and_translate(2 * z_axis + x_axis + y_axis))  # top right    (tr)
            points.append(scale_and_translate(2 * z_axis - x_axis + y_axis))  # top left     (tl)
            points.append(scale_and_translate(2 * z_axis - x_axis - y_axis))  # bottom left  (bl)

    if axes_rotations is not None:
        assert len(axes_origins) == len(axes_rotations)
        for origin, rotation in zip(axes_origins, axes_rotations):
            if row_major:
                update_points_and_lines(origin, rotation[:, 0], rotation[:, 1], rotation[:, 2])
            else:
                update_points_and_lines(origin, rotation[0, :], rotation[1, :], rotation[2, :])
    else:
        for origin in axes_origins:
            x_axis, y_axis, z_axis = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
            update_points_and_lines(origin, x_axis, y_axis, z_axis)

    if colors is None:
        colors = [RGB if not frustum else RGB + ([[0, 0, 0]] * 8) for _ in range(len(axes_origins))]

    colors = np.asarray(colors).reshape((-1, 3))
    colors_shape = colors.shape  # (num_lines, 3)
    lines_shape = np.asarray(lines).shape  # (num_lines, 2)
    assert colors_shape[0] == lines_shape[0]

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(np.asarray(colors).reshape((-1, 3)))

    return line_set


###############################################################################
###############################################################################


def draw_trajectory(args: argparse.Namespace, trajectory: List[uavmvs.TrajectoryCamera]) -> None:
    skipped_any = False
    camera_positions = []
    camera_rotation_matrices = []
    for camera in trajectory:
        if not camera.spline_interpolated:
            # NOTE *.traj files represent rotations as 3x3 matrices
            camera = camera.into(uavmvs.TrajectoryCameraKind.Traj)
            camera_positions.append(camera.position)
            camera_rotation_matrices.append(camera.rotation)
        else:
            # NOTE this skips some TrajectoryCameraKind.Csv cameras
            skipped_any = True

    if skipped_any:
        print("Note: some cameras have been skipped (spline interpolated positions)")

    if args.export is not None:
        pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(camera_positions))
        o3d.io.write_point_cloud(filename=args.export, pointcloud=pcd, print_progress=args.verbose)

    draw_list = [
        coordinate_axes_line_set(
            axes_origins=camera_positions,
            axes_rotations=camera_rotation_matrices,
            size=args.size,
            # colors=[RGB if not _.spline_interpolated else BLACK for _ in trajectory],
            frustum=True,
            # NOTE uavmvs stores matrices in column major notation, so we would need to use its rows
            # but, since we transpose it in parse_uavmvs_traj(), it is ok to assume row major order!
            row_major=True,
        )
    ]

    if args.trace:
        points = [np.asarray(p, dtype=np.float32) for p in camera_positions]
        lines = [[i - 1, i] for i, _ in enumerate(camera_positions)]
        draw_list.append(
            o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
        )

    if args.mesh:
        mesh = o3d.io.read_triangle_mesh(args.mesh)
        draw_list.append(mesh)
        if args.verbose:
            print(mesh)
            # print(np.asarray(mesh.vertices))
            # print(np.asarray(mesh.triangles))
            print()

    if args.cloud:
        cloud = o3d.io.read_point_cloud(args.cloud)
        draw_list.append(cloud)
        if args.verbose:
            print(cloud)
            # print(np.asarray(cloud.points))
            print()

    o3d.visualization.draw_geometries(draw_list)


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
        assert ext in uavmvs.parse_uavmvs.keys(), trajectory_path

        trajectory = uavmvs.parse_uavmvs[ext](trajectory_path)[:-5]  # XXX :SkipLastFive:
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

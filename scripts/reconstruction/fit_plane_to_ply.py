import os
import argparse

import numpy as np
import open3d as o3d

from plane_from_points import plane_from_points


def fit_plane_to_ply(
    ply_path: str, distance_threshold: float, ransac_n: int, num_iterations: int
) -> None:
    assert os.path.isfile(ply_path), f"Invalid file path: '{ply_path}'"

    pcd = o3d.io.read_point_cloud(ply_path)
    # np_pcd = np.asarray(pcd.points)
    center = pcd.get_center()
    aabb = pcd.get_axis_aligned_bounding_box()
    print(f"Center: {center}\n{aabb}")

    plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    a, b, c, d = plane_model
    print(f"Open3D's plane equation: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

    print(f"{len(inliers)} inliers (out of {len(pcd.points)} points)")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    a, b, c, d = plane_from_points(np.asarray(inlier_cloud.points))
    print(f"Computed plane equation: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fit a plane to a given point cloud using RANSAC")
    parser.add_argument("ply_path", type=str, help="Path to the PLY file")
    parser.add_argument("distance", type=float, help="Maximum distance from a point to the plane")
    parser.add_argument("--ransac_n", type=int, default=6, help="Initial points in each iteration")
    parser.add_argument("--iter", type=int, default=1000, help="Number of iterations")
    args = parser.parse_args()
    fit_plane_to_ply(args.ply_path, args.distance, args.ransac_n, args.iter)

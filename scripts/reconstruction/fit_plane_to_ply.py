import os
import argparse
from typing import Optional

import open3d as o3d

import numpy as np


def plane_from_points(points: np.ndarray) -> Optional[np.ndarray]:
    """ Fits a plane to a collection of points. Returns None if the points do not span a plane.

        Reference: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    """
    assert points.ndim == 2 and points.shape[1] == 3
    n, _ = points.shape
    if n < 3:
        return None

    centroid = np.sum(points, axis=0) / n
    xs, ys, zs = np.rollaxis(points - centroid, axis=1)

    # Calculate full 3x3 covariance matrix, excluding symmetries
    xx = np.sum(xs * xs, axis=0) / n
    xy = np.sum(xs * ys, axis=0) / n
    xz = np.sum(xs * zs, axis=0) / n
    yy = np.sum(ys * ys, axis=0) / n
    yz = np.sum(ys * zs, axis=0) / n
    zz = np.sum(zs * zs, axis=0) / n

    det_x = yy * zz - yz * yz
    det_y = xx * zz - xz * xz
    det_z = xx * yy - xy * xy

    weighted_dir = np.zeros(3)

    def add_to_weighted_dir(axis_dir, weight):
        nonlocal weighted_dir
        if np.dot(weighted_dir, axis_dir) < 0:
            weighted_dir -= axis_dir * weight
        else:
            weighted_dir += axis_dir * weight

    add_to_weighted_dir(np.array([det_x, xz * yz - xy * zz, xy * yz - xz * yy]), det_x * det_x)
    add_to_weighted_dir(np.array([xz * yz - xy * zz, det_y, xy * xz - yz * xx]), det_y * det_y)
    add_to_weighted_dir(np.array([xy * yz - xz * yy, xy * xz - yz * xx, det_z]), det_z * det_z)

    norm = np.linalg.norm(weighted_dir)
    if norm == 0:
        return None

    def plane_from_point_and_normal(point, normal):
        d = -np.dot(normal, point)
        a, b, c = normal
        return np.array([a, b, c, d])

    return plane_from_point_and_normal(centroid, weighted_dir / norm)


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

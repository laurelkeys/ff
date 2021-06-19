# TODO move this to ie/

import copy

from typing import Optional

import numpy as np
import open3d as o3d


###############################################################################
###############################################################################


try:
    registration = o3d.registration
except AttributeError:  # NOTE changed from 0.10.0 to 0.11.0
    registration = o3d.pipelines.registration


def RANSACConvergenceCriteria(max_iteration=None, max_validation=1000, confidence=0.999):
    try:
        return registration.RANSACConvergenceCriteria(
            max_iteration=1000 if max_iteration is None else max_iteration,
            max_validation=max_validation,
        )
    except TypeError:  # NOTE changed from 0.11.1 to 0.12.0
        return registration.RANSACConvergenceCriteria(
            max_iteration=100000 if max_iteration is None else max_iteration,
            confidence=confidence,
        )


###############################################################################
###############################################################################


v2d = o3d.utility.Vector2dVector  # (n, 2) float64 numpy array -> Open3D format
v3d = o3d.utility.Vector3dVector  # (n, 3) float64 numpy array -> Open3D format

v2i = o3d.utility.Vector2iVector  # (n, 2) int32 numpy array -> Open3D format
v3i = o3d.utility.Vector3iVector  # (n, 3) int32 numpy array -> Open3D format
v4i = o3d.utility.Vector4iVector  # (n, 4) int32 numpy array -> Open3D format


###############################################################################
###############################################################################


def split_points_by_index(pcd, index_list):
    return pcd.select_by_index(index_list), pcd.select_by_index(index_list, invert=True)


###############################################################################
###############################################################################


def plane_from_point_and_normal(point: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Returns `[a, b, c, d]`, such that `normal == [a, b, c]` and `normal.dot(point) + d == 0`."""
    a, b, c = normal
    d = -np.dot(normal, point)
    return np.array([a, b, c, d])


def plane_from_points(points: np.ndarray, use_old_method: bool = False) -> Optional[np.ndarray]:
    """Fits a plane to a collection of points. Returns None if the points do not span a plane."""
    assert points.ndim == 2 and points.shape[1] == 3
    n, _ = points.shape
    if n < 3:
        return None

    centroid = np.sum(points, axis=0) / n
    xs, ys, zs = np.rollaxis(points - centroid, axis=1)

    # Calculate full 3x3 covariance matrix, excluding symmetries
    # NOTE it is not necessary to divide by n as we normalize it
    xx = np.sum(xs * xs, axis=0)
    xy = np.sum(xs * ys, axis=0)
    xz = np.sum(xs * zs, axis=0)
    yy = np.sum(ys * ys, axis=0)
    yz = np.sum(ys * zs, axis=0)
    zz = np.sum(zs * zs, axis=0)

    det_x = yy * zz - yz * yz
    det_y = xx * zz - xz * xz
    det_z = xx * yy - xy * xy

    if use_old_method:
        # https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
        normal = (
            np.array([det_x, xz * yz - xy * zz, xy * yz - xz * yy])
            if det_x > det_y and det_x > det_z
            else np.array([xz * yz - xy * zz, det_y, xy * xz - yz * xx])
            if det_y > det_z
            else np.array([xy * yz - xz * yy, xy * xz - yz * xx, det_z])
        )
    else:
        # https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
        x_axis_dir = np.array([det_x, xz * yz - xy * zz, xy * yz - xz * yy])
        y_axis_dir = np.array([xz * yz - xy * zz, det_y, xy * xz - yz * xx])
        z_axis_dir = np.array([xy * yz - xz * yy, xy * xz - yz * xx, det_z])

        def sign(is_negative):
            return -1 if is_negative else +1

        weighted_dir = +(det_x * det_x) * x_axis_dir
        weighted_dir += (det_y * det_y) * y_axis_dir * sign(np.dot(weighted_dir, y_axis_dir) < 0)
        weighted_dir += (det_z * det_z) * z_axis_dir * sign(np.dot(weighted_dir, z_axis_dir) < 0)

        normal = weighted_dir

    norm = np.linalg.norm(normal)
    if norm == 0:
        return None

    return plane_from_point_and_normal(centroid, normal / norm)


def points_above_planes_mask(planes: np.ndarray, points: np.ndarray) -> np.ndarray:
    if planes.ndim == 1:
        planes = planes.reshape((1, -1))

    n, _ = points.shape
    assert points.shape[1] == 3  # [x, y, z]
    assert planes.shape[1] == 4  # [a, b, c, d]
    points = np.hstack((points, np.ones((n, 1))))  # [x, y, z] -> [x, y, z, 1]

    above_planes_mask = np.logical_and.reduce(
        list(np.einsum("j,ij->i", plane, points) > 0 for plane in planes)
    )

    return above_planes_mask
    # return points[above_planes_mask][:, :3]  # [x, y, z] -> [x, y, z, 1]


###############################################################################
###############################################################################


def create_unit_sphere_point_cloud(n, point_fn=None, color_fn=None, eps=0.0001):
    if point_fn is None:
        point_fn = lambda point: point
    if color_fn is None:
        color_fn = lambda point: [0, 0, 0]

    points = []
    colors = []
    for _ in range(n):
        while True:
            point = np.random.rand(3) * 2.0 - 1.0
            norm = np.linalg.norm(point)
            if eps < norm <= 1.0:
                break
        point /= norm
        points.append(point_fn(point))
        colors.append(color_fn(point))

    pcd = o3d.geometry.PointCloud()
    pcd.points = v3d(points)
    pcd.colors = v3d(colors)

    return pcd


def create_plane_triangle_mesh(point, normal, eps=0.01):
    dummy = [1, 0, 0] if np.dot(normal, [1, 0, 0]) > eps else [0, 1, 0]
    ortho1 = np.cross(normal, dummy)
    ortho2 = np.cross(normal, ortho1)
    return o3d.geometry.TriangleMesh(
        vertices=v3d([point + ortho1, point + ortho2, point - ortho1, point - ortho2]),
        triangles=v3i([[0, 1, 2], [0, 2, 3]]),
    )


def create_plane_and_normal_line_set(point, normal, scale=1.0, plane_subdivisions=3, eps=0.01):
    plane = o3d.geometry.LineSet.create_from_triangle_mesh(
        create_plane_triangle_mesh(point, normal).subdivide_midpoint(plane_subdivisions)
    )
    plane_normal = o3d.geometry.LineSet(
        points=v3d([point, point + normal / np.linalg.norm(normal)]),
        lines=v2i([[0, 1]]),
    )
    if scale != 1.0:
        plane = plane.scale(scale, center=point)
        plane_normal = plane_normal.scale(scale, center=point)
    return plane, plane_normal


###############################################################################
###############################################################################


def draw_source_aligned_to_target(source, target, transformation, source2=None, target2=None):
    """Visualize a target point cloud (in cyan) and a source point cloud
    (in yellow), transformed with a source -> target alignment matrix.

    http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
    """
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)

    source_tmp.paint_uniform_color([1, 0.706, 0])  # yellow
    source_tmp.transform(transformation)
    target_tmp.paint_uniform_color([0, 0.651, 0.929])  # cyan

    draw_list = [source_tmp, target_tmp]

    if source2 is not None:
        source2_tmp = copy.deepcopy(source2)
        source2_tmp.paint_uniform_color([1, 0.706, 0])  # yellow
        source2_tmp.transform(transformation)
        draw_list.append(source2_tmp)

    if target2 is not None:
        target2_tmp = copy.deepcopy(target2)
        target2_tmp.paint_uniform_color([0, 0.651, 0.929])  # cyan
        draw_list.append(target2_tmp)

    o3d.visualization.draw_geometries(draw_list)


###############################################################################
###############################################################################

from typing import Optional

import numpy as np


def plane_from_point_and_normal(point: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """ Returns `[a, b, c, d]`, such that `normal == [a, b, c]` and `normal.dot(point) + d == 0`. """
    a, b, c = normal
    d = -np.dot(normal, point)
    return np.array([a, b, c, d])


def plane_from_points(points: np.ndarray, use_old_method: bool = False) -> Optional[np.ndarray]:
    """
    Fits a plane to a collection of points. Returns None if the points do not span a plane.

    Reference: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    """
    assert points.ndim == 2 and points.shape[1] == 3
    n, _ = points.shape
    if n < 3:
        return None

    centroid = np.sum(points, axis=0) / n
    xs, ys, zs = np.rollaxis(points - centroid, axis=1)

    # Calculate full 3x3 covariance matrix, excluding symmetries
    # NOTE it is not necessary to divide by n since we normalize
    xx = np.sum(xs * xs, axis=0)
    xy = np.sum(xs * ys, axis=0)
    xz = np.sum(xs * zs, axis=0)
    yy = np.sum(ys * ys, axis=0)
    yz = np.sum(ys * zs, axis=0)
    zz = np.sum(zs * zs, axis=0)

    # cov = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    # # w, v = np.linalg.eig(cov)
    # # return v[np.argmin(w)]
    # w, v = np.linalg.eigh(cov)
    # # assert np.argmin(w) == 0
    # return v[0]

    det_x = yy * zz - yz * yz
    det_y = xx * zz - xz * xz
    det_z = xx * yy - xy * xy

    if use_old_method:
        normal = (
            np.array([det_x, xz * yz - xy * zz, xy * yz - xz * yy]) if det_x > det_y and det_x > det_z else
            np.array([xz * yz - xy * zz, det_y, xy * xz - yz * xx]) if det_y > det_z else
            np.array([xy * yz - xz * yy, xy * xz - yz * xx, det_z])
        )
    else:
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


def test_plane_from_points() -> None:
    import open3d as o3d

    # As in the reference article this generates 50000 random points on the surface sphere.
    # For each point, we find the closest 32 neighbors to that point and fit a plane to it,
    # color-coding the point based on the normal of the plane.

    def debug_color(unit_vector):
        return (unit_vector + 1) * 0.5

    def radial_noise():
        return 1 + np.random.normal(0.0, 0.05)

    points = []
    colors = []
    for _ in range(50000):
        while True:
            point = np.random.rand(3) * 2 - 1
            norm = np.linalg.norm(point)
            if 0.0001 < norm <= 1:
                break
        point /= norm
        points.append(point * radial_noise())
        colors.append(debug_color(point))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    kdtree = o3d.geometry.KDTreeFlann(pcd)

    old_colors = []
    new_colors = []
    for i, point in enumerate(pcd.points):
        k, indices, distance2 = kdtree.search_knn_vector_3d(query=point, knn=32)
        assert k == 32 and i in indices
        neighbors = pcd.select_by_index(indices)

        [*abc, d] = plane_from_points(np.asarray(neighbors.points))
        normal = np.asarray(abc)
        new_colors.append(debug_color(-normal if np.dot(normal, point) < 0 else normal))

        [*abc, d] = plane_from_points(np.asarray(neighbors.points), use_old_method=True)
        normal = np.asarray(abc)
        old_colors.append(debug_color(-normal if np.dot(normal, point) < 0 else normal))

    pcd2015 = o3d.geometry.PointCloud(pcd)
    pcd2015.colors = o3d.utility.Vector3dVector(old_colors)

    pcd2017 = o3d.geometry.PointCloud(pcd)
    pcd2017.colors = o3d.utility.Vector3dVector(new_colors)

    draw_list = []
    draw_list.append(o3d.geometry.TriangleMesh.create_coordinate_frame())
    # https://github.com/intel-isl/Open3D/pull/738#issuecomment-564785941

    draw_list.append(pcd)
    draw_list.append(pcd2015.translate([0, 0, -2.5]))
    draw_list.append(pcd2017.translate([0, 0, +2.5]))
    draw_list.append(o3d.geometry.TriangleMesh.create_coordinate_frame().translate([0, 0, +2.5]))

    o3d.visualization.draw_geometries(draw_list)

    del o3d


if __name__ == "__main__":
    test_plane_from_points()

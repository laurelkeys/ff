import numpy as np
import open3d as o3d
import ie.open3dy as o3dy


def test_plane_from_points() -> None:
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

        [*abc, d] = o3dy.plane_from_points(np.asarray(neighbors.points))
        normal = np.asarray(abc)
        new_colors.append(debug_color(-normal if np.dot(normal, point) < 0 else normal))

        [*abc, d] = o3dy.plane_from_points(np.asarray(neighbors.points), use_old_method=True)
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


if __name__ == "__main__":
    test_plane_from_points()

import numpy as np
import open3d as o3d

from plane_from_points import plane_from_points

if __name__ == "__main__":

    def debug_color(unit_vector):
        return (unit_vector + 1) * 0.5

    def radial_noise():
        return 1 + np.random.normal(0.0, 0.01)

    v3d = o3d.utility.Vector3dVector  # float64 numpy array of shape (n, 3) -> Open3D format
    v3i = o3d.utility.Vector3iVector  #   int32 numpy array of shape (n, 3) -> Open3D format
    v2i = o3d.utility.Vector2iVector  #   int32 numpy array of shape (n, 2) -> Open3D format

    def points_on_sphere(n):
        points = []
        colors = []
        for _ in range(n):
            while True:
                point = np.random.rand(3) * 2 - 1
                norm = np.linalg.norm(point)
                if 0.0001 < norm <= 1:
                    break
            point /= norm
            points.append(point * radial_noise())
            colors.append(debug_color(point))
        pcd = o3d.geometry.PointCloud()
        pcd.points = v3d(points)
        pcd.colors = v3d(colors)
        return pcd

    N, NN = 50000, 32

    pcd1 = points_on_sphere(N)
    kdtree = o3d.geometry.KDTreeFlann(pcd1)

    neighbors_indices = []
    planes = []
    colors = []
    for i, point in enumerate(pcd1.points):
        k, indices, distance2 = kdtree.search_knn_vector_3d(query=point, knn=NN)
        assert k == NN and i in indices
        neighbors = pcd1.select_by_index(indices)
        neighbors_indices.append(indices)
        [*abc, d] = plane_from_points(np.asarray(neighbors.points))
        normal = np.asarray(abc)
        if np.dot(normal, point) < 0:
            normal = -normal
            d = -d
        planes.append((normal, d))
        colors.append(debug_color(normal))

    pcd2 = o3d.geometry.PointCloud(pcd1)
    pcd2.colors = v3d(colors)

    draw_list = []
    draw_list.append(o3d.geometry.TriangleMesh.create_coordinate_frame())
    # https://github.com/intel-isl/Open3D/pull/738#issuecomment-564785941

    def create_plane_triangle_mesh(point, normal):
        dummy = [1, 0, 0] if np.dot(normal, [1, 0, 0]) > 0.01 else [0, 1, 0]
        ortho1 = np.cross(normal, dummy)
        ortho2 = np.cross(normal, ortho1)
        return o3d.geometry.TriangleMesh(
            vertices=v3d([point + ortho1, point + ortho2, point - ortho1, point - ortho2]),
            triangles=v3i([[0, 1, 2], [0, 2, 3]]),
        )

    def create_plane_and_normal_line_set(point, normal, scale=1.0):
        plane = o3d.geometry.LineSet.create_from_triangle_mesh(
            create_plane_triangle_mesh(point, normal).subdivide_midpoint(3)
        )
        plane_normal = o3d.geometry.LineSet(
            points=v3d([point, point + normal / np.linalg.norm(normal)]), lines=v2i([[0, 1]])
        )
        if scale != 1.0:
            plane = plane.scale(scale, center=point)
            plane_normal = plane_normal.scale(scale, center=point)
        return plane, plane_normal

    def split_points_by_index(pcd, index_list):
        return pcd.select_by_index(index_list), pcd.select_by_index(index_list, invert=True)

    INDEX = 0  # TODO interactively
    point = pcd1.points[INDEX]
    normal, d = planes[INDEX]
    n_indices = neighbors_indices[INDEX]

    neighbors, pcd1 = split_points_by_index(pcd1, neighbors_indices[INDEX])
    draw_list.append(pcd1)
    draw_list.append(neighbors.paint_uniform_color([0, 0, 0]))

    plane, plane_normal = create_plane_and_normal_line_set(point, normal, scale=0.75)
    draw_list.append(plane.paint_uniform_color([1, 0, 0]))
    draw_list.append(plane_normal.paint_uniform_color([1, 0, 0]))

    expected_plane, expected_plane_normal = create_plane_and_normal_line_set(point, point, scale=0.75)
    draw_list.append(expected_plane.paint_uniform_color([0, 0, 1]))
    draw_list.append(expected_plane_normal.paint_uniform_color([0, 0, 1]))

    o3d.visualization.draw_geometries(draw_list)

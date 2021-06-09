import numpy as np
import open3d as o3d

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
            points.append(point)  # * radial_noise())
            colors.append(debug_color(point))
        pcd = o3d.geometry.PointCloud()
        pcd.points = v3d(points)
        pcd.colors = v3d(colors)
        return pcd

    N, NN = 50000, 32
    pcd = points_on_sphere(N)

    draw_list = []
    draw_list.append(o3d.geometry.TriangleMesh.create_coordinate_frame())
    # https://github.com/intel-isl/Open3D/pull/738#issuecomment-564785941

    # draw_list.append(pcd)

    # NOTE for a perfect sphere, this would be np.linalg.norm([2, 2, 2]) == 2 * 3**0.5 ~= 3.4641
    diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    camera = [0, 0, diameter]
    radius = diameter * 100  # radius of the sperical projection
    draw_list.append(points_on_sphere(100).translate(camera).scale(1 / radius, camera))

    def compute_spherical_projection(points, camera, radius):
        spherical_projection = []
        for point in points:
            projected_point = point - camera
            norm = np.linalg.norm(projected_point)
            spherical_projection.append(
                projected_point + 2 * (radius - norm) * projected_point / norm
            )
        return o3d.geometry.PointCloud(v3d(spherical_projection))

    visible_mesh, pt_map = pcd.hidden_point_removal(camera, radius)
    draw_list.append(pcd.select_by_index(pt_map))
    draw_list.append(pcd.select_by_index(pt_map, invert=True).paint_uniform_color([0, 0, 0]))
    # draw_list.append(compute_spherical_projection(pcd.points, camera, radius))

    o3d.visualization.draw_geometries(draw_list)

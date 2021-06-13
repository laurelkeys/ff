from typing import Optional, NamedTuple

import numpy as np

from airsim.types import Vector3r

KNN_MAX = 24
KNN_RADIUS = None
FAST_NORMAL_COMPUTATION = True


class CameraVisible(NamedTuple):
    points: np.ndarray
    normals: np.ndarray
    closest_point: np.ndarray
    closest_point_distance: float
    spherical_projection_radius: float
    points_indices_in_original_pcd: np.ndarray


def visible_points_with_normals(
    camera_position: np.ndarray,
    pcd_points: np.ndarray,
    transform_pcd_points: Optional[np.ndarray],
    spherical_projection_radius_factor: float = 1.0,
) -> CameraVisible:
    n, _ = pcd_points.shape
    assert pcd_points.shape[1] == 3
    assert camera_position.shape == (3,)

    if transform_pcd_points is None:
        points = pcd_points
    else:
        assert transform_pcd_points.shape == (4, 4)
        points = np.hstack((pcd_points, np.ones((n, 1))))
        points = np.einsum("ij,nj->ni", transform_pcd_points, points)
        points = points[:, :3]

    points_squared_distance_to_camera = np.array(
        [np.dot(camera_position - point, camera_position - point) for point in points]
    )

    closest_point = points[np.argmin(points_squared_distance_to_camera)]
    closest_point_distance = np.sqrt(np.min(points_squared_distance_to_camera))

    # FIXME there are better ways than guessing this value for noisy point clouds
    spherical_projection_radius = closest_point_distance
    spherical_projection_radius *= spherical_projection_radius_factor

    import open3d as o3d
    import ie.open3dy as o3dy

    pcd = o3d.geometry.PointCloud(o3dy.v3d(points))

    # NOTE we could also use remove_radius_outlier and remove_statistical_outlier
    _, pt_map = pcd.hidden_point_removal(camera_position, spherical_projection_radius)
    pcd = pcd.select_by_index(pt_map)

    pcd.estimate_normals(
        search_param=(
            o3d.geometry.KDTreeSearchParamKNN(knn=KNN_MAX)
            if KNN_RADIUS is None
            else o3d.geometry.KDTreeSearchParamHybrid(radius=KNN_RADIUS, max_nn=KNN_MAX)
        ),
        fast_normal_computation=FAST_NORMAL_COMPUTATION,
    )

    pcd.orient_normals_towards_camera_location(camera_position)

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    del o3dy
    del o3d

    return CameraVisible(
        points, normals, closest_point, closest_point_distance, spherical_projection_radius, pt_map
    )


if __name__ == "__main__":
    import sys

    import argparse
    import airsim
    import open3d as o3d

    import ff
    import ie.airsimy as airsimy
    import ie.open3dy as o3dy

    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path", type=str)
    parser.add_argument("--align_path", type=str)
    parser.add_argument("--max_points", type=int, default=10_000)
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.pcd_path, print_progress=True)
    print(f"Point cloud has {len(pcd.points)} points")

    if len(pcd.points) > args.max_points:
        k = int(np.ceil(len(pcd.points) / args.max_points))
        pcd = pcd.uniform_down_sample(every_k_points=k)
        print(f"Downsampled to {len(pcd.points)} points")

    pcd_points = np.asarray(pcd.points)  # XXX flipping Y and Z coordinates
    pcd_points[:, 1] *= -1
    pcd_points[:, 2] *= -1

    if args.align_path is not None:
        align_pcd_to_airsim = np.loadtxt(args.align_path)
    else:
        align_pcd_to_airsim = None

    client = airsimy.connect(ff.SimMode.ComputerVision)
    # client = airsimy.connect(ff.SimMode.Multirotor)
    client.simFlushPersistentMarkers()

    camera = client.simGetCameraInfo(ff.CameraName.front_center)
    # camera = client.simGetCameraInfo(ff.CameraName.bottom_center)

    # NOTE the orientation is not considered for hidden point removal, so we could
    # use either the vehicle pose or of one the cameras's. However, if we consider
    # the (horizontal) FOV of the camera we can remove points that are out of view.
    camera_position = camera.pose.position.to_numpy_array()

    print(f"FOV = {camera.fov} degrees")
    print(f"Projection matrix = {camera.proj_mat}")
    print(f"Orientation (XYZW) = {camera.pose.orientation.to_numpy_array()}")

    # TODO remove pcd_points that are outside the camera's view frustum by using these values:
    front_axis, right_axis, up_axis = airsimy.AirSimNedTransform.local_axes_frame(camera.pose)

    front_axis = (front_axis / front_axis.get_length()).to_numpy_array()
    right_axis = (right_axis / right_axis.get_length()).to_numpy_array()
    up_axis = (up_axis / up_axis.get_length()).to_numpy_array()

    aspect_ratio = 16 / 9
    half_hfov = 0.5 * np.deg2rad(camera.fov)
    half_vfov = half_hfov / aspect_ratio
    ch, sh = np.cos(half_hfov), np.sin(half_hfov)
    cv, sv = np.cos(half_vfov), np.sin(half_vfov)

    # ref.: https://steve.hollasch.net/cgindex/math/rotvec.html
    def make_matrix(axis):
        x, y, z = axis
        return np.array([[0, z, -y], [-z, 0, x], [y, -x, 0]])

    right_matrix = make_matrix(right_axis)
    view_to_top = front_axis @ (np.eye(3) + sv * right_matrix + (1 - cv) * right_matrix @ right_matrix)

    up_matrix = make_matrix(up_axis)
    view_to_right = front_axis @ (np.eye(3) + sh * up_matrix + (1 - ch) * up_matrix @ up_matrix)

    view_to_bottom = -view_to_top + 2 * np.dot(view_to_top, front_axis) * front_axis
    view_to_left = -view_to_right + 2 * np.dot(view_to_right, front_axis) * front_axis

    top_left = Vector3r(*((view_to_top + view_to_left) * 0.5 + camera_position))
    top_right = Vector3r(*((view_to_top + view_to_right) * 0.5 + camera_position))
    bottom_left = Vector3r(*((view_to_bottom + view_to_left) * 0.5 + camera_position))
    bottom_right = Vector3r(*((view_to_bottom + view_to_right) * 0.5 + camera_position))

    client.simPlotLineList(
        [
            camera.pose.position, top_left,
            camera.pose.position, top_right,
            camera.pose.position, bottom_left,
            camera.pose.position, bottom_right,
            top_left, top_right,
            top_right, bottom_right,
            bottom_right, bottom_left,
            bottom_left, top_left,
        ],
        [1, 1, 1, 1],
        5,
        -1,
        True,
    )

    client.simPlotTransforms([camera.pose], 100, 5, -1, True)
    client.simPlotPoints([airsim.Vector3r(*p) for p in pcd_points], [1, 0, 0, 1], 3, -1, True)

    visible = visible_points_with_normals(
        camera_position,
        pcd_points,
        align_pcd_to_airsim,
        spherical_projection_radius_factor=100.0,
    )

    visible_points = [airsim.Vector3r(*p) for p in visible.points]
    visible_normals = [airsim.Vector3r(*(p + n)) for p, n in zip(visible.points, visible.normals)]

    client.simPlotPoints(visible_points, [0, 0, 1, 1], 6, -1, True)
    client.simPlotArrows(visible_points, visible_normals, [0, 1, 0, 1], 2, 100, -1, True)

    # pcd.orient_normals_towards_camera_location(pose.position.to_numpy_array())
    # visible_pcd, not_visible_pcd = o3dy.split_points_by_index(pcd, visible.points_indices_in_original_pcd)
    # visible_pcd.paint_uniform_color([0, 0, 1])
    # not_visible_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([visible_pcd])
    # o3d.visualization.draw_geometries([visible_pcd, not_visible_pcd])

    del sys, airsim, o3d, ff, airsimy, o3dy

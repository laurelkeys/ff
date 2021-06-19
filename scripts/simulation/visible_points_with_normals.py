from typing import Optional, NamedTuple

import numpy as np

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
    print(f"Projection matrix = {camera.proj_mat.matrix}")
    print(f"Orientation (XYZW) = {camera.pose.orientation.to_numpy_array()}")

    aspect_ratio = 16 / 9  # NOTE check that this matches AirSim's settings.json

    # Remove pcd_points that are outside of the camera's view frustum / viewport
    tl, tr, bl, br = airsimy.viewport_vectors(camera.pose, camera.fov, aspect_ratio)
    viewport_points = airsimy.frustum_plot_list_from_viewport_vectors(camera.pose, tl, tr, bl, br)
    top, left, right, bottom = airsimy.frustum_plane_normals_from_viewport_vectors(tl, tr, bl, br)

    in_frustum_mask = o3dy.points_above_planes_mask(
        pcd_points,
        np.array(
            [
                o3dy.plane_from_point_and_normal(camera_position, top),
                o3dy.plane_from_point_and_normal(camera_position, left),
                o3dy.plane_from_point_and_normal(camera_position, right),
                o3dy.plane_from_point_and_normal(camera_position, bottom),
            ]
        ),
    )

    pcd_points_in_frustum = pcd_points[in_frustum_mask]

    client.simPlotLineList(viewport_points, [1, 1, 1, 1], 3, -1, True)
    client.simPlotTransforms([camera.pose], 200, 3, -1, True)
    client.simPlotPoints([airsim.Vector3r(*p) for p in pcd_points[~in_frustum_mask]], [1, 1, 1, 1], 3, -1, True)
    client.simPlotPoints([airsim.Vector3r(*p) for p in pcd_points_in_frustum], [1, 0, 0, 1], 3, -1, True)

    visible = visible_points_with_normals(
        camera_position,
        pcd_points_in_frustum,  # pcd_points,
        align_pcd_to_airsim,
        spherical_projection_radius_factor=100.0,
    )

    visible_points = [airsim.Vector3r(*p) for p in visible.points]
    visible_normals = [airsim.Vector3r(*(p + n)) for p, n in zip(visible.points, visible.normals)]

    client.simPlotPoints(visible_points, [0, 0, 1, 1], 6, -1, True)
    client.simPlotArrows(visible_points, visible_normals, [0, 1, 0, 1], 2, 100, -1, True)

    del sys, argparse, airsim, o3d, ff, airsimy, o3dy

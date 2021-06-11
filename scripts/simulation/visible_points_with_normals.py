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


def visible_points_with_normals(
    camera_position: np.ndarray,
    # camera_orientation_xyzw: np.ndarray,
    pcd_points: np.ndarray,
    transform_pcd_points: Optional[np.ndarray],
    spherical_projection_radius: Optional[float] = None,
) -> CameraVisible:
    assert camera_position.shape == (3,)
    # assert camera_orientation_xyzw.shape == (4,)

    n, _ = pcd_points.shape
    assert pcd_points.shape[1] == 3

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

    if spherical_projection_radius is None:
        # FIXME this is a bad value guess for noisy point clouds
        spherical_projection_radius = closest_point_distance
        spherical_projection_radius *= 30  # FIXME

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
        points, normals, closest_point, closest_point_distance, spherical_projection_radius
    )


if __name__ == "__main__":
    import sys

    import ff
    import numpy as np
    import airsim
    import open3d as o3d
    import ie.airsimy as airsimy

    MAX_POINTS = 50_000

    pcd = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
    print(f"Point cloud has {len(pcd.points)} points")

    if len(pcd.points) > MAX_POINTS:
        k = int(np.ceil(len(pcd.points) / MAX_POINTS))
        pcd = pcd.uniform_down_sample(every_k_points=k)
        print(f"Downsampled to {len(pcd.points)} points")

    pcd_points = np.asarray(pcd.points)
    pcd_points[:, 1] *= -1
    pcd_points[:, 2] *= -1

    try:
        align_pcd_to_airsim = np.loadtxt(sys.argv[2])
    except IndexError:
        align_pcd_to_airsim = None

    client = airsimy.connect(ff.SimMode.ComputerVision)
    pose = client.simGetVehiclePose()

    # client = airsimy.connect(ff.SimMode.Multirotor)
    # pose = client.simGetCameraInfo(ff.CameraName.front_center).pose

    client.simFlushPersistentMarkers()

    print(f"{pose = }")
    print(f"{pcd = }")
    print(f"{align_pcd_to_airsim = }")

    client.simPlotPoints([airsim.Vector3r(*p) for p in pcd_points], [1, 0, 0, 1], 3, -1, True)

    visible = visible_points_with_normals(
        pose.position.to_numpy_array(),
        pcd_points,
        align_pcd_to_airsim,
        spherical_projection_radius=None,
    )

    print(f"{visible = }")
    visible_points = [airsim.Vector3r(*p) for p in visible.points]
    visible_normals = [airsim.Vector3r(*(p + n)) for p, n in zip(visible.points, visible.normals)]
    client.simPlotPoints(visible_points, [0, 0, 1, 1], 6, -1, True)
    client.simPlotArrows(visible_points, visible_normals, [0, 1, 0, 1], 2, 100, -1, True)

    # TODO plot everything inside AirSim

    # client.reset()

    del sys, ff, np, airsim, o3d, airsimy

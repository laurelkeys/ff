from typing import Tuple, Optional

import numpy as np

KNN_MAX = 24
KNN_RADIUS = None
FAST_NORMAL_COMPUTATION = True
SPHERICAL_PROJECTION_RADIUS = 1  # FIXME

# NOTE since this function is called during flight (i.e. from scripts that interact with AirSim)
# it can't "leak" Open3D imports, since the airsim and open3d packages have conflicts in msgpack.


def visible_points_with_normals(
    camera_position_xyz: np.ndarray,
    camera_orientation_xyzw: np.ndarray,
    reconstructed_pcd_points: np.ndarray,
    transform_points_to_airsim: Optional[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    assert camera_position_xyz.shape == (3,)
    assert camera_orientation_xyzw.shape == (4,)

    n, _ = reconstructed_pcd_points.shape
    assert reconstructed_pcd_points.shape[1] == 3

    import open3d as o3d
    import ie.open3dy as o3dy

    if transform_points_to_airsim is None:
        pcd = o3d.geometry.PointCloud(o3dy.v3d(reconstructed_pcd_points))
    else:
        assert transform_points_to_airsim.shape == (4, 4)
        points = np.hstack((reconstructed_pcd_points, np.ones((n, 1))))
        points = np.einsum("ij,nj->ni", transform_points_to_airsim, points)
        points = points[:, :3]
        pcd = o3d.geometry.PointCloud(o3dy.v3d(points))

    visible_mesh, pt_map = pcd.hidden_point_removal(
        camera_location=camera_position_xyz, radius=SPHERICAL_PROJECTION_RADIUS
    )

    print(f"{type(visible_mesh)} : {visible_mesh = }")
    print(f"{type(pt_map)} : {pt_map = }")

    pcd = pcd.select_by_index(pt_map)

    # TODO use remove_radius_outlier and remove_statistical_outlier

    pcd.estimate_normals(
        search_param=(
            o3d.geometry.KDTreeSearchParam(knn=KNN_MAX)
            if KNN_RADIUS is None
            else o3d.geometry.KDTreeSearchParamHybrid(radius=KNN_RADIUS, max_nn=KNN_MAX)
        ),
        fast_normal_computation=FAST_NORMAL_COMPUTATION,
    )

    pcd.orient_normals_towards_camera_location(camera_location=camera_position_xyz)

    visible_points = np.asarray(pcd.points)
    visible_normals = np.asarray(pcd.normals)

    del o3dy
    del o3d

    return visible_points, visible_normals

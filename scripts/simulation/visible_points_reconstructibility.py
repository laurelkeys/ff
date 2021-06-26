from typing import Tuple

import numpy as np

K1 = 32.0
ALPHA1 = np.pi / 16.0

K3 = 8.0
ALPHA3 = np.pi / 4.0

# NOTE the GSD value changes based on the reconstruction. For the default horizontal FOV = 90
# using 1280x720 resolution, a height of 50 meters leads to GSD = 50m * 1px / 360px = 0.139m.
D_GSD = 50.0
D_MAX = 2.0 * D_GSD


def logistic(x, k, x0):
    return 1.0 / (1.0 + np.exp(-k * (x - x0)))


def w1(alpha, k1=K1, alpha1=ALPHA1):
    return logistic(alpha, k1, alpha1)  # triangulation


def w2(d_m, d_max=D_MAX):
    return 1 - min(d_m / d_max, 1)


def w3(alpha, k3=K3, alpha3=ALPHA3):
    return 1.0 - logistic(alpha, k3, alpha3)  # matchability


def q(s: np.ndarray, n: np.ndarray, v1: np.ndarray, v2: np.ndarray) -> float:
    """ Returns the pairwise view contribution of views `v1` and `v2` to the
        reconstructibility of a sample point at position `s` with normal `n`.
    """
    assert s.shape == v1.shape == v2.shape == n.shape == (3,)
    assert np.isclose(np.linalg.norm(n), 1.0)

    sv1, sv2 = (v1 - s), (v2 - s)
    norm_sv1 = np.linalg.norm(sv1)
    norm_sv2 = np.linalg.norm(sv2)

    normalized_sv1 = sv1 / norm_sv1
    normalized_sv2 = sv2 / norm_sv2

    theta1 = np.arccos(np.dot(normalized_sv1, n))
    theta2 = np.arccos(np.dot(normalized_sv2, n))

    theta_m = max(theta1, theta2)
    alpha = np.arccos(np.dot(normalized_sv1, normalized_sv2))

    # NOTE the return of np.arccos is in the range [0, np.pi]
    if theta_m > np.deg2rad(45) or alpha > np.deg2rad(60):
        return 0.0

    return w1(alpha) * w2(max(norm_sv1, norm_sv2)) * w3(alpha) * np.cos(theta_m)


def h(s: np.ndarray, n: np.ndarray, vs: np.ndarray) -> float:
    """ Returns the reconstructibility of a sample point at
        position `s` with normal `n` over the view set `vs`.

        Note: assumes the sample isn't hidden from any view.
    """
    return sum(q(s, n, vs[i], vs[j]) for i in range(len(vs)) for j in range(i + 1, len(vs)))


def heuristic(
    sample_positions: np.ndarray,
    sample_normals: np.ndarray,
    view_positions: np.ndarray,
    visibility_matrix: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """ Returns the total reconstructibility of all surface samples
        and the summation of redundancy degree r(v) of each view v.

        Note: if visibility_matrix[i, j] is True then the i-th view
        sees the j-th sample, otherwise it doesn't (i.e. the sample
        is estimated to be hidden/occluded from this view position).
    """
    assert sample_positions.shape[0] == sample_normals.shape[0]
    assert sample_positions.shape[1] == sample_normals.shape[1] == view_positions.shape[1] == 3

    v_count = view_positions.shape[0]
    s_count = sample_positions.shape[0]

    assert visibility_matrix.shape == (v_count, s_count)

    reconstructibility = np.array(
        [
            h(s, n, view_positions[views_that_see_this_sample])
            for s, n, views_that_see_this_sample in zip(
                sample_positions, sample_normals, visibility_matrix.T
            )
        ]
    )

    redundancy_degree = np.array(
        [
            # np.min(reconstructibility[samples_that_this_view_sees][reconstructibility[samples_that_this_view_sees] != 0.0])
            np.min(reconstructibility[samples_that_this_view_sees])
            for samples_that_this_view_sees in visibility_matrix
        ]
    )

    assert redundancy_degree.shape == (v_count,)
    assert reconstructibility.shape == (s_count,)

    return reconstructibility, redundancy_degree


if __name__ == "__main__":
    import sys
    import argparse

    import airsim
    import open3d as o3d

    import ff
    import ie.airsimy as airsimy
    import ie.open3dy as o3dy

    from visible_points_with_normals import visible_points_with_normals

    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path", type=str)
    parser.add_argument("rec_path", type=str)
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

    view_poses = [
        airsim.Pose(rec.position, rec.orientation)
        for rec in airsimy.AirSimRecord.list_from(args.rec_path)
    ]

    client = airsimy.connect(ff.SimMode.ComputerVision)
    # client = airsimy.connect(ff.SimMode.Multirotor)
    client.simFlushPersistentMarkers()

    client.simPlotPoints([airsim.Vector3r(*p) for p in pcd_points], [1, 1, 1, 1], 3, -1, True)
    # client.simPlotTransforms(view_poses, 200, 3, -1, True)

    for i, pose in enumerate(view_poses):
        tl, tr, bl, br = airsimy.viewport_vectors(pose, hfov_degrees=90, aspect_ratio=(16 / 9))
        viewport_points = airsimy.frustum_plot_list_from_viewport_vectors(pose, tl, tr, bl, br)
        client.simPlotLineList(viewport_points, [1, 1, 1, 1], 3, 30, False)
        client.simPlotTransformsWithNames(
            [pose], [f"pose #{i}"], 200, 3, 1, [0, 0, 0, 1], 30
        )

    # ref.: https://stackoverflow.com/a/60291167
    visibility_matrix = np.array(
        [
            np.array(
                np.isin(
                    np.arange(len(pcd_points)),
                    visible_points_with_normals(
                        view.position.to_numpy_array(), pcd_points, align_pcd_to_airsim
                    ).points_indices_in_original_pcd,
                )
            )
            for view in view_poses
        ]
    )
    assert visibility_matrix.shape == (len(view_poses), len(pcd_points))

    pcd.orient_normals_towards_camera_location(view_poses[0].position.to_numpy_array())  # FIXME flip y and z

    view_positions = np.array([view.position.to_numpy_array() for view in view_poses])

    camera_indices_mask = [i for i in range(len(view_positions))]
    # camera_indices_mask = [0, 1]

    reconstructibility, redundancy_degree = heuristic(
        pcd_points,
        np.asarray(pcd.normals),
        view_positions[camera_indices_mask],
        visibility_matrix[camera_indices_mask, :]
    )

    print("total reconstructibility:", reconstructibility.sum())
    print("sum of redundancy degree:", redundancy_degree.sum())

    del sys, argparse, airsim, o3d, ff, airsimy, o3dy

from typing import Tuple, Optional, NamedTuple

import numpy as np

K1 = 32
ALPHA1 = np.pi / 16

K3 = 8
ALPHA3 = np.pi / 4

D_GSD = 1.0  # FIXME
D_MAX = 2.0 * D_GSD


def w1(alpha, k1=K1, alpha1=ALPHA1):
    return 1 / (1 + np.exp(-k1 * (alpha - alpha1)))


def w2(d_m, d_max=D_MAX):
    return 1 - min(d_m / d_max, 1)


def w3(alpha, k3=K3, alpha3=ALPHA3):
    return 1 - 1 / (1 + np.exp(-k3 * (alpha - alpha3)))


def q(s: np.ndarray, v1: np.ndarray, v2: np.ndarray, n: np.ndarray) -> float:
    assert s.shape == v1.shape == v2.shape == n.shape == (3,)
    assert np.linalg.norm(n) == 1.0

    sv1, sv2 = (v1 - s), (v2 - s)
    norm_sv1 = np.linalg.norm(sv1)
    norm_sv2 = np.linalg.norm(sv2)

    normalized_sv1 = sv1 / norm_sv1
    normalized_sv2 = sv2 / norm_sv2

    theta1 = np.arccos(np.dot(normalized_sv1, n))
    theta2 = np.arccos(np.dot(normalized_sv2, n))

    theta_m = max(theta1, theta2)
    alpha = np.arccos(np.dot(normalized_sv1, normalized_sv2))
    d_m = max(norm_sv1, norm_sv2)

    return w1(alpha) * w2(d_m) * w3(alpha) * np.cos(theta_m)


def visible_points_reconstructibility(
    camera_pose1: Tuple[np.ndarray, np.ndarray],
    camera_pose2: Tuple[np.ndarray, np.ndarray],
    visible_pcd_points: np.ndarray,
    visible_pcd_normals: np.ndarray,
) -> np.ndarray:
    position1, orientation1 = camera_pose1
    position2, orientation2 = camera_pose2

    assert position1.shape == position2.shape == (3,)
    assert orientation1.shape == orientation2.shape == (4,)

    # TODO ...

    return np.asarray([])


if __name__ == "__main__":
    # from visible_points_with_normals import visible_points_with_normals, CameraVisible
    pass

import os
import argparse

from typing import Optional

import numpy as np
import open3d as o3d
import ie.open3dy as o3dy


def visualize_cameras_and_point_cloud(
    source_cameras_path: str,
    target_cameras_path: str,
    source_scene_path: str,
    target_scene_path: str,
    matrix_path: Optional[str],
    flip_target_scene_yz: bool,
) -> None:
    assert os.path.isfile(source_cameras_path), f"Invalid file path: '{source_cameras_path}'"
    assert os.path.isfile(target_cameras_path), f"Invalid file path: '{target_cameras_path}'"
    assert os.path.isfile(source_scene_path), f"Invalid file path: '{source_scene_path}'"
    assert os.path.isfile(target_scene_path), f"Invalid file path: '{target_scene_path}'"

    source_cameras_pcd = o3d.io.read_point_cloud(source_cameras_path)  # from Meshroom's cameras.sfm
    target_cameras_pcd = o3d.io.read_point_cloud(target_cameras_path)  # from AirSim's airsim_rec.txt
    source_scene_pcd = o3d.io.read_point_cloud(source_scene_path)  # from Meshroom's sfm.ply (converted from sfm.abc)
    target_scene_pcd = o3d.io.read_point_cloud(target_scene_path)  # from sampled .fbx meshes taken from the UE4 env

    if flip_target_scene_yz:
        target_scene_points = np.asarray(target_scene_pcd.points)
        target_scene_points[:, [1, 2]] *= -1
        target_scene_pcd.points = o3dy.v3d(target_scene_points)

    print("\n> Initial alignment")
    transformation = np.identity(4)
    o3dy.draw_source_aligned_to_target(
        source_cameras_pcd, target_cameras_pcd, np.identity(4), source_scene_pcd, target_scene_pcd
    )

    if matrix_path is not None:
        assert os.path.isfile(matrix_path), f"Invalid file path: '{matrix_path}'"
        transformation = np.loadtxt(matrix_path)

        print("\n> Alignment with matrix")
        print(transformation)
        o3dy.draw_source_aligned_to_target(
            source_cameras_pcd,
            target_cameras_pcd,
            transformation,
            source_scene_pcd,
            target_scene_pcd,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Interactively visualize both the camera trajectories as point clouds and "
        "the reconstructed (source) and ground-truth (target) scene point clouds, aligned by a given matrix"
    )

    parser.add_argument("source_cameras_path", type=str, help="Path to the source camera point cloud PLY file")
    parser.add_argument("target_cameras_path", type=str, help="Path to the target camera point cloud PLY file")
    parser.add_argument("source_scene_path", type=str, help="Path to the source (reconstructed) scene point cloud PLY file")
    parser.add_argument("target_scene_path", type=str, help="Path to the target (ground-truth) scene point cloud PLY file")
    parser.add_argument("--source_to_target_path", "-s2t", type=str, help="Path to a alignment matrix TXT file")
    parser.add_argument("--flip_yz", "-yz", action="store_true", help="Flip Y and Z axis of target point cloud")

    args = parser.parse_args()

    visualize_cameras_and_point_cloud(
        args.source_cameras_path,
        args.target_cameras_path,
        args.source_scene_path,
        args.target_scene_path,
        args.source_to_target_path,
        args.flip_yz,
    )

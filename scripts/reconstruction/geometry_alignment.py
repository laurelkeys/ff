import os
import argparse

from typing import Optional

import numpy as np
import open3d as o3d

from align_with_icp import draw_registration_result


def visualize_alignment(source_path: str, target_path: str, transform_path: Optional[str]) -> None:
    assert os.path.isfile(source_path), f"Invalid file path: '{source_path}'"
    assert os.path.isfile(target_path), f"Invalid file path: '{target_path}'"

    source_pcd = o3d.io.read_point_cloud(source_path)  # parsed from Meshroom's cameras.sfm
    target_pcd = o3d.io.read_point_cloud(target_path)  # parsed from AirSim's airsim_rec.txt

    print("\n> Initial alignment")
    draw_registration_result(source_pcd, target_pcd, np.identity(4))

    if transform_path is not None:
        assert os.path.isfile(transform_path), f"Invalid file path: '{transform_path}'"
        print("\n> Alignment matrix")
        transform = np.loadtxt(transform_path)
        print(transform)
        draw_registration_result(source_pcd, target_pcd, transform)

    # XXX debugging
    print("\n> Alignment matrix")
    transform = np.array(
        [
            [0.994, -0.031, -0.102, -0.437],
            [0.038, 0.997, 0.07, 11.211],
            [0.1, -0.074, 0.992, 9.209],
            [0, 0, 0, 1],
        ]
    )
    print(transform)
    transform = np.linalg.inv(transform)
    draw_registration_result(source_pcd, target_pcd, transform)
    print(transform)
    draw_registration_result(source_pcd, target_pcd, transform)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize the alignment matrix between two point clouds in an interactive window"
    )

    parser.add_argument("source_path", type=str, help="Path to the source point cloud PLY file")
    parser.add_argument("target_path", type=str, help="Path to the target point cloud PLY file")
    parser.add_argument("--transform_path", type=str, help="Path to the alignment matrix TXT file")

    args = parser.parse_args()

    visualize_alignment(args.source_path, args.target_path, args.transform_path)

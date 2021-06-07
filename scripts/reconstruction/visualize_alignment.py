import os
import argparse

from typing import Optional

import numpy as np
import open3d as o3d

from open3dy import draw_source_aligned_to_target


def visualize_alignment(source_path: str, target_path: str, matrix_path: Optional[str]) -> None:
    assert os.path.isfile(source_path), f"Invalid file path: '{source_path}'"
    assert os.path.isfile(target_path), f"Invalid file path: '{target_path}'"

    source_pcd = o3d.io.read_point_cloud(source_path)  # parsed from Meshroom's cameras.sfm
    target_pcd = o3d.io.read_point_cloud(target_path)  # parsed from AirSim's airsim_rec.txt

    print("\n> Initial alignment")
    transformation = np.identity(4)
    draw_source_aligned_to_target(source_pcd, target_pcd, np.identity(4))

    if matrix_path is not None:
        assert os.path.isfile(matrix_path), f"Invalid file path: '{matrix_path}'"
        transformation = np.loadtxt(matrix_path)

        print("\n> Alignment with matrix")
        print(transformation)
        draw_source_aligned_to_target(source_pcd, target_pcd, transformation)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Interactively visualize the alignment matrix between two point clouds"
    )
    parser.add_argument("source_path", type=str, help="Path to the source point cloud PLY file")
    parser.add_argument("target_path", type=str, help="Path to the target point cloud PLY file")
    parser.add_argument(
        "--source_to_target_path", "-s2t", type=str, help="Path to a alignment matrix TXT file"
    )
    args = parser.parse_args()
    visualize_alignment(args.source_path, args.target_path, args.source_to_target_path)

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
        transform = np.loadtxt(transform_path)
    else:
        # TODO parse directly from .mlp
        transform = np.array(
            [
                [4.97279, 0.277086, 0.472141, -0.318997],
                [0.240651, -4.98187, 0.389083, 3.7313],
                [0.491713, -0.364035, -4.96528, 11.8308],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    print("\n> Alignment matrix")
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

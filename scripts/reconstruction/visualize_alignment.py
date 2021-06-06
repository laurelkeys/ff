import os
import copy
import argparse

from typing import Optional

import numpy as np
import open3d as o3d


def visualize_alignment(source_path: str, target_path: str, matrix_path: Optional[str]) -> None:
    assert os.path.isfile(source_path), f"Invalid file path: '{source_path}'"
    assert os.path.isfile(target_path), f"Invalid file path: '{target_path}'"

    source_pcd = o3d.io.read_point_cloud(source_path)  # parsed from Meshroom's cameras.sfm
    target_pcd = o3d.io.read_point_cloud(target_path)  # parsed from AirSim's airsim_rec.txt

    def draw_source_aligned_to_target(source, target, transformation):
        """ Visualize a target point cloud (in cyan) and a source point cloud
            (in yellow), transformed with a source -> target alignment matrix.

            http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
        """
        source_tmp = copy.deepcopy(source)
        target_tmp = copy.deepcopy(target)

        source_tmp.paint_uniform_color([1, 0.706, 0])  # yellow
        source_tmp.transform(transformation)
        target_tmp.paint_uniform_color([0, 0.651, 0.929])  # cyan

        o3d.visualization.draw_geometries([source_tmp, target_tmp])

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

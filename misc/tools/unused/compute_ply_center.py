import copy
import argparse

import numpy as np
import open3d as o3d

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "Computes the center of mass of a PLY file by averaging its coordinates."
    )
    parser.add_argument("input", type=str)
    args = parser.parse_args()

    pcd_in = o3d.io.read_point_cloud(args.input)
    center = np.asarray(pcd_in.points).mean(axis=0)
    center_aabb = 0.5 * (pcd_in.get_min_bound() + pcd_in.get_max_bound())

    print(f"{center = }")
    print(f"{center_aabb = }")
    print(f"{pcd_in.get_min_bound() = }")
    print(f"{pcd_in.get_max_bound() = }")

    o3d.visualization.draw_geometries(
        [
            pcd_in,
            # o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=center),
            # o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=center_aabb),
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0.0, 5.7, 0.0]),  # Statue_airsim_flipped
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[-2.2, -35.4, 13.2]),  # Building_03_C_airsim_flipped
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[-55.4, -10.9, 16.5]),  # Building_07_A_airsim_flipped
        ]
    )

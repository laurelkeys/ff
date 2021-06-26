# python transform_ply.py Statue.ply Statue_airsim.ply 67.2 -57.8 -72.1
# python transform_ply.py Building_03_C.ply Building_03_C_airsim.ply -51.4 87.9 -1.2
# python transform_ply.py Building_07_A.ply Building_07_A_airsim.ply -51.4 87.9 -1.2

import copy
import argparse

import numpy as np
import open3d as o3d

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Saves a PLY file with transformed coordinates.")

    parser.add_argument("input", type=str)
    parser.add_argument("output", type=str)

    parser.add_argument("offset", type=float, nargs=3)

    parser.add_argument("--flip_x", "-x", action="store_true")
    parser.add_argument("--flip_y", "-y", action="store_true")
    parser.add_argument("--flip_z", "-z", action="store_true")

    args = parser.parse_args()
    print(args)

    tx, ty, tz = args.offset

    sx = -1.0 if args.flip_x else 1.0
    sy = -1.0 if args.flip_y else 1.0
    sz = -1.0 if args.flip_z else 1.0

    matrix = np.array([
        [sx, 0., 0., tx],
        [0., sy, 0., ty],
        [0., 0., sz, tz],
        [0., 0., 0., 1.],
    ])

    pcd_in = o3d.io.read_point_cloud(args.input)
    # o3d.visualization.draw_geometries([pcd_in])

    pcd_out = copy.deepcopy(pcd_in)
    pcd_out.transform(matrix)

    # o3d.visualization.draw_geometries([pcd_in, pcd_out])
    o3d.io.write_point_cloud(args.output, pcd_out)

import argparse

import numpy as np
import open3d as o3d

# ref.: https://github.com/intel-isl/Open3D/blob/master/examples/Python/Basic/pointcloud.py

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("fname", type=str, default="Ignatius.recall.ply", nargs='?')
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.fname)
    o3d.visualization.draw_geometries([pcd])

import argparse
import numpy as np
import open3d as o3d


# ref.: https://stackoverflow.com/a/58191090
#       https://github.com/intel-isl/Open3D/issues/404
#       https://github.com/intel-isl/Open3D/blob/master/examples/Python/Advanced/interactive_visualization.py


def crop_geometry(ply_file):
    print("Manual geometry cropping")
    print("1) Press 'K' to lock the screen and switch to selection mode")
    print("2) Drag for rectangle selection,")
    print("   or use Ctrl+LMB for polygon selection")
    print("3) Press 'C' to get a selected geometry and to save it")
    print("4) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud(ply_file)
    o3d.visualization.draw_geometries_with_editing([pcd])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactively crop a PLY file")
    parser.add_argument("ply_file")

    args = parser.parse_args()

    crop_geometry(args.ply_file)

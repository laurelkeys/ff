import os
import argparse

import open3d as o3d


def crop_geometry(ply_path: str, view_bb: bool) -> None:
    assert os.path.isfile(ply_path), f"Invalid file path: '{ply_path}'"

    pcd = o3d.io.read_point_cloud(ply_path)
    # np_pcd = np.asarray(pcd.points)
    center = pcd.get_center()
    aabb = pcd.get_axis_aligned_bounding_box()
    obb = pcd.get_oriented_bounding_box()
    print(f"Center: {center}")
    print(f"AABB: max_bound = {aabb.max_bound}, min_bound = {aabb.min_bound}")
    print(f"OBB:  center = {obb.center}, extent = {obb.extent}")

    import numpy as np
    print(np.asarray(aabb.get_box_points()))
    print(np.asarray(obb.get_box_points()))

    if view_bb:
        aabb.color = [1, 0, 0]
        obb.color = [0, 1, 0]
        o3d.visualization.draw_geometries([pcd, aabb, obb])

    print("Manual geometry cropping")
    print("1) Press 'X', 'Y' or 'Z' to align the geometry with an orthogonal axis")
    print("2) Press 'K' to lock the screen and switch to selection mode")
    print("3) Drag for rectangle selection, or use Ctrl+LMB for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")

    o3d.visualization.draw_geometries_with_editing([pcd])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactively crop a PLY file")
    parser.add_argument("ply_path", type=str, help="Path to the PLY file")
    parser.add_argument("--view_bb", "-bb", action="store_true")
    args = parser.parse_args()
    crop_geometry(args.ply_path, args.view_bb)

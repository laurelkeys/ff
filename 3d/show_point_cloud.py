import sys
import numpy as np
import open3d as o3d

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(sys.argv[1])
    aabb = pcd.get_axis_aligned_bounding_box()

    # coordinate axis
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    print("min:", pcd.get_min_bound())
    print("max:", pcd.get_max_bound())
    
    o3d.visualization.draw_geometries([pcd, mesh_frame, aabb.scale(1.1)])

# ref.: http://www.open3d.org/docs/release/tutorial/Basic/visualization.html
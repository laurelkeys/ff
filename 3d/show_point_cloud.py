import sys
import numpy as np
import open3d as o3d

def main(fname, bbox_scale=1):
    pcd = o3d.io.read_point_cloud()
    aabb = pcd.get_axis_aligned_bounding_box()

    # coordinate axis
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    print("min:", pcd.get_min_bound())
    print("max:", pcd.get_max_bound())
    

    o3d.visualization.draw_geometries([pcd, mesh_frame, aabb.scale(bbox_scale)])

if __name__ == "__main__":
    try:    bbox_scale = float(sys.argv[2])
    except: bbox_scale = 1
    main(sys.argv[1], bbox_scale)

# ref.: http://www.open3d.org/docs/release/tutorial/Basic/visualization.html
# {
# 	"class_name" : "ViewTrajectory",
# 	"interval" : 29,
# 	"is_loop" : false,
# 	"trajectory" : 
# 	[
# 		{
# 			"boundingbox_max" : [ 3.6501868699999998, 2.5728734475000001, 9.4113190849999988 ],
# 			"boundingbox_min" : [ -2.3517677499999996, -5.6753468175000004, -1.8354641050000002 ],
# 			"field_of_view" : 80.0,
# 			"front" : [ -0.018948916972519419, -0.0062009415059815394, -0.99980122367899116 ],
# 			"lookat" : [ 0.56018541402301136, -1.5503583101639871, 3.7864076982734947 ],
# 			"up" : [ -0.99978684180357469, 0.0083169729427907783, 0.018897061081107853 ],
# 			"zoom" : 0.69999999999999996
# 		}
# 	],
# 	"version_major" : 1,
# 	"version_minor" : 0
# }
import os
import copy
import argparse

import open3d as o3d
import numpy as np


def draw_registration_result(source, target, transformation):
    """ Visualize a target point cloud (in cyan) and a source point cloud
        (in yellow), transformed with an alignment transformation matrix.

        http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
    """
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)

    # NOTE paint_uniform_color() and transform() change the point cloud, hence the copies
    source_tmp.paint_uniform_color([1, 0.706, 0])  # yellow
    target_tmp.paint_uniform_color([0, 0.651, 0.929])  # cyan
    source_tmp.transform(transformation)

    o3d.visualization.draw_geometries([source_tmp, target_tmp])


def show_registration_result(source, target, result):
    print(result)
    print("> Transformation is:")
    print(result.transformation)

    draw_registration_result(source, target, result.transformation)


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.meshroom_ply), f"Invalid file path: '{args.meshroom_ply}'"
    assert os.path.isfile(args.airsim_ply), f"Invalid file path: '{args.airsim_ply}'"

    print(f"(source) meshroom_ply path: '{args.meshroom_ply}'")
    print(f"(target) airsim_ply path: '{args.airsim_ply}'")
    print()

    meshroom_pcd = o3d.io.read_point_cloud(args.meshroom_ply)  # parsed from cameras.sfm
    airsim_pcd = o3d.io.read_point_cloud(args.airsim_ply)  # parsed from airsim_rec.txt

    threshold = 10.5  # Maximum correspondence points-pair distance
    init_transformation = np.identity(4)  # Initial transformation estimation

    # Align the point clouds with a global algorithm.                                                       http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html
    #                                                                                                       http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.evaluate_registration.html
    print("\n> Initial alignment")

    init_registration = o3d.pipelines.registration.evaluate_registration(
        meshroom_pcd, airsim_pcd,
        threshold, init_transformation
    )

    show_registration_result(meshroom_pcd, airsim_pcd, init_registration)

    # Refine the alignment with ICP.                                                                        http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
    #                                                                                                       http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_icp.html
    print("\n> Applying point-to-point ICP")

    estimation_method, convergence_criteria = (
        # One of: 'PointToPoint', 'PointToPlane', 'ForColoredICP'
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        # By default: max_iteration = 30
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    p2p_registration = o3d.pipelines.registration.registration_icp(
        meshroom_pcd, airsim_pcd,
        threshold, init_registration.transformation,
        estimation_method, convergence_criteria
    )

    show_registration_result(meshroom_pcd, airsim_pcd, p2p_registration)

    # TODO visualize the results
    # http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html

    # TODO apply the same transformation to the reconstructed scene (in a different script)
    # http://www.open3d.org/docs/release/tutorial/geometry/transformation.html


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Applies the ICP (Iterative Closest Point) registration algorithm to align"
        " a point cloud generated by Meshroom to the ground truth values obtained from AirSim."
    )

    parser.add_argument("meshroom_ply", type=str, help="Path to the PLY file representing Meshroom's reconstruction")
    parser.add_argument("airsim_ply", type=str, help="Path to the PLY file representing AirSim's ground truth data")

    # TODO add arguments for:
    #      - `threshold`
    #      - `max_iteration`
    #      - `init_transformation`

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

import os
import copy
import argparse

import numpy as np
import open3d as o3d

registration3d = o3d.pipelines.registration


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

    # Show the initial (un)alignment.
    #
    print("\n> Initial alignment")  # TODO pass the transformation matrix as an argument
    init_transformation = np.identity(4)  # Initial transformation estimation                               http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.evaluate_registration.html

    init_registration = registration3d.evaluate_registration(
        meshroom_pcd, airsim_pcd,
        args.threshold, init_transformation
    )
    show_registration_result(meshroom_pcd, airsim_pcd, init_registration)

    # NOTE one of: 'PointToPoint', 'PointToPlane', 'ForColoredICP'                                          http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.TransformationEstimationPointToPoint.html
    estimation_method = registration3d.TransformationEstimationPointToPoint(with_scaling=True)

    # Align the point clouds with a global algorithm.                                                       http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html
    #                                                                                                       http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_ransac_based_on_correspondence.html
    print("\n> Applying global RANSAC registration")

    correspondence = None  # FIXME http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_ransac_based_on_correspondence.html

    ransac_criteria = registration3d.RANSACConvergenceCriteria(
        max_iteration=(args.max_iter_ransac or args.max_iteration),
        max_validation=1000
    )

    ransac_registration = registration3d.registration_ransac_based_on_correspondence(
        meshroom_pcd, airsim_pcd,
        correspondence, args.threshold,
        estimation_method, ransac_criteria
    )
    show_registration_result(meshroom_pcd, airsim_pcd, ransac_registration)

    # Refine the alignment with ICP.                                                                        http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
    #                                                                                                       http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_icp.html
    print("\n> Applying point-to-point ICP registration")

    icp_criteria = registration3d.ICPConvergenceCriteria(
        max_iteration=(args.max_iter_icp or args.max_iteration),
    )

    icp_registration = registration3d.registration_icp(
        meshroom_pcd, airsim_pcd,
        args.threshold, init_registration.transformation,
        estimation_method, icp_criteria
    )
    show_registration_result(meshroom_pcd, airsim_pcd, icp_registration)


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

    parser.add_argument("--threshold", type=float, default=7.5, help="Maximum correspondence points-pair distance  (default: %(default)f)")
    parser.add_argument("--max_iteration", type=int, default=1000, help="Maximum iterations used for RANSAC and ICP.  (default: %(default)d)")
    parser.add_argument("--max_iter_icp", type=int, help="Set a specific number of maximum iterations for ICP.")
    parser.add_argument("--max_iter_ransac", type=int, help="Set a specific number of maximum iterations for RANSAC.")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)


# TODO see https://github.com/intel-isl/TanksAndTemples/blob/master/python_toolbox/evaluation/registration.py#L97
# TODO apply the same transformation to the reconstructed scene (in a different script)
# http://www.open3d.org/docs/release/tutorial/geometry/transformation.html

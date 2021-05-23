import os
import copy
import argparse

import numpy as np
import open3d as o3d


# FIXME temporarily testing hardcoded values generated with new_sfm_from_rec.py
MESHROOM_TO_AIRSIM_CORRESPONDENCES = (
    [[0, 33], [1, 24], [2, 40], [3, 64], [4, 69], [5, 1], [6, 14], [7, 47], [8, 37], [9, 10], [10, 31], [11, 46], [12, 27], [13, 21], [14, 34], [15, 23], [16, 52], [17, 29], [18, 51], [19, 75], [20, 11], [21, 25], [22, 8], [23, 38], [24, 26], [25, 73], [26, 45], [27, 59], [28, 63], [29, 6], [30, 76], [31, 5], [32, 58], [33, 22], [34, 15], [35, 56], [36, 18], [37, 2], [38, 32], [39, 66], [40, 4], [41, 70], [42, 57], [43, 55], [44, 7], [45, 0], [46, 9], [47, 74], [48, 44], [49, 36], [50, 39], [51, 12], [52, 41], [53, 17], [54, 28], [55, 72], [56, 71], [57, 13], [58, 65], [59, 20], [60, 60], [61, 30], [62, 49], [63, 35], [64, 68], [65, 43], [66, 61], [67, 53], [68, 19], [69, 50], [70, 42], [71, 62], [72, 48], [73, 16], [74, 3], [75, 54], [76, 77], [77, 67]]
    # [[0, 24], [1, 34], [2, 98], [3, 104], [4, 66], [5, 59], [6, 61], [7, 70], [8, 105], [9, 37], [10, 36], [11, 107], [12, 63], [13, 89], [14, 102], [15, 31], [16, 68], [17, 51], [18, 79], [19, 106], [20, 73], [21, 80], [22, 103], [23, 46], [24, 33], [25, 64], [26, 44], [27, 77], [28, 38], [29, 39], [30, 47], [31, 78], [32, 82], [33, 65], [34, 62], [35, 99], [36, 48], [37, 88], [38, 108], [39, 35], [40, 53], [41, 40], [42, 45], [43, 100], [44, 30], [45, 96], [46, 97], [47, 54], [48, 27], [49, 110], [50, 95], [51, 67], [52, 55], [53, 93], [54, 83], [55, 32], [56, 28], [57, 56], [58, 101], [59, 69], [60, 92], [61, 74], [62, 76], [63, 43], [64, 75], [65, 85], [66, 41], [67, 84], [68, 109], [69, 52], [70, 50], [71, 29], [72, 42], [73, 49], [74, 94], [75, 72], [76, 57], [77, 91], [78, 25], [79, 60], [80, 71], [81, 86], [82, 90], [83, 26], [84, 81], [85, 58], [86, 87], [87, 23]]
)


# XXX @volatile used in geometry_alignment.py
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
    print("\n> Initial alignment")  # TODO pass the source to target transformation matrix as an argument
    init_transformation = np.identity(4)  # initial transformation estimation                               http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.evaluate_registration.html

    registration3d = o3d.pipelines.registration

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

    ransac_correspondences = (  # correspondence indices between source and target point clouds
        o3d.utility.Vector2iVector(np.array(MESHROOM_TO_AIRSIM_CORRESPONDENCES))
    )

    ransac_criteria = registration3d.RANSACConvergenceCriteria(
        max_iteration=(args.ransac_max_iter or args.max_iteration),
        confidence=0.999,
        # max_validation=1000  # FIXME open3d 12 replaced `max_validation` with `confidence`
    )

    ransac_registration = registration3d.registration_ransac_based_on_correspondence(
        meshroom_pcd, airsim_pcd,
        ransac_correspondences, args.threshold,
        estimation_method, args.n_ransac, criteria=ransac_criteria
        # estimation_method, args.n_ransac, ransac_criteria  # FIXME open3d 12 added a new argument `checkers`
    )
    show_registration_result(meshroom_pcd, airsim_pcd, ransac_registration)

    # Refine the alignment with ICP.                                                                        http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
    #                                                                                                       http://www.open3d.org/docs/release/python_api/open3d.pipelines.registration.registration_icp.html
    print("\n> Applying point-to-point ICP registration")

    icp_criteria = registration3d.ICPConvergenceCriteria(
        max_iteration=(args.icp_max_iter or args.max_iteration),
    )

    icp_registration = registration3d.registration_icp(
        meshroom_pcd, airsim_pcd,
        args.threshold, ransac_registration.transformation,
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

    parser.add_argument("--n_ransac", type=int, default=6, help="Fit RANSAC with this number of correspondences  (default: %(default)d)")
    parser.add_argument("--threshold", type=float, default=1.0, help="Maximum correspondence points-pair distance  (default: %(default)f)")
    parser.add_argument("--max_iteration", type=int, default=1000, help="Maximum iterations used for RANSAC and ICP  (default: %(default)d)")
    parser.add_argument("--icp_max_iter", type=int, help="Set a specific number of maximum iterations for ICP")
    parser.add_argument("--ransac_max_iter", type=int, help="Set a specific number of maximum iterations for RANSAC")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)


# TODO see https://github.com/intel-isl/TanksAndTemples/blob/master/python_toolbox/evaluation/registration.py#L97
# TODO apply the same transformation to the reconstructed scene (in a different script)
# http://www.open3d.org/docs/release/tutorial/geometry/transformation.html

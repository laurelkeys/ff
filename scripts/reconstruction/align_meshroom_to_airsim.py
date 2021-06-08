import os
import copy
import argparse

from collections import namedtuple

import numpy as np
import open3d as o3d

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser

import open3dy as o3dy

from solve_helmert_transform_lstsq import (
    compute_helmert_A_b,
    solve_helmert_lstsq,
    compute_helmert_matrix,
)

###############################################################################
## open3d-related #############################################################
###############################################################################


RANSAC_N = 4  # number of correspondences used to fit RANSAC

MAX_CORRESPONDENCE_DISTANCE = 1.5  # maximum correspondence points-pair distance

ESTIMATION_METHOD = o3dy.registration.TransformationEstimationPointToPoint(with_scaling=True)

RANSAC_CRITERIA = o3dy.RANSACConvergenceCriteria(max_iteration=1000)

ICP_CRITERIA = o3dy.registration.ICPConvergenceCriteria(
    # ICP stops if the relative change (difference) of fitness or inliner RMSE scores hit
    # relative_fitness or relative_rmse, or if the iteration number exceeds max_iteration
    relative_fitness=1e-06,
    relative_rmse=1e-06,
    max_iteration=30,
)


def evaluate_registration(source, target, transformation=np.eye(4)):
    result = o3dy.registration.evaluate_registration(
        source, target, MAX_CORRESPONDENCE_DISTANCE, transformation
    )

    fitness = result.fitness  # higher is better
    inlier_rmse = result.inlier_rmse  # lower is better
    correspondence_set = np.asarray(result.correspondence_set)

    # Sanity check (the i-th source point should match the i-th target)
    assert np.all(correspondence_set[:, 0] == correspondence_set[:, 1])
    assert fitness == (len(correspondence_set) / len(target.points))
    assert np.all(transformation == result.transformation)

    Result = namedtuple("Result", "fitness inlier_rmse correspondence_set")
    return Result(fitness, inlier_rmse, correspondence_set)


def apply_transformation_to_points(transformation, points):
    array_of_points = np.asarray(points)
    n, d = array_of_points.shape
    assert transformation.shape == (4, 4) and d == 3
    array_of_homogeneous_points = np.hstack((array_of_points, np.ones((n, 1))))
    transformed_points = np.einsum("ij,nj->ni", transformation, array_of_homogeneous_points)
    return transformed_points[:, :3]  # remove the 1 added for points' homogenous coordinates


# TODO can we use this to improve the ICP registration parameters?
def compute_points_pair_distances(source, target, transformation):
    assert len(source.points) == len(target.points)
    source = copy.deepcopy(source).transform(transformation)
    # NOTE we could also avoid making a copy of the point cloud
    # by doing [*p_source, _] = transformation @ [*p_source, 1]
    return [
        np.linalg.norm(p_source - p_target)
        for p_source, p_target in zip(source.points, target.points)
    ]


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    airsim_records = AirSimRecord.dict_from(args.airsim_rec)
    meshroom_views, meshroom_poses = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.meshroom_sfm)
    )

    # NOTE do *not* assume that the points we read from Meshroom's cameras.sfm
    # come in the same order as AirSim's *rec.txt, they *must* be paired based
    # on the image filenames: views[id]["path"] <-> ImageFile
    def path_matches_image_file(meshroom_path, airsim_image_file):
        meshroom_name = os.path.basename(meshroom_path)
        airsim_name = os.path.basename(airsim_image_file)
        return meshroom_name.endswith(airsim_name)

    record_view_pose_matches = []
    for image_file, record in airsim_records.items():
        [(view, pose)] = [
            (view, meshroom_poses[view.pose_id])
            for view in meshroom_views.values()
            if path_matches_image_file(view.path, image_file)
        ]
        record_view_pose_matches.append((record, view, pose))

    # print(f"{airsim_records = }")
    # print(f"{meshroom_views = }")
    # print(f"{meshroom_poses = }")
    # print(f"{record_view_pose_matches = }")

    airsim_points, meshroom_points = zip(
        *[(r.position.to_numpy_array(), np.array(p.center)) for r, _, p in record_view_pose_matches]
    )
    pcd_airsim = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(airsim_points)))  # target
    pcd_meshroom = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(meshroom_points)))  # source
    index_correspondences = o3dy.v2i(np.array([[i, i] for i in range(len(pcd_airsim.points))]))

    #
    # Apply RANSAC global registration based on point correspondences
    #

    ransac_registration = o3dy.registration.registration_ransac_based_on_correspondence(
        pcd_meshroom,
        pcd_airsim,
        index_correspondences,
        MAX_CORRESPONDENCE_DISTANCE,
        ESTIMATION_METHOD,
        RANSAC_N,
        criteria=RANSAC_CRITERIA,
    )
    ransac_transformation = ransac_registration.transformation

    #
    # Compute the Helmert matrix as a second approximation (after RANSAC)
    # NOTE we could sample a subset of points if this starts getting slow
    #

    As, bs = compute_helmert_A_b(
        xs_source=apply_transformation_to_points(ransac_transformation, pcd_meshroom.points),
        xs_target=pcd_airsim.points,
    )
    helmert_solution, lstsq_solution = solve_helmert_lstsq(As, bs)
    helmert_transformation = compute_helmert_matrix(*helmert_solution) @ ransac_transformation

    #
    # Refine the transformatin matrix with Iterative Closest Point (ICP)
    #

    icp_registration = o3dy.registration.registration_icp(
        pcd_meshroom,
        pcd_airsim,
        MAX_CORRESPONDENCE_DISTANCE,
        helmert_transformation,
        ESTIMATION_METHOD,
        ICP_CRITERIA,
    )
    icp_transformation = icp_registration.transformation

    #
    # Evaluate registration results
    #

    init_eval = evaluate_registration(pcd_meshroom, pcd_airsim)
    ransac_eval = evaluate_registration(pcd_meshroom, pcd_airsim, ransac_transformation)
    helmert_eval = evaluate_registration(pcd_meshroom, pcd_airsim, helmert_transformation)
    icp_eval = evaluate_registration(pcd_meshroom, pcd_airsim, icp_transformation)

    results_string = [
        (
            "\nInlier RMSE, Fitness, and Number of Correspondences (with "
            f"{MAX_CORRESPONDENCE_DISTANCE} maximum points-pair distance)"
        ),
        f"- Initial: {init_eval.inlier_rmse:.4f} ( {(100 * init_eval.fitness):.2f}% =  {len(init_eval.correspondence_set)} out of {len(pcd_airsim.points)} points)",
        f"+ RANSAC:  {ransac_eval.inlier_rmse:.4f} ({(100 * ransac_eval.fitness):.2f}% = {len(ransac_eval.correspondence_set)} out of {len(pcd_airsim.points)} points)",
        f"+ Helmert: {helmert_eval.inlier_rmse:.4f} ({(100 * helmert_eval.fitness):.2f}% = {len(helmert_eval.correspondence_set)} out of {len(pcd_airsim.points)} points)",
        f"+ ICP:     {icp_eval.inlier_rmse:.4f} ({(100 * icp_eval.fitness):.2f}% = {len(icp_eval.correspondence_set)} out of {len(pcd_airsim.points)} points)",
    ]

    print(*results_string, sep="\n")

    #
    # Save final "source to target" alignment transformation matrix
    #

    np.savetxt(
        "align_meshroom_to_airsim.txt" if args.output is None else args.output,
        icp_transformation,
        header="\n".join(
            results_string
            + [
                "",
                f"airsim_rec = {os.path.abspath(args.airsim_rec)}",
                f"meshroom_sfm = {os.path.abspath(args.meshroom_sfm)}",
                "",
                f"{RANSAC_N = }",
                f"{MAX_CORRESPONDENCE_DISTANCE = }",
                f"{ESTIMATION_METHOD = }",
                f"{RANSAC_CRITERIA = }",
                f"{ICP_CRITERIA = }",
                "",
            ]
        ),
    )


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Given the reconstructed camera trajectories from Meshroom's cameras.sfm"
        " and their ground-truth positions from AirSim's *rec.txt, the matrix that aligns"
        " Meshroom to AirSim is estimated by computing a Helmert transformation, followed"
        " by ICP (Iterative Closest Point) with scaling to refine it."
    )

    parser.add_argument("meshroom_sfm", type=str, help="Path to Meshroom .sfm file (source)")
    parser.add_argument("airsim_rec", type=str, help="Path to AirSim .txt file (target)")
    parser.add_argument("--output", type=str, help="Path to output NumPy matrix .txt")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    assert os.path.isfile(args.meshroom_sfm), f"Invalid file path: '{args.meshroom_sfm}'"
    assert os.path.isfile(args.airsim_rec), f"Invalid file path: '{args.airsim_rec}'"
    if not args.meshroom_sfm.endswith(".sfm"):
        if args.airsim_rec.endswith(".sfm"):
            print("Swapping argument order based on file extensions\n")
            args.meshroom_sfm, args.airsim_rec = args.airsim_rec, args.meshroom_sfm
        else:
            assert False, f"Expected sfm file as a .sfm: '{args.meshroom_sfm}'"
    assert args.airsim_rec.endswith(".txt"), f"Expected rec file as a .txt: '{args.airsim_rec}'"

    main(args)

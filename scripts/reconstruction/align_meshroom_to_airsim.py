import os
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


def check_paths_are_valid(args):
    assert os.path.isfile(args.meshroom_sfm), f"Invalid file path: '{args.meshroom_sfm}'"
    assert os.path.isfile(args.airsim_rec), f"Invalid file path: '{args.airsim_rec}'"
    if not args.meshroom_sfm.endswith(".sfm"):
        if args.airsim_rec.endswith(".sfm"):
            print("Swapping argument order based on file extensions\n")
            args.meshroom_sfm, args.airsim_rec = args.airsim_rec, args.meshroom_sfm
        else:
            assert False, f"Expected sfm file as a .sfm: '{args.meshroom_sfm}'"
    assert args.airsim_rec.endswith(".txt"), f"Expected rec file as a .txt: '{args.airsim_rec}'"


def print_helmert_solution(tx, ty, tz, S, rx, ry, rz):
    print(f"{[tx, ty, tz] = }")
    print(f"{[rx, ry, rz] = }")
    print(f"{S = }")


def print_lstsq_solution(As, bs, xs, residuals, rank, singular):
    As, bs, xs = As.squeeze(), bs.squeeze(), xs.squeeze()
    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")

    print(f"{xs = }")
    print(f"{As @ xs = }")
    print(f"{bs = }")

    print(f"{residuals = }")
    print(f"{rank = }")
    print(f"{singular = }")


MAX_CORRESPONDENCE_DISTANCE = 0.5  # maximum correspondence points-pair distance

ICP_WITH_SCALING = True

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


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    check_paths_are_valid(args)
    airsim_records = AirSimRecord.dict_from(args.airsim_rec)
    meshroom_views, meshroom_poses = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.meshroom_sfm)
    )

    # NOTE do *not* assume that the points we read from Meshroom's cameras.sfm
    # come in the same order as AirSim's *rec.txt, they *must* be paired based
    # on the image filenames: views[id]["path"] <-> ImageFile (NOTE ignore dir)
    def path_matches_image_file(meshroom_path, airsim_image_file):
        meshroom_name = os.path.basename(meshroom_path)
        airsim_name = os.path.basename(airsim_image_file)
        return meshroom_name.endswith(airsim_name)
        # return meshroom_name == airsim_name

    record_view_pose_triples = []
    for image_file, record in airsim_records.items():
        view_pose_matches = [
            (view, meshroom_poses[view.pose_id])
            for view in meshroom_views.values()
            if path_matches_image_file(view.path, image_file)
        ]
        assert len(view_pose_matches) == 1, f"'{image_file}'': {view_pose_matches}"
        [(view, pose)] = view_pose_matches
        record_view_pose_triples.append((record, view, pose))

    # print(f"{airsim_records = }")
    # print(f"{meshroom_views = }")
    # print(f"{meshroom_poses = }")
    # print(f"{record_view_pose_triples = }")

    airsim_points, meshroom_points = zip(
        *[
            (record.position.to_numpy_array(), np.array(pose.center, dtype=np.float32))
            for record, _, pose in record_view_pose_triples
        ]
    )
    pcd_airsim = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(airsim_points)))
    pcd_meshroom = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(meshroom_points)))

    #
    # Helmert transformation
    #

    # TODO limit the number of points that are used in here (e.g. sample them with RANSAC)
    As, bs = compute_helmert_A_b(
        xs_source=np.asarray(pcd_meshroom.points), xs_target=np.asarray(pcd_airsim.points)
    )

    helmert_solution, lstsq_solution = solve_helmert_lstsq(As, bs)
    [tx, ty, tz], S, [rx, ry, rz] = helmert_solution
    xs, residuals, rank, singular = lstsq_solution

    align_meshroom_to_airsim = compute_helmert_matrix(tx, ty, tz, S, rx, ry, rz)

    # print(f"{align_meshroom_to_airsim = }")
    # print_lstsq_solution(As, bs, xs, residuals, rank, singular)
    # print_helmert_solution(tx, ty, tz, S, rx, ry, rz)

    #
    # Iterative Closest Point (ICP)
    #

    icp_registration = o3dy.registration.registration_icp(
        pcd_meshroom,
        pcd_airsim,
        MAX_CORRESPONDENCE_DISTANCE,
        align_meshroom_to_airsim,
        o3dy.registration.TransformationEstimationPointToPoint(with_scaling=ICP_WITH_SCALING),
        ICP_CRITERIA,
    )

    align_meshroom_to_airsim_icp = icp_registration.transformation

    print(f"{icp_registration = }")

    #
    # Evaluate registration results
    #

    initial_evaluation = evaluate_registration(pcd_meshroom, pcd_airsim)
    helmert_evaluation = evaluate_registration(pcd_meshroom, pcd_airsim, align_meshroom_to_airsim)
    icp_evaluation = evaluate_registration(pcd_meshroom, pcd_airsim, align_meshroom_to_airsim_icp)

    print(f"{initial_evaluation = }")
    print(f"{helmert_evaluation = }")
    print(f"{icp_evaluation = }")

    #
    # Save source -> target alignment transformation matrix
    #

    # np.savetxt("align_meshroom_to_airsim.txt", align_meshroom_to_airsim)  # XXX
    # np.savetxt("align_meshroom_to_airsim_icp.txt", align_meshroom_to_airsim_icp)  # XXX


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

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

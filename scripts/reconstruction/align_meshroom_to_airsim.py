import argparse
import os

import numpy as np
import open3d as o3d

import open3dy as o3dy
from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser, MeshroomTransform, MeshroomQuaternion

from convert_to_ply import np_points_from_airsim_rec, np_points_from_cameras_sfm
from solve_helmert_transform_lstsq import (
    compute_helmert_A_b,
    solve_helmert_lstsq,
    compute_helmert_matrix,
)


def check_paths_are_valid(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.meshroom_sfm), f"Invalid file path: '{args.meshroom_sfm}'"
    assert os.path.isfile(args.airsim_rec), f"Invalid file path: '{args.airsim_rec}'"
    if not args.meshroom_sfm.endswith(".sfm"):
        if args.airsim_rec.endswith(".sfm"):
            print("Swapping argument order based on file extensions\n")
            args.meshroom_sfm, args.airsim_rec = args.airsim_rec, args.meshroom_sfm
        else:
            assert False, f"Expected sfm file as a .sfm: '{args.meshroom_sfm}'"
    assert args.airsim_rec.endswith(".txt"), f"Expected rec file as a .txt: '{args.airsim_rec}'"


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    check_paths_are_valid(args)
    airsim_records = AirSimRecord.dict_from(args.airsim_rec)
    meshroom_views, meshroom_poses = MeshroomParser.extract_views_and_poses(
        *MeshroomParser.parse_cameras(args.meshroom_sfm)
    )

    # XXX this doesn't work, see the NOTE below...
    #
    # with open(args.meshroom_sfm, "r") as f:
    #     lines = f.readlines()
    #     np_points = np_points_from_cameras_sfm(lines)
    #     pcd_meshroom = o3d.geometry.PointCloud(o3dy.v3d(np_points))
    #
    # with open(args.airsim_rec, "r") as f:
    #     lines = f.readlines()
    #     np_points = np_points_from_airsim_rec(lines)
    #     pcd_airsim = o3d.geometry.PointCloud(o3dy.v3d(np_points))

    # NOTE do *not* assume that the points we read from Meshroom's cameras.sfm
    # come in the same order as AirSim's *rec.txt, they *must* be paired based
    # on the image filenames: views[id]["path"] <-> ImageFile (NOTE ignore dir)
    def path_matches_image_file(meshroom_path, airsim_image_file):
        # return os.path.basename(meshroom_path) == os.path.basename(airsim_image_file)
        return os.path.basename(meshroom_path).endswith(os.path.basename(airsim_image_file))

    record_view_pose_triples = []
    for image_file, record in airsim_records.items():
        view_pose_matches = [
            (view, meshroom_poses[view.pose_id])
            for view in meshroom_views.values()
            if path_matches_image_file(view.path, image_file)
        ]

        if not view_pose_matches:
            print(f"No matches found for '{image_file}'")
            continue

        assert len(view_pose_matches) == 1, view_pose_matches
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

    # print(f"{airsim_points = }")
    # print(f"{meshroom_points = }")

    pcd_airsim = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(airsim_points)))
    pcd_meshroom = o3d.geometry.PointCloud(o3dy.v3d(np.asarray(meshroom_points)))

    # TODO use a sample of the points (e.g. with RANSAC)
    As, bs = compute_helmert_A_b(
        xs_source=np.asarray(pcd_meshroom.points), xs_target=np.asarray(pcd_airsim.points)
    )

    helmert_solution, lstsq_solution = solve_helmert_lstsq(As, bs)
    [tx, ty, tz], S, [rx, ry, rz] = helmert_solution
    xs, residuals, rank, singular = lstsq_solution

    align_meshroom_to_airsim = compute_helmert_matrix(tx, ty, tz, S, rx, ry, rz)  # source -> target

    xs = xs.squeeze()
    As = As.squeeze()
    bs = bs.squeeze()
    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")

    print(f"{xs = }")
    print(f"{As @ xs = }")
    print(f"{bs = }")

    # NOTE residuals is the squared Euclidean 2-norm for each column in `bs - As @ xs`
    print(f"{residuals = }")  # sums of squared residuals
    print(f"{rank = }")  # rank of matrix As
    print(f"{singular = }")  # singular values of As

    print(f"{[tx, ty, tz] = }")
    print(f"{[rx, ry, rz] = }")
    print(f"{S = }")

    print(f"{align_meshroom_to_airsim = }")
    np.savetxt("align_meshroom_to_airsim.txt", align_meshroom_to_airsim)


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

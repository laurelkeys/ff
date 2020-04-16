import os
import argparse
import numpy as np
import open3d as o3d

from tanksandtemples.plot import plot_graph
from tanksandtemples.evaluation import EvaluateHisto
from tanksandtemples.registration import trajectory_alignment, registration_vol_ds, registration_unif
from tanksandtemples.trajectory_io import read_trajectory


# ref.: https://github.com/intel-isl/TanksAndTemples/tree/master/python_toolbox/evaluation


def run_evaluation(
    scene, dTau,
    scene_dir, out_dir,
    ply_fname, traj_fname,
    gt_ply_fname, gt_traj_fname,
    crop_fname, alignment_fname,
):
    print(f"\n{'=' * 27}\nEvaluating {scene}\n{'=' * 27}")

    ply_fname = os.path.join(scene_dir, ply_fname)
    traj_fname = os.path.join(scene_dir, traj_fname)
    gt_ply_fname = os.path.join(scene_dir, gt_ply_fname)
    gt_traj_fname = os.path.join(scene_dir, gt_traj_fname)
    crop_fname = os.path.join(scene_dir, crop_fname)
    alignment_fname = os.path.join(scene_dir, alignment_fname)

    os.makedirs(out_dir, exist_ok=True)

    # Load reconstruction and according ground-truth
    print(ply_fname)
    pcd = o3d.io.read_point_cloud(ply_fname)
    print(gt_ply_fname)
    gt_pcd = o3d.io.read_point_cloud(gt_ply_fname)

    traj = read_trajectory(traj_fname) # generated .log file
    gt_traj = read_trajectory(gt_traj_fname) # reference .log file
    gt_trans = np.loadtxt(alignment_fname) # alignment matrix (<scene>_trans.txt)

    transformation = trajectory_alignment(scene, traj, gt_traj, gt_trans)

    # Refine alignment by using the actual ground-truth pointcloud
    vol = o3d.visualization.read_selection_polygon_volume(crop_fname)

    # Registration refinement in 3 iterations
    r2 = registration_vol_ds(pcd, gt_pcd,    transformation, vol,       dTau, dTau * 80, 20)
    r3 = registration_vol_ds(pcd, gt_pcd, r2.transformation, vol, dTau / 2.0, dTau * 20, 20)
    r  = registration_unif(  pcd, gt_pcd, r3.transformation, vol,   2 * dTau,        20)

    # Histograms and P/R/F1
    precision, recall, fscore, *histograms_data = EvaluateHisto(
        scene, out_dir, pcd, gt_pcd,  r.transformation, vol, dTau / 2.0, dTau, plot_stretch=5
    )

    print("==============================")
    print("evaluation result : %s" % scene)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")

    # Plotting
    edges_source, cum_source, edges_target, cum_target = histograms_data
    plot_graph(
        scene, out_dir, fscore,
        edges_source, cum_source, edges_target, cum_target,
        plot_stretch=5, dist_threshold=dTau,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "scene_dir", type=str,
        help="Path to a scene directory with the necessary files.",
    )
    parser.add_argument(
        "--out_dir", type=str, default="",
        help="Output directory, by default an 'evaluation' directory is created.",
    )
    parser.add_argument(
        "--dTau", type=float, default=0.01,
        help="Scene's tau value, used to downlsample large PCD to speed up alignment.",
    )
    parser.add_argument(
        "--traj_fname", type=str,
        help="Name of the reconstructed trajectory .log file.",
    )
    parser.add_argument(
        "--gt_traj_fname", type=str,
        help="Name of the ground-truth trajectory .log file.",
    )
    parser.add_argument(
        "--ply_fname", type=str,
        help="Name of the reconstruction .ply file.",
    )
    parser.add_argument(
        "--gt_ply_fname", type=str,
        help="Name of the ground-truth .ply file.",
    )
    parser.add_argument(
        "--crop_fname", type=str,
        help="Name of the scene's bounding box coordinates crop .json file.",
    )
    parser.add_argument(
        "--alignment_fname", type=str,
        help="Name of the alignment matrix as a .txt file.",
    )
    args = parser.parse_args()

    scene = os.path.basename(os.path.normpath(args.scene_dir))

    if args.traj_fname is None: args.traj_fname = f"{scene}_Meshroom.log"
    if args.gt_traj_fname is None: args.gt_traj_fname = f"{scene}.log"
    if args.ply_fname is None: args.ply_fname = f"{scene}_Meshroom.ply"
    if args.gt_ply_fname is None: args.gt_ply_fname = f"{scene}.ply"
    if args.crop_fname is None: args.crop_fname = f"{scene}.json"
    if args.alignment_fname is None: args.alignment_fname = f"{scene}_trans.txt"

    if args.out_dir.strip() == "":
        args.out_dir = os.path.join(
            os.path.dirname(args.scene_dir), "evaluation"
        )

    # for k, v in vars(args).items():
    #     print(f"{k}: {v}")
    # print()

    run_evaluation(
        scene=scene,
        scene_dir=args.scene_dir,
        out_dir=args.out_dir,
        dTau=args.dTau,
        traj_fname=args.traj_fname,
        gt_traj_fname=args.gt_traj_fname,
        ply_fname=args.ply_fname,
        gt_ply_fname=args.gt_ply_fname,
        crop_fname=args.crop_fname,
        alignment_fname=args.alignment_fname,
    )


# What we need:
# - <scene>.ply -- ground-truth pcd
# - <scene>.log -- ground-truth camera poses
# - <scene>_Meshroom.ply -- reconstructed pcd
# - <scene>_Meshroom.log -- reconstructed camera poses
# - <scene>.json -- bounding box coordinates (crop file)
# - <scene>_trans.txt -- alignment matrix (with ground-truth point cloud as the reference)
#
# Ground-truth .log files should be captured together with the photos, from AirSim itself
# Reconstructed trajectory .log files can be converted from Meshroom's cameras.sfm
# The alignment matrix can be copied from a MeshLab .mlp project
# Bounding box coordinates need to be generated through Open3d
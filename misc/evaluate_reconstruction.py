import os
import argparse

import numpy as np
import open3d as o3d

from plot import plot_graph
from config import scenes_tau_dict
from evaluation import EvaluateHisto
from trajectory_io import read_trajectory


def crop_pcd(pcd, crop_volume, trans=None):
    pcd_copy = copy.deepcopy(pcd)
    if trans is not None:
        pcd_copy.transform(trans)
    return crop_volume.crop_point_cloud(pcd_copy)


def uniform_downsample(pcd, max_points):
    n_points = len(pcd_crop.points)
    if n_points > max_points:
        every_k_points = int(round(n_points / max_points))
        return pcd_crop.uniform_down_sample(every_k_points)


def voxel_downsample(pcd, voxel_size=0.01):
    return pcd.voxel_down_sample(voxel_size)


def uniform_registration(
    source, target, init_trans, crop_volume, threshold, max_iter, max_size=None, verbose=True
):
    if max_size is not None:
        max_points = max_size / 4
    else:
        max_size, max_points = 16e6, 4e6

    if verbose:
        print("[Registration] threshold: %f, max_size: %f" % (threshold, max_size))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = uniform_downsample(crop_pcd(source, crop_volume, init_trans), max_points)
    t = uniform_downsample(crop_pcd(target, crop_volume), max_points)

    reg = o3d.registration.registration_icp(
        s, t, threshold, np.identity(4),
        o3d.registration.TransformationEstimationPointToPoint(True),  # with_scaling
        o3d.registration.ICPConvergenceCriteria(1e-6, max_iter),
    )

    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg


def voxel_registration(
    source, target, init_trans, crop_volume, threshold, max_iter, voxel_size, verbose=True
):
    if verbose:
        print("[Registration] threshold: %f, voxel_size: %f" % (threshold, voxel_size))
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = voxel_downsample(crop_pcd(source, crop_volume, init_trans), voxel_size)
    t = voxel_downsample(crop_pcd(target, crop_volume), voxel_size)

    reg = o3d.registration.registration_icp(
        s, t, threshold, np.identity(4),
        o3d.registration.TransformationEstimationPointToPoint(True),  # with_scaling
        o3d.registration.ICPConvergenceCriteria(1e-6, max_iter),
    )

    reg.transformation = np.matmul(reg.transformation, init_trans)
    return reg


def evaluate(
    scene_name,
    out_dir,
    dTau,
    gt_ply_path,  # Ground-truth (gt)
    gt_log_path,
    est_ply_path,  # Estimate (est) reconstruction
    est_log_path,
    alignment_txt_path,  # Transformation matrix to align 'est' with 'gt'
    crop_json_path,  # Area cropping for the 'gt' point cloud
    plot_stretch,
    map_file=None,
):
    # Load reconstruction and according ground-truth
    est_pcd = o3d.io.read_point_cloud(est_ply_path)  # "source"
    gt_pcd = o3d.io.read_point_cloud(gt_ply_path)  # "target"

    transform = np.loadtxt(alignment_txt_path)
    est_traj = read_trajectory(est_log_path)
    gt_traj = read_trajectory(gt_log_path)

    traj_transformation = trajectory_alignment(map_file, est_traj, gt_traj, transform)

    # Refine alignment by using the actual 'gt' and 'est' point clouds
    # Big pointclouds will be downsampled to 'dTau' to speed up alignment
    vol = o3d.visualization.read_selection_polygon_volume(crop_json_path)

    # Registration refinment in 3 iterations
    r2 = voxel_registration(
        est_pcd, gt_pcd, init_trans=traj_transformation,
        crop_volume=vol, threshold=dTau * 80, max_iter=20, voxel_size=dTau
    )
    r3 = voxel_registration(
        est_pcd, gt_pcd, init_trans=r2.transformation,
        crop_volume=vol, threshold=dTau * 20, max_iter=20, voxel_size=dTau / 2,
    )
    r = uniform_registration(
        est_pcd, gt_pcd, init_trans=r3.transformation,
        crop_volume=vol, threshold=2 * dTau, max_iter=20,
    )

    # Generate histograms and compute P/R/F1
    # [precision, recall, fscore, edges_source, cum_source, edges_target, cum_target]
    return EvaluateHisto(
        est_pcd,
        gt_pcd,
        trans=r.transformation,
        crop_volume=vol,
        voxel_size=dTau / 2,
        threshold=dTau,
        filename_mvs=out_dir,
        plot_stretch=plot_stretch,
        scene_name=scene_name,
    )


def run_evaluation(dataset_dir, traj_path, ply_path, out_dir, scene_dTau=None):
    scene = os.path.basename(os.path.normpath(dataset_dir))

    if scene_dTau is None and scene not in scenes_tau_dict:
        print(dataset_dir, scene)
        raise Exception("invalid dataset-dir, not in scenes_tau_dict")

    print("")
    print("===========================")
    print("Evaluating %s" % scene)
    print("===========================")

    dTau = scene_dTau if scene_dTau is not None else scenes_tau_dict[scene]
    # put the crop-file, the GT file, the COLMAP SfM log file and
    # the alignment of the according scene in a folder of
    # the same scene name in the dataset_dir
    colmap_ref_logfile = os.path.join(dataset_dir, scene + "_COLMAP_SfM.log")
    alignment = os.path.join(dataset_dir, scene + "_trans.txt")
    gt_filen = os.path.join(dataset_dir, scene + ".ply")
    cropfile = os.path.join(dataset_dir, scene + ".json")
    map_file = os.path.join(dataset_dir, scene + "_mapping_reference.txt")

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    print(ply_path)
    print(gt_filen)
    plot_stretch = 5

    [precision, recall, fscore, edges_source, cum_source, edges_target, cum_target] = evaluate(
        scene,
        out_dir,
        dTau,
        gt_ply_path=gt_filen,
        gt_log_path=colmap_ref_logfile,
        est_ply_path=ply_path,
        est_log_path=traj_path,
        alignment_txt_path=alignment,
        crop_json_path=cropfile,
        plot_stretch=plot_stretch,
        map_file=map_file,
    )

    print("==============================")
    print("evaluation result : %s" % scene)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")

    plot_graph(
        scene,
        fscore,
        dTau,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        plot_stretch,
        out_dir,
    )

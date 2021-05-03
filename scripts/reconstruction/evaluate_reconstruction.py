from __future__ import annotations

import os
import glob
import argparse

from enum import Enum
from typing import NamedTuple

import numpy as np
import open3d as o3d

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser

try:
    from include_in_path import include, FF_PROJECT_ROOT

    python_toolbox_path = [FF_PROJECT_ROOT, "vendor", "TanksAndTemples", "python_toolbox"]
    v1_tanksandtemples_path = [FF_PROJECT_ROOT, "misc", "v1", "reconstruction", "tanksandtemples"]

    # FIXME stop using the modified version from v1/, but also, don't use the different
    # files from python_toolbox/evaluation/, instead, use tanksandtemples_evaluator.py!
    # Check out eval_Cidadela_2021-01-09-18-27-07.py to see what changes are necessary.
    if False:
        include(*python_toolbox_path, "evaluation", "plot")
        include(*python_toolbox_path, "evaluation", "evaluation")
        include(*python_toolbox_path, "evaluation", "registration")
        include(*python_toolbox_path, "evaluation", "trajectory_io")
    else:
        include(*v1_tanksandtemples_path, "plot")
        include(*v1_tanksandtemples_path, "evaluation")
        include(*v1_tanksandtemples_path, "registration")
        include(*v1_tanksandtemples_path, "trajectory_io")

    from plot import plot_graph
    from evaluation import EvaluateHisto
    from registration import registration_unif, registration_vol_ds, trajectory_alignment
    from trajectory_io import read_trajectory

    include(*python_toolbox_path, "convert_to_logfile")
    from convert_to_logfile import quat2rotmat, write_SfM_log
except:
    raise


# FIXME turn these into args
DTAU = 0.01
SCENE = "Cidadela"
CONVERT_AIRSIM_OUT = "tanksandtemples.rec.log"
CONVERT_MESHROOM_OUT = "tanksandtemples.sfm.log"
EVALUATION_OUT_FOLDER = "tanksandtemples.eval"


class TanksAndTemples:
    # class Result(NamedTuple):

    # NOTE the .log format used by TanksAndTemples  is not the same as the one used by TartanAir
    class LogCameraPose(NamedTuple):
        id: int
        image_path: str
        log_matrix: np.ndarray


###############################################################################
###############################################################################


class Method(Enum):
    Meshroom = 0
    AirSim = 1


def convert_to_log(
    from_method, image_folder, logfile_out, cameras_sfm_path=None, airsim_rec_path=None
):
    input_images = glob.glob(os.path.join(image_folder, f"*.png"))
    input_images.sort()
    n_of_images = len(input_images)

    T, i_map = [], []
    TF, i_mapF = [], []

    camera_poses = []

    def transform(rotation, center):
        # homogeneous transformation matrix
        transformation = np.identity(4)
        transformation[:3, :3] = rotation
        transformation[:3, 3] = center
        return transformation

    if from_method == Method.Meshroom:
        views, poses = MeshroomParser.parse_cameras(cameras_sfm_path)
        views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)
        for id, pose in poses_dict.items():
            # FIXME triple-check this (shouldn't we use MeshroomTransform.rotation?)
            # 3x3 (column-major) rotation matrix
            rotation = np.array(pose.rotation).reshape((3, 3))
            rotation[:, 1:] *= -1  # https://colmap.github.io/format.html#images-txt
            # camera center in world coordinates
            center = np.array(pose.center)
            camera_poses.append(
                TanksAndTemples.LogCameraPose(id, views_dict[id].path, transform(rotation, center))
            )

    elif from_method == Method.AirSim:
        for record in AirSimRecord.list_from(airsim_rec_path):
            # FIXME triple-check this (note that quat2rotmat expects wxyz, not xyzw)
            rotation = quat2rotmat(
                np.array(
                    [
                        record.orientation.w_val,
                        record.orientation.x_val,
                        record.orientation.y_val,
                        record.orientation.z_val,
                    ]
                )
            )
            # rotation[:, 1:] *= -1
            center = record.position.to_numpy_array()
            camera_poses.append(
                TanksAndTemples.LogCameraPose(
                    record.time_stamp, record.image_file, transform(rotation, center)
                )
            )

    for pose in camera_poses:
        A = np.matrix(pose.log_matrix)
        T.append(A.I)
        image_name = os.path.basename(pose.image_path)
        matching = [i for i, s in enumerate(input_images) if image_name in s]
        i_map.append([pose.id, matching[0], 0])

    for k in range(n_of_images):
        try:
            # find the k-th view id
            view_id = [i for i, item in enumerate(i_map) if k == item[1]][0]
            i_mapF.append(np.array([k, k, 0], dtype="int"))
            TF.append(T[view_id])
        except IndexError:
            # assign the identity matrix to the k-th view id
            # as the log file needs an entry for every image
            i_mapF.append(np.array([k, -1, 0], dtype="int"))
            TF.append(np.identity(4))

    write_SfM_log(TF, i_mapF, logfile_out)


def convert_meshroom_to_log(cameras_sfm_path, image_folder):
    convert_to_log(
        Method.Meshroom,
        image_folder,
        logfile_out=CONVERT_MESHROOM_OUT,
        cameras_sfm_path=cameras_sfm_path,
        airsim_rec_path=None,
    )


def convert_airsim_to_log(airsim_rec_path, image_folder):
    convert_to_log(
        Method.AirSim,
        image_folder,
        logfile_out=CONVERT_AIRSIM_OUT,
        cameras_sfm_path=None,
        airsim_rec_path=airsim_rec_path,
    )


###############################################################################
###############################################################################


def evaluate(
    airsim_traj_path,
    meshroom_traj_path,
    airsim_ply_path,
    meshroom_ply_path,
    crop_bbox_path,
    alignment_matrix_path,
):
    assert os.path.isfile(airsim_traj_path), f"File not found: '{airsim_traj_path}'"
    assert os.path.isfile(meshroom_traj_path), f"File not found: '{meshroom_traj_path}'"
    assert os.path.isfile(airsim_ply_path), f"File not found: '{airsim_ply_path}'"
    assert os.path.isfile(meshroom_ply_path), f"File not found: '{meshroom_ply_path}'"
    assert os.path.isfile(crop_bbox_path), f"File not found: '{crop_bbox_path}'"
    assert os.path.isfile(alignment_matrix_path), f"File not found: '{alignment_matrix_path}'"

    # TODO update TanksAndTemples's python_toolbox/evaluation/run.py and move
    # the main "evaluation portion" of the code to the TanksAndTemples class.
    os.makedirs(EVALUATION_OUT_FOLDER, exist_ok=False)

    # Load reconstruction and according ground-truth
    print(meshroom_ply_path)
    pcd = o3d.io.read_point_cloud(meshroom_ply_path)
    print(airsim_ply_path)
    gt_pcd = o3d.io.read_point_cloud(airsim_ply_path)

    traj = read_trajectory(meshroom_traj_path)  # generated .log file
    gt_traj = read_trajectory(airsim_traj_path)  # reference .log file
    gt_trans = np.loadtxt(alignment_matrix_path)  # alignment matrix (<scene>_trans.txt)

    transformation = trajectory_alignment(None, traj, gt_traj, gt_trans)

    # Refine alignment by using the actual ground-truth pointcloud
    vol = o3d.visualization.read_selection_polygon_volume(crop_bbox_path)

    # Registration refinement in 3 iterations
    r2 = registration_vol_ds(pcd, gt_pcd, transformation, vol, DTAU, DTAU * 80, 20)
    r3 = registration_vol_ds(pcd, gt_pcd, r2.transformation, vol, DTAU / 2, DTAU * 20, 20)
    r = registration_unif(pcd, gt_pcd, r3.transformation, vol, 2 * DTAU, 20)

    # Histograms and P/R/F1
    precision, recall, fscore, *histograms_data = EvaluateHisto(
        SCENE,
        filename_mvs=EVALUATION_OUT_FOLDER,
        source=pcd,
        target=gt_pcd,
        trans=r.transformation,
        crop_volume=vol,
        voxel_size=DTAU / 2,
        threshold=DTAU,
        plot_stretch=5,
    )

    # XXX ^^^^ move to a specific function

    print("==============================")
    print("evaluation result : %s" % SCENE)
    print("==============================")
    print("distance tau : %.3f" % DTAU)
    print("precision : %.4f" % precision)
    print("recall : %.4f" % recall)
    print("f-score : %.4f" % fscore)
    print("==============================")

    # Plotting
    edges_source, cum_source, edges_target, cum_target = histograms_data
    plot_graph(
        SCENE,
        mvs_outpath=EVALUATION_OUT_FOLDER,
        fscore=fscore,
        edges_source=edges_source,
        cum_source=cum_source,
        edges_target=edges_target,
        cum_target=cum_target,
        plot_stretch=5,
        dist_threshold=DTAU,
    )


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")

    parser.add_argument(
        "--image_folder", type=str, help="Path to the folder containing the input images"
    )
    parser.add_argument(
        "--convert_meshroom",
        "-cm",
        nargs=1,
        metavar="CAMERAS_SFM_PATH",  # NOTE don't use new_cameras.sfm!
        help="Convert Meshroom's cameras.sfm to TanksAndTemples .log format",
    )
    parser.add_argument(
        "--convert_airsim",
        "-ca",
        nargs=1,
        metavar="AIRSIM_REC_PATH",
        help="Convert AirSim's airsim_rec.txt to TanksAndTemples .log format",
    )

    parser.add_argument(
        "--eval",
        action="store_true",
        help="Compare (AirSim) ground-truth to (Meshroom) estimate reconstruction .log files",
    )
    parser.add_argument("--log", nargs=2, metavar=("GT_TRAJECTORY_PATH", "EST_TRAJECTORY_PATH"))
    parser.add_argument("--ply", nargs=2, metavar=("GT_PLY_PATH", "EST_PLY_PATH"))
    parser.add_argument("--bbox", type=str, help="Path to the JSON crop file")
    parser.add_argument("--matrix", type=str, help="Path to the TXT alignment matrix file")

    args = parser.parse_args()

    # NOTE the .log format used by TanksAndTemples (http://redwood-data.org/indoor/fileformat.html)
    # is not the same as the one used by TartanAir (https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)

    if args.convert_meshroom is not None:
        assert os.path.isdir(args.image_folder)
        (cameras_sfm_path,) = args.convert_meshroom
        convert_meshroom_to_log(cameras_sfm_path, args.image_folder)

    if args.convert_airsim is not None:
        assert os.path.isdir(args.image_folder)
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path, args.image_folder)

    if args.eval:
        gt_traj_path, est_traj_path = args.log
        gt_ply_path, est_ply_path = args.ply
        evaluate(
            airsim_traj_path=gt_traj_path,
            meshroom_traj_path=est_traj_path,
            airsim_ply_path=gt_ply_path,
            meshroom_ply_path=est_ply_path,
            crop_bbox_path=args.bbox,
            alignment_matrix_path=args.matrix,
        )


###############################################################################
###############################################################################

from __future__ import annotations

import os
import glob
import argparse

from enum import Enum
from typing import NamedTuple

import numpy as np

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser

try:
    from include_in_path import include
except:
    pass
finally:
    include("..", "..", "vendor", "TanksAndTemples", "python_toolbox", "convert_to_logfile")
    from convert_to_logfile import quat2rotmat, write_SfM_log


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
        logfile_out="tanksandtemples.sfm.log",
        cameras_sfm_path=cameras_sfm_path,
        airsim_rec_path=None,
    )


def convert_airsim_to_log(airsim_rec_path, image_folder):
    convert_to_log(
        Method.AirSim,
        image_folder,
        logfile_out="tanksandtemples.rec.log",
        cameras_sfm_path=None,
        airsim_rec_path=airsim_rec_path,
    )


###############################################################################
###############################################################################


def evaluate(airsim_traj_path, meshroom_traj_path):
    assert os.path.isfile(airsim_traj_path), f"File not found: '{airsim_traj_path}'"
    assert os.path.isfile(meshroom_traj_path), f"File not found: '{meshroom_traj_path}'"

    pass


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("image_folder", help="Path to the folder containing the input images")
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
    parser.add_argument("--log", nargs=2, metavar=("GT_TRAJECTORY_PATH", "EST_TRAJECTORY_PATH"))
    parser.add_argument("--ply", nargs=2, metavar=("GT_PLY_PATH", "EST_PLY_PATH"))
    parser.add_argument("--bbox", type=str, help="Path to the JSON crop file")
    parser.add_argument("--matrix", type=str, help="Path to the TXT alignment matrix file")
    args = parser.parse_args()

    # NOTE the .log format used by TanksAndTemples (http://redwood-data.org/indoor/fileformat.html)
    # is not the same as the one used by TartanAir (https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)

    assert os.path.isdir(args.image_folder)

    if args.convert_meshroom is not None:
        (cameras_sfm_path,) = args.convert_meshroom
        convert_meshroom_to_log(cameras_sfm_path, args.image_folder)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path, args.image_folder)


###############################################################################
###############################################################################

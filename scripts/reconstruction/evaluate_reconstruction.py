from __future__ import annotations

import os
import glob
import json
import argparse

from enum import Enum
from typing import Tuple, Optional, NamedTuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser, MeshroomTransform

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
        pose_id: int
        image_path: str
        log_matrix: np.ndarray


###############################################################################
###############################################################################


FORMATP = "png"


class Method(Enum):
    Meshroom = 0
    AirSim = 1


def convert_to_log(
    from_method, image_folder, logfile_out, cameras_sfm_path=None, airsim_rec_path=None
):
    input_images = glob.glob(os.path.join(image_folder, f"*.{FORMATP}"))
    input_images.sort()
    n_of_images = len(input_images)

    T, i_map = [], []
    TF, i_mapF = [], []

    # def inv_A(r, translation):
    #     w = np.zeros((4, 4))
    #     w[3, 3] = 1
    #     w[:3, :3] = r
    #     w[:3, 3] = translation
    #     A = np.matrix(w)
    #     return A.I

    camera_poses = []

    if from_method == Method.Meshroom:
        views = {}
        with open(cameras_sfm_path, "r") as sfm_file:
            sfm_data = json.load(sfm_file)

            for view in sfm_data["views"]:
                views[view["poseId"]] = view["path"]  # NOTE equal to the 'viewId'

            for camera_pose in sfm_data["poses"]:
                pose_id = camera_pose["poseId"]
                pose_transform = camera_pose["pose"]["transform"]

                # 3x3 (column-major) rotation matrix
                rotation = np.array([float(_) for _ in pose_transform["rotation"]]).reshape((3, 3))
                rotation[:, 1:] *= -1  # ref.: [2]

                # camera center in world coordinates
                center = np.array([float(_) for _ in pose_transform["center"]])

                # homogeneous transformation matrix
                mat = np.identity(4)
                mat[:3, :3] = rotation
                mat[:3, 3] = center

                camera_poses.append(TanksAndTemples.LogCameraPose(pose_id, views[pose_id], mat))

    elif from_method == Method.AirSim:
        for entry in pd.read_csv(airsim_rec_path, delim_whitespace=True).itertuples():
            timestamp = entry.TimeStamp
            pos = np.array([entry.POS_X, entry.POS_Y, entry.POS_Z])  # (x, y, z)
            orien = np.array(
                [entry.Q_W, entry.Q_X, entry.Q_Y, entry.Q_Z]
            )  # (w, x, y, z) quaternion
            imagefile = entry.ImageFile  # name of the corresponding image file

            # FIXME triple-check this
            # rotation = quaternion.as_rotation_matrix(orien)
            rotation = quat2rotmat(orien)
            # rotation[:, 1:] *= -1
            center = pos
            mat = np.identity(4)
            mat[:3, :3] = rotation
            mat[:3, 3] = center
            camera_poses.append(TanksAndTemples.LogCameraPose(timestamp, imagefile, mat))

    # ...

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
        logfile_out="test",
        cameras_sfm_path=cameras_sfm_path,
        airsim_rec_path=None,
    )


def convert_airsim_to_log(airsim_rec_path, image_folder):
    convert_to_log(
        Method.AirSim,
        image_folder,
        logfile_out="test",
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

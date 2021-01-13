from __future__ import annotations

import os
import argparse

from typing import Tuple, Optional, NamedTuple

import numpy as np
import matplotlib.pyplot as plt

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser, MeshroomTransform

# try:
#     from include_in_path import include
# except:
#     pass
# finally:
#     include("..", "..", "vendor", "TanksAndTemples", "python_toolbox", "evaluation", "evaluation")


class TanksAndTemples:
    # class Result(NamedTuple):

    pass


###############################################################################
###############################################################################


def make_record_line(timestamp, position, orientation, as_string=True):
    """ `timestamp tx ty tz qx qy qz qw`, where:
        - `timestamp`: number of seconds since the Unix epoch
        - `tx ty tz`: position of the camera's optical center
        - `qx qy qz qw`: orientation of the camera's optical center (as a unit quaternion)

        Note: position and orientation values are given with respect to the world origin,
        as defined by the motion capture system.
    """
    tx, ty, tz = position
    qx, qy, qz, qw = orientation
    return (
        f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}"
        if as_string
        else (timestamp, tx, ty, tz, qx, qy, qz, qw)
    )


###############################################################################
###############################################################################


def convert_meshroom_to_log(cameras_sfm_path):
    pass


###############################################################################
###############################################################################


def convert_airsim_to_log(airsim_rec_path):
    pass


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
    parser.add_argument("--trans", type=str, help="Path to the TXT alignment matrix file")
    args = parser.parse_args()

    # NOTE the .log format used by TanksAndTemples (http://redwood-data.org/indoor/fileformat.html)
    # is not the same as the one used by TartanAir (https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)

    if args.convert_meshroom is not None:
        (cameras_sfm_path,) = args.convert_meshroom
        convert_meshroom_to_log(cameras_sfm_path)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path)


###############################################################################
###############################################################################

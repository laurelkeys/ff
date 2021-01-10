from __future__ import annotations

import os
import argparse

from typing import NamedTuple

import numpy as np
import airsim

from ie.meshroomy import MeshroomParser, MeshroomTransform

try:
    from include_in_path import include
except:
    pass
finally:
    include("..", "..", "vendor", "tartanair_tools")
    from tartanair_tools.evaluation.tartanair_evaluator import TartanAirEvaluator


# FIXME I didn't retest this file after moving it into scripts/
# NOTE ATE is well-suited for measuring the performance of visual SLAM systems, in contrast,
#      RPE is well-suited for measuring the drift of visual odometry systems (e.g. the drift per second)


class TartanAir:
    class Result(NamedTuple):
        ate_score: float
        rpe_score: float
        kitti_score: float
        gt_aligned: np.ndarray
        est_aligned: np.ndarray

    @staticmethod
    def evaluate_trajectory(
        gt_traj: np.ndarray, est_traj: np.ndarray, scale: bool = True
    ) -> TartanAir.Result:
        result = TartanAirEvaluator().evaluate_one_trajectory(
            gt_traj,
            est_traj,
            scale,  # NOTE True for monocular track, False for stereo track
            kittitype=False,
        )
        return TartanAir.Result(
            result["ate_score"],
            result["rpe_score"],
            result["kitti_score"],
            result["gt_aligned"],
            result["est_aligned"],
        )

    @staticmethod
    def trajectory_from_file(traj_path: str, skip_header: bool = False) -> np.ndarray:
        if skip_header:
            return np.loadtxt(traj_path, skiprows=1)
        return np.loadtxt(traj_path)

    @staticmethod
    def evaluate_trajectory_files(
        gt_traj_path: str,
        est_traj_path: str,
        scale: bool = True,
        gt_skip_header: bool = False,
        est_skip_header: bool = False,
    ) -> TartanAir.Result:
        return TartanAir.evaluate_trajectory(
            gt_traj=TartanAir.trajectory_from_file(gt_traj_path, gt_skip_header),
            est_traj=TartanAir.trajectory_from_file(est_traj_path, est_skip_header),
            scale=scale,
        )


###############################################################################
###############################################################################


def make_record_line(timestamp, position, orientation, as_string=True):
    """`timestamp tx ty tz qx qy qz qw`, where:
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


def convert_meshroom_to_log(cameras_fpath):
    assert os.path.isfile(cameras_fpath), f"File not found: '{cameras_fpath}'"

    views, poses = MeshroomParser.parse_cameras(cameras_fpath)
    views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

    record_lines = []
    for _view_id, view in views_dict.items():
        timestamp = os.path.splitext(os.path.basename(view.path))[0].split("_")[1]  # HACK

        pose = poses_dict[view.pose_id]
        position = MeshroomTransform.translation(pose.center)
        orientation = MeshroomTransform.rotation(pose.rotation, as_quaternion=True)

        line_str = make_record_line(timestamp, position, orientation)
        record_lines.append((timestamp, line_str))  # store a tuple

    # sort by timestamp and filter out the first element in the tuple
    meshroom_record = ["timestamp tx ty tz qx qy qz qw"]
    meshroom_record.extend([_[1] for _ in sorted(record_lines)])
    print("\n".join(meshroom_record))


###############################################################################
###############################################################################


def evaluate(airsim_traj_fpath, meshroom_traj_fpath):
    assert os.path.isfile(airsim_traj_fpath), f"File not found: '{airsim_traj_fpath}'"
    assert os.path.isfile(meshroom_traj_fpath), f"File not found: '{meshroom_traj_fpath}'"

    # with open(airsim_traj_fpath) as f:
    #     airsim_record = [line.rstrip() for line in f]

    # with open(meshroom_traj_fpath) as f:
    #     meshroom_record = [line.rstrip() for line in f]

    try:
        print(
            Vendor.TartanAir.evaluate(
                Vendor.TartanAir.traj_from_file(airsim_traj_fpath),
                Vendor.TartanAir.traj_from_file(meshroom_traj_fpath),
            )
        )
    except ValueError:
        print(
            Vendor.TartanAir.evaluate(
                Vendor.TartanAir.traj_from_file(airsim_traj_fpath, True),
                Vendor.TartanAir.traj_from_file(meshroom_traj_fpath, True),
            )
        )


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--convert",
        nargs=1,
        metavar="MESHROOM_CAMERAS_FPATH",
        help="Convert Meshroom's cameras.json to .log format",
    )
    parser.add_argument(
        "--eval",
        nargs=2,
        metavar=("GT_TRAJECTORY_FPATH", "EST_TRAJECTORY_FPATH"),
        help="Compare (AirSim) ground-truth to (Meshroom) estimate reconstruction .log files",
    )
    args = parser.parse_args()

    if args.convert is not None:
        convert_meshroom_to_log(args.convert[0])

    if args.eval is not None:
        gt_traj_fpath, est_traj_fpath = args.eval
        evaluate(airsim_traj_fpath=gt_traj_fpath, meshroom_traj_fpath=est_traj_fpath)


###############################################################################
###############################################################################

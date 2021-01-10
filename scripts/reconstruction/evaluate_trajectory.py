from __future__ import annotations

import os
import argparse

from typing import NamedTuple, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser, MeshroomTransform

try:
    from include_in_path import include
except:
    pass
finally:
    include("..", "..", "vendor", "tartanair_tools", "evaluation", "tartanair_evaluator")
    from tartanair_evaluator import TartanAirEvaluator


# FIXME I didn't retest this file after moving it into scripts/
# NOTE ATE is well-suited for measuring the performance of visual SLAM systems, in contrast,
#      RPE is well-suited for measuring the drift of visual odometry systems (e.g. the drift per second)


class TartanAir:
    class Result(NamedTuple):
        ate_score: float
        rpe_score: float
        kitti_score: Tuple[float, float]
        gt_aligned: np.ndarray
        est_aligned: np.ndarray

        # NOTE https://github.com/castacks/tartanvo/blob/de1d3c3b272c9cfb37380f86832f3143ee25f9b5/Datasets/utils.py#L230
        def plot(
            self, vis: bool = True, title: Optional[str] = None, savefigname: Optional[str] = None
        ) -> None:
            fig = plt.figure(figsize=(4, 4))
            cm = plt.cm.get_cmap("Spectral")

            plt.subplot(111)
            plt.plot(self.gt_aligned[:, 0], self.gt_aligned[:, 1], linestyle="dashed", c="k")
            plt.plot(self.est_aligned[:, 0], self.est_aligned[:, 1], c="#ff7f0e")

            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            plt.legend(["Ground Truth", "Estimate"])
            plt.title(title if title is not None else f"ATE {self.ate_score:.4f}")

            if savefigname is not None:
                plt.savefig(savefigname)
            if vis:
                plt.show()

            plt.close(fig)

    @staticmethod
    def evaluate_trajectory(
        gt_traj: np.ndarray, est_traj: np.ndarray, scale: bool = True
    ) -> TartanAir.Result:
        result = TartanAirEvaluator().evaluate_one_trajectory(
            gt_traj,
            est_traj,
            scale,  # True for monocular track, False for stereo track
            kittitype=False,
        )
        return TartanAir.Result(
            result["ate_score"],
            result["rpe_score"],
            result["kitti_score"],
            result["gt_aligned"],
            result["est_aligned"],
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


def convert_meshroom_to_log(cameras_sfm_path):
    assert os.path.isfile(cameras_sfm_path), f"File not found: '{cameras_sfm_path}'"

    views, poses = MeshroomParser.parse_cameras(cameras_sfm_path)
    views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

    record_lines = []
    for _view_id, view in views_dict.items():
        timestamp = os.path.splitext(os.path.basename(view.path))[0].split("_")[-1]  # HACK

        pose = poses_dict[view.pose_id]
        position = MeshroomTransform.translation(pose.center)
        orientation = MeshroomTransform.rotation(pose.rotation, as_xyzw_quaternion=True)

        line_str = make_record_line(timestamp, position, orientation)
        record_lines.append((timestamp, line_str))  # store a tuple

    # sort by timestamp and filter out the first element in the tuple
    meshroom_record = ["timestamp tx ty tz qx qy qz qw"]
    meshroom_record.extend([_[1] for _ in sorted(record_lines)])
    print("\n".join(meshroom_record))


###############################################################################
###############################################################################


# NOTE this is probably already done in some other old script...
# but it was probably harder to find it rather than reimplement it
def convert_airsim_to_log(airsim_rec_path):
    assert os.path.isfile(airsim_rec_path), f"File not found: '{airsim_rec_path}'"

    record_lines = []
    for record in AirSimRecord.list_from(airsim_rec_path):
        # timestamp = record.time_stamp
        # FIXME what convert_meshroom_to_log gets with HACK is not actually
        # the timestamp... but since it's only used for matching, this should work:
        timestamp = os.path.splitext(os.path.basename(record.image_file))[0].split("_")[-1]  # HACK

        position = record.position.to_numpy_array()
        orientation = record.orientation.to_numpy_array()
        assert np.isclose(orientation[3], record.orientation.w_val)

        line_str = make_record_line(timestamp, position, orientation)
        record_lines.append((timestamp, line_str))  # store a tuple

    # sort by timestamp and filter out the first element in the tuple
    airsim_record = ["timestamp tx ty tz qx qy qz qw"]
    airsim_record.extend([_[1] for _ in sorted(record_lines)])
    print("\n".join(airsim_record))


###############################################################################
###############################################################################


def evaluate(airsim_traj_path, meshroom_traj_path):
    assert os.path.isfile(airsim_traj_path), f"File not found: '{airsim_traj_path}'"
    assert os.path.isfile(meshroom_traj_path), f"File not found: '{meshroom_traj_path}'"

    result = TartanAir.evaluate_trajectory(
        # NOTE skip the header and the timestamp column, keeping tx ty tz qx qy qz qw
        gt_traj=np.loadtxt(airsim_traj_path, skiprows=1, usecols=(1, 2, 3, 4, 5, 6, 7)),
        est_traj=np.loadtxt(meshroom_traj_path, skiprows=1, usecols=(1, 2, 3, 4, 5, 6, 7)),
    )

    print(
        "==> ATE: %.4f,\t KITTI-R/t: %.4f, %.4f"
        % (result.ate_score, result.kitti_score[0], result.kitti_score[1])
    )

    result.plot()
    # TODO clean up this file and add these to argparse:
    # np.savetxt("est_aligned.txt", result.est_aligned)
    # np.savetxt("gt_aligned.txt", result.gt_aligned)

    print(result)


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--convert_meshroom",
        "-cm",
        nargs=1,
        metavar="CAMERAS_SFM_PATH",  # NOTE don't use new_cameras.sfm!
        help="Convert Meshroom's cameras.sfm to .log format",
    )
    parser.add_argument(
        "--convert_airsim",
        "-ca",
        nargs=1,
        metavar="AIRSIM_REC_PATH",
        help="Convert AirSim's airsim_rec.txt to .log format",
    )
    parser.add_argument(
        "--eval",
        nargs=2,
        metavar=("GT_TRAJECTORY_PATH", "EST_TRAJECTORY_PATH"),
        help="Compare (AirSim) ground-truth to (Meshroom) estimate reconstruction .log files",
    )
    args = parser.parse_args()

    if args.convert_meshroom is not None:
        (cameras_sfm_path,) = args.convert_meshroom
        convert_meshroom_to_log(cameras_sfm_path)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path)

    if args.eval is not None:
        gt_traj_path, est_traj_path = args.eval
        evaluate(airsim_traj_path=gt_traj_path, meshroom_traj_path=est_traj_path)


###############################################################################
###############################################################################

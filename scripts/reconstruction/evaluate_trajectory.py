from __future__ import annotations

import os
import argparse

from typing import Tuple, Optional, NamedTuple

import numpy as np
import matplotlib.pyplot as plt

from ie.airsimy import AirSimRecord
from ie.meshroomy import MeshroomParser, MeshroomTransform

try:
    from include_in_path import include, FF_PROJECT_ROOT

    include(FF_PROJECT_ROOT, "vendor", "tartanair_tools", "evaluation", "tartanair_evaluator")
    from tartanair_evaluator import TartanAirEvaluator
except:
    raise


DEBUG_ATE = True


class TartanAir:
    # NOTE ATE is well-suited for measuring the performance of visual SLAM systems, in contrast,
    #      RPE is well-suited for measuring the drift of visual odometry systems (e.g. the drift per second)
    class Result(NamedTuple):
        ate_score: float
        rpe_score: Tuple[float, float]
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
        result = TartanAirEvaluator.evaluate_one_trajectory(
            gt_traj,
            est_traj,
            scale,  # True for monocular track, False for stereo track
            kittitype=False,
        )

        if DEBUG_ATE:
            np_indent = lambda array, string: str(array).replace("\n", "\n" + " " * len(string))
            print(f"Scale: {result['scale']}")
            print(f"ATE scale: {result['ate_scale']}")
            print(f"    T: {np_indent(result['ate_T'], '    T: ')}")
            print(f"    rot: {np_indent(result['ate_rot'], '    rot: ')}")
            print(f"    trans: {np_indent(result['ate_trans'], '    trans: ')}")

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
    assert os.path.isfile(cameras_sfm_path), f"File not found: '{cameras_sfm_path}'"

    views, poses = MeshroomParser.parse_cameras(cameras_sfm_path)
    views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

    record_lines = []
    for _, view in views_dict.items():
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
        # FIXME what convert_meshroom_to_log gets (see HACK in it) is not actually
        # the timestamp... but since it's only used for matching, this should work:
        timestamp = os.path.splitext(os.path.basename(record.image_file))[0].split("_")[-1]  # HACK
        # timestamp = record.time_stamp

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


def evaluate(gt_traj, est_traj):
    result = TartanAir.evaluate_trajectory(gt_traj, est_traj, scale=True)
    print(
        "\n==> ATE: %.4f,\tRPE-R/t: %.4f, %.4f,\tKITTI-R/t: %.4f, %.4f"
        % (
            result.ate_score,
            result.rpe_score[0],
            result.rpe_score[1],
            result.kitti_score[0],
            result.kitti_score[1],
        )
    )
    result.plot()
    print(result)
    return result.gt_aligned, result.est_aligned


def evaluate_files(gt_traj_path, est_traj_path, save_gt_aligned=False, save_est_aligned=False):
    assert os.path.isfile(gt_traj_path), f"File not found: '{gt_traj_path}'"
    assert os.path.isfile(est_traj_path), f"File not found: '{est_traj_path}'"

    gt_aligned, est_aligned = evaluate(
        # NOTE skip the header and the timestamp column, keeping tx ty tz qx qy qz qw
        gt_traj=np.loadtxt(gt_traj_path, skiprows=1, usecols=(1, 2, 3, 4, 5, 6, 7)),
        est_traj=np.loadtxt(est_traj_path, skiprows=1, usecols=(1, 2, 3, 4, 5, 6, 7)),
    )

    if save_gt_aligned:
        np.savetxt("gt_aligned.txt", gt_aligned)
    if save_est_aligned:
        np.savetxt("est_aligned.txt", est_aligned)


###############################################################################
###############################################################################


def main(args):
    if args.convert_meshroom is not None:
        (cameras_sfm_path,) = args.convert_meshroom
        convert_meshroom_to_log(cameras_sfm_path)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path)

    if args.eval is not None:
        airsim_traj_path, meshroom_traj_path = args.eval
        evaluate_files(
            gt_traj_path=airsim_traj_path,
            est_traj_path=meshroom_traj_path,
            save_est_aligned=args.save_est_aligned,
            save_gt_aligned=args.save_gt_aligned,
        )


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare an AirSim (ground-truth) to a Meshroom (estimate) trajectory"
        " by computing the absolute trajectory error (ATE) and relative pose error (RPE)."
    )

    parser.add_argument(
        "--convert_meshroom",
        "-m",
        nargs=1,
        metavar="cameras_sfm_path",  # NOTE don't use new_cameras.sfm!
        help="Convert Meshroom's cameras.sfm to .log format",
    )
    parser.add_argument(
        "--convert_airsim",
        "-a",
        nargs=1,
        metavar="airsim_rec_path",
        help="Convert AirSim's airsim_rec.txt to .log format",
    )

    parser.add_argument(
        "--eval",
        nargs=2,
        metavar=("gt_trajectory_path", "est_trajectory_path"),
        help="Compare (AirSim) ground-truth to (Meshroom) estimate trajectory .log files",
    )

    parser.add_argument("--save_est_aligned", "-sest", action="store_true")
    parser.add_argument("--save_gt_aligned", "-sgt", action="store_true")

    main(args=parser.parse_args())


###############################################################################
###############################################################################

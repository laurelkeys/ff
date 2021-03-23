from __future__ import annotations

import os
import argparse

import numpy as np

from evaluate_trajectory import evaluate, make_record_line, convert_airsim_to_log

try:
    from include_in_path import FF_PROJECT_ROOT, include

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_parse_traj")
    import uavmvs_parse_traj as uavmvs
except:
    raise


###############################################################################
###############################################################################


def convert_uavmvs_to_log(uavmvs_traj_path):
    assert os.path.isfile(uavmvs_traj_path), f"File not found: '{uavmvs_traj_path}'"
    assert os.path.splitext(uavmvs_traj_path)[1] in [".traj", ".csv"]

    trajectory_cameras = uavmvs.parse_uavmvs_traj(uavmvs_traj_path)

    record_lines = []
    for i, camera in enumerate(trajectory_cameras):
        timestamp = f"{i:016}"
        position = camera.position

        # NOTE *.traj stores a 3x3 rotation matrix, while *.csv uses wxyz quaternions
        camera = camera.into(uavmvs.TrajectoryCameraKind.Csv)
        assert np.allclose(camera.position, position)  # sanity check
        assert camera.rotation.shape == (4,)  # sanity check
        w, x, y, z = camera.rotation

        line_str = make_record_line(timestamp, position, orientation=(x, y, z, w))
        record_lines.append(line_str)

    print("timestamp tx ty tz qx qy qz qw")
    print("\n".join(record_lines))


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare a trajectory planned by uavmvs with the actual trajectory flown in"
        " AirSim by computing the absolute trajectory error (ATE) and relative pose error (RPE)."
    )

    parser.add_argument(
        "--convert_uavmvs",
        "-u",
        nargs=1,
        metavar="UAVMVS_TRAJ_PATH",
        help="Convert uavmvs's *.traj (or *.csv) to .log format",
    )
    parser.add_argument(
        "--convert_airsim",
        "-a",
        nargs=1,
        metavar="AIRSIM_REC_PATH",
        help="Convert AirSim's airsim_rec.txt to .log format",
    )

    parser.add_argument(
        "--eval",
        nargs=2,
        metavar=("GT_TRAJECTORY_PATH", "EST_TRAJECTORY_PATH"),
        help="Compare (AirSim) ground-truth to (uavmvs) estimate trajectory .log files",
    )

    args = parser.parse_args()

    if args.convert_uavmvs is not None:
        (uavmvs_traj_path,) = args.convert_uavmvs
        convert_uavmvs_to_log(uavmvs_traj_path)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path)

    if args.eval is not None:
        airsim_traj_path, uavmvs_traj_path = args.eval
        evaluate(gt_traj_path=airsim_traj_path, est_traj_path=uavmvs_traj_path)


###############################################################################
###############################################################################

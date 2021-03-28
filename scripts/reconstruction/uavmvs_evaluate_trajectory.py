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


def main(airsim_log_path, uavmvs_out_path):
    assert os.path.isfile(airsim_log_path), f"File not found: '{airsim_log_path}'"
    assert os.path.splitext(uavmvs_out_path)[1] == ".log"

    airsim_traj = np.loadtxt(airsim_log_path, skiprows=1, usecols=(1, 2, 3, 4, 5, 6, 7))
    print(f"airsim_traj.shape = {airsim_traj}")

    # XXX this code started as an inlining of `convert_meshroom_to_log`
    # and `convert_airsim_to_log`, but addapted to uavmvs' file format:
    assert os.path.isfile(uavmvs_out_path), f"File not found: '{uavmvs_out_path}'"
    assert os.path.splitext(uavmvs_out_path)[1] in [".traj", ".csv"]

    uavmvs_traj = np.zeros_like(airsim_traj)
    for i, camera in enumerate(uavmvs.parse_uavmvs_traj(uavmvs_out_path)):
        position = camera.position

        # NOTE .traj stores a 3x3 rotation matrix, while .csv uses WXYZ quaternions
        camera = camera.into(uavmvs.TrajectoryCameraKind.Csv)
        assert np.allclose(camera.position, position)  # sanity check
        assert camera.rotation.shape == (4,)  # sanity check
        w, x, y, z = camera.rotation

        # NOTE we skip the timestamp anyway (when calling evaluate)
        line_tuple = make_record_line(i, position, orientation=(x, y, z, w), as_string=False)
        uavmvs_traj[i] = np.array(line_tuple[1:], dtype=uavmvs_traj.dtype)

    print(f"uavmvs_traj.shape = {uavmvs_traj}")

    raise NotImplementedError
    # TODO evaluate(gt_traj_path=airsim_traj_path, est_traj_path=uavmvs_traj_path)


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare a trajectory planned by uavmvs with the actual trajectory flown in"
        " AirSim by computing the absolute trajectory error (ATE) and relative pose error (RPE)."
    )

    parser.add_argument(
        "uavmvs_out_path",
        type=str,
        help="Path to uavmvs's .traj / .csv estimate trajectory",
    )
    parser.add_argument(
        "airsim_log_path",
        type=str,
        # NOTE use evaluate_trajectory.py to convert airsim_rec.txt to .log
        help="Path to AirSim's .log ground-truth trajectory",
    )

    # NOTE unlike evaluate_trajectory.py, we can't simply convert a uavmvs trajectory
    # without the corresponding AirSim file since it is only a "flight plan". Hence,
    # it doesn't have any timestamps, so we need to associate the planned positions
    # to the ones that were actually recorded in AirSim and copy their timestamps.

    args = parser.parse_args()
    main(args.airsim_log_path, args.uavmvs_out_path)


###############################################################################
###############################################################################

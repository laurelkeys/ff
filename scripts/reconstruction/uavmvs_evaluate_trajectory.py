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


def main(args):
    # NOTE this is pretty much an inlining of `convert_meshroom_to_log`
    # and `convert_airsim_to_log`, but addapted to uavmvs' file format:
    assert os.path.isfile(args.uavmvs_out_path), f"File not found: '{args.uavmvs_out_path}'"
    assert os.path.splitext(args.uavmvs_out_path)[1] in [".traj", ".csv"]

    uavmvs_log_path = f"{args.uavmvs_out_path}.TUM.log"
    assert not os.path.exists(uavmvs_log_path)  # temporary file

    with open(uavmvs_log_path, "w") as uavmvs_log_file:
        trajectory_cameras = uavmvs.parse_uavmvs_traj(args.uavmvs_out_path)

        record_lines = []
        for i, camera in enumerate(trajectory_cameras):
            timestamp = f"{i:016}"
            position = camera.position

            # NOTE .traj stores a 3x3 rotation matrix, while .csv uses WXYZ quaternions
            camera = camera.into(uavmvs.TrajectoryCameraKind.Csv)
            assert np.allclose(camera.position, position)  # sanity check
            assert camera.rotation.shape == (4,)  # sanity check
            w, x, y, z = camera.rotation

            line_str = make_record_line(timestamp, position, orientation=(x, y, z, w))
            record_lines.append(line_str)

        print("timestamp tx ty tz qx qy qz qw", file=uavmvs_log_file)
        print("\n".join(record_lines), file=uavmvs_log_file)

    if args.convert_airsim is not None:
        (airsim_rec_path,) = args.convert_airsim
        convert_airsim_to_log(airsim_rec_path)

    if args.eval is not None:
        airsim_traj_path, uavmvs_traj_path = args.eval
        evaluate(gt_traj_path=airsim_traj_path, est_traj_path=uavmvs_traj_path)


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
        help="Path to uavmvs's .traj / .csv (or .log) estimate trajectory",
    )
    parser.add_argument(
        "airsim_rec_path",
        type=str,
        help="Path to AirSim's airsim_rec.txt (or .log) ground-truth trajectory",
    )

    # NOTE unlike evaluate_trajectory.py, we can't simply conver a uavmvs trajectory
    # without the corresponding AirSim file since it is only a "flight plan". Hence,
    # it doesn't have any timestamps, so we need to associate the planned positions
    # to the ones that were actually recorded in AirSim and copy their timestamps.

    main(args=parser.parse_args())


###############################################################################
###############################################################################

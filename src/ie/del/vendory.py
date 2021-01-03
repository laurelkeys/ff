import os
import sys

import numpy as np

sys.path.append(
    # FIXME
    os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "vendor", "tartanair_tools", "evaluation"
        )
    )
)

from tartanair_evaluator import TartanAirEvaluator, transform_trajs, quats2SEs

###############################################################################
###############################################################################


class Vendor:

    ###########################################################################
    ###########################################################################

    class TanksAndTemples:
        pass

    ###########################################################################
    ###########################################################################

    class TartanAir:
        @staticmethod
        def evaluate_traj(gt_traj_fname, est_traj_fname, scale=True):
            """ Returns a dict with `ate_score`, `rpe_score` and `kitti_score` keys.\n
                Note: use `scale=True` for monocular track, `scale=False` for stereo track.
            """
            return TartanAirEvaluator().evaluate_one_trajectory(
                gt_traj_fname, est_traj_fname, scale
            )


        @staticmethod
        def traj_from_file(traj_fname, skip_header=False):
            """ Returns a numpy array representation of `traj_fname`. """
            if skip_header:
                return np.loadtxt(traj_fname, skiprows=1)
            return np.loadtxt(traj_fname)

        @staticmethod
        def evaluate(gt_traj, est_traj, scale=True):
            """ Returns a dict with `ate_score`, `rpe_score` and `kitti_score` keys.\n
                Note: use `scale=True` for monocular track, `scale=False` for stereo track.
            """
            evaluator = TartanAirEvaluator()

            # transform and scale
            gt_traj_trans, est_traj_trans, _ = transform_trajs(gt_traj, est_traj, scale)
            gt_SEs, est_SEs = quats2SEs(gt_traj_trans, est_traj_trans)

            # evaluate
            ate_score, *_ = evaluator.ate_eval.evaluate(gt_traj, est_traj, scale)
            rpe_score = evaluator.rpe_eval.evaluate(gt_SEs, est_SEs)
            kitti_score = evaluator.kitti_eval.evaluate(gt_SEs, est_SEs)

            return {"ate_score": ate_score, "rpe_score": rpe_score, "kitti_score": kitti_score}


###############################################################################
###############################################################################

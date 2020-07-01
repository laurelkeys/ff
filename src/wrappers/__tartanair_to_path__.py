import os, sys

from __vendor_to_path__ import __vendor_path

__TartanAir_path = os.path.join(__vendor_path, "tartanair_tools")

sys.path.insert(0, os.path.join(__TartanAir_path, "evaluation"))

from tartanair_tools.evaluation import (
    transformation,
    trajectory_transform,   # imports transformation

    evaluate_ate_scale,
    evaluate_kitti,
    evaluate_rpe,
    evaluator_base,         # imports transformation, trajectory_transform

    tartanair_evaluator,    # imports evaluator_base
)

del sys.path[0], sys, os

if __name__ == "__main__":
    print(f"vendor path = '{__vendor_path}'")
    print(f"TartanAir path = '{__TartanAir_path}'")

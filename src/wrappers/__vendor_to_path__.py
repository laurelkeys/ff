import os
import sys

__vendor_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "vendor"
)

__TanksAndTemples_path = os.path.join(__vendor_path, "TanksAndTemples")
__TartanAir_path = os.path.join(__vendor_path, "tartanair_tools")

sys.path.insert(0,  __vendor_path)

from TanksAndTemples.python_toolbox import (
    # convert_to_logfile,
    interpolate_log_file,
)

from TanksAndTemples.python_toolbox.evaluation import (
    config,
    evaluation,
    plot,
    registration,
    run,
    trajectory_io,
    util,
)

from tartanair_tools.evaluation import (
    evaluate_ate_scale,
    evaluate_kitti,
    evaluate_rpe,
    evaluator_base,
    tartanair_evaluator,
    trajectory_transform,
    transformation,
)

del sys.path[0], sys, os

if __name__ == "__main__":
    print(f"vendor path = '{__vendor_path}'")

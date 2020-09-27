import os, sys

from .__vendor_to_path__ import __vendor_path

__TanksAndTemples_path = os.path.join(__vendor_path, "TanksAndTemples")

sys.path.insert(0,  os.path.join(__TanksAndTemples_path, "python_toolbox"))
sys.path.insert(0,  os.path.join(__TanksAndTemples_path, "python_toolbox", "evaluation"))

from TanksAndTemples.python_toolbox import (
    # convert_to_logfile,   # imports read_model (NOTE needs to be copied from https://github.com/colmap/colmap)
    interpolate_log_file,
)

from TanksAndTemples.python_toolbox.evaluation import (
    config,
    util,
    plot,

    trajectory_io,          # imports open3d
    registration,           # imports open3d, trajectory_io

    evaluation,             # imports open3d

    run,                    # imports open3d, config, registration, evaluation, util, plot
)

del sys.path[0:2], sys, os

if __name__ == "__main__":
    print(f"vendor path = '{__vendor_path}'")
    print(f"Tanks and Temples path = '{__TanksAndTemples_path}'")

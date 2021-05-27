import sys
import json
import argparse

from os.path import join, abspath, dirname, isfile

try:
    FF_PROJECT_ROOT = abspath(join(abspath(__file__), "..", "..", "..", ".."))
    FF_SCRIPTS_DATA = join(FF_PROJECT_ROOT, "scripts", "data")

    def include(*relative_path):
        file_dir_path = dirname(abspath(__file__))
        absolute_path = abspath(join(file_dir_path, *relative_path))
        sys.path.append(dirname(absolute_path))

    include(FF_PROJECT_ROOT, "misc", "tools", "tanksandtemples_evaluator")
    from tanksandtemples_evaluator import TanksAndTemplesEvaluator
except:
    raise

DEBUG = True  # HACK

SCENE_NAME = None
OUTPUT_FOLDER = None
SOURCE_PLY_PATH = None  # Estimate reconstruction
TARGET_PLY_PATH = None  # Ground-truth reconstruction

DTAU_THRESHOLD = 0.5
PLOT_STRETCH = 5

# Skip the three registration steps used for refining the transformation
# matrix that aligns source_ply_path to target_ply_path.
SKIP_REFINEMENT = False

# Crop volume in the same reference frame as target_ply_path (optional).
TARGET_CROP_JSON_PATH = None

# Camera positions used for computing a rough pre-alignment matrix from source_ply_path
# to target_ply_path (optional). It is assumed that source_log_path's coordinate system
# is the same as source_ply_path's, while target_log_path's might be different from
# target_ply_path's (in which case target_log_to_ply_align_txt_path is required).
SOURCE_LOG_PATH, TARGET_LOG_PATH = None, None

# Transformation matrix that converts the positions from target_log_path to the
# reference frame used by target_ply_path. This is only required if the trajectory
# described by target_log_path is not in the same coordinate system as target_ply_path.
TARGET_LOG_TO_PLY_ALIGN_TXT_PATH = None

# Transformation matrix that aligns source_ply_path to target_ply_path (optional).
# This can be used as a replacement for the pre-alignment computed from the .log
# trajectory files (i.e. source_ply_path and target_ply_path).
SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH = None


def update_run_args(run_args, dollar_replace=None):
    if dollar_replace is None:
        for arg in run_args.values():
            assert "$" not in arg, f"found $ in '{arg}' but --replace was not used"
    else:
        for key in run_args.keys():
            value = run_args[key]
            try:
                if "$" in value:
                    run_args[key] = value.replace("$", dollar_replace)
            except TypeError:
                pass

    global SCENE_NAME
    global OUTPUT_FOLDER
    global SOURCE_PLY_PATH
    global TARGET_PLY_PATH
    global DTAU_THRESHOLD
    global PLOT_STRETCH
    global SKIP_REFINEMENT
    global TARGET_CROP_JSON_PATH
    global TARGET_LOG_TO_PLY_ALIGN_TXT_PATH
    global SOURCE_LOG_PATH
    global TARGET_LOG_PATH
    global SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH

    SCENE_NAME = run_args["scene_name"]
    OUTPUT_FOLDER = run_args["output_folder"]
    SOURCE_PLY_PATH = run_args["source_ply_path"]
    TARGET_PLY_PATH = run_args["target_ply_path"]

    DTAU_THRESHOLD = run_args.get("dtau_threshold", DTAU_THRESHOLD)
    PLOT_STRETCH = run_args.get("plot_stretch", PLOT_STRETCH)

    SKIP_REFINEMENT = run_args.get("skip_refinement", SKIP_REFINEMENT)

    TARGET_CROP_JSON_PATH = run_args.get("target_crop_json_path", TARGET_CROP_JSON_PATH)

    SOURCE_LOG_PATH = run_args.get("source_log_path", SOURCE_LOG_PATH)
    TARGET_LOG_PATH = run_args.get("target_log_path", TARGET_LOG_PATH)
    TARGET_LOG_TO_PLY_ALIGN_TXT_PATH = run_args.get(
        "target_log_to_ply_align_txt_path", TARGET_LOG_TO_PLY_ALIGN_TXT_PATH
    )

    SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH = run_args.get(
        "source_ply_to_ply_align_txt_path", SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH
    )


def main(args):
    with open(args.json_path, "r") as f:
        run_args = json.load(f)
        if args.verbose:
            print(json.dumps(run_args, indent=2))
        update_run_args(run_args, args.replace)

    assert SCENE_NAME is not None
    assert OUTPUT_FOLDER is not None
    assert SOURCE_PLY_PATH is not None
    assert TARGET_PLY_PATH is not None

    if args.verbose:
        print(f"{SCENE_NAME = }")
        print(f"{OUTPUT_FOLDER = }")
        print(f"{SOURCE_PLY_PATH = }")
        print(f"{TARGET_PLY_PATH = }")
        print(f"{DTAU_THRESHOLD = }")
        print(f"{PLOT_STRETCH = }")
        print(f"{SKIP_REFINEMENT = }")
        print(f"{TARGET_CROP_JSON_PATH = }")
        print(f"{SOURCE_LOG_PATH = }")
        print(f"{TARGET_LOG_PATH = }")
        print(f"{TARGET_LOG_TO_PLY_ALIGN_TXT_PATH = }")
        print(f"{SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH = }")

    if DEBUG:
        return

    TanksAndTemplesEvaluator.evaluate_reconstruction(
        scene_name=SCENE_NAME,
        output_folder=OUTPUT_FOLDER,
        dtau_threshold=DTAU_THRESHOLD,
        source_ply_path=SOURCE_PLY_PATH,  # Estimate reconstruction
        target_ply_path=TARGET_PLY_PATH,  # Ground-truth reconstruction
        # Axis-aligned bounding box in the ground-truth reference frame, used
        # to select the region of interest from the dense point clouds (.ply).
        target_crop_json_path=TARGET_CROP_JSON_PATH,
        # Transformation matrix that takes the (possibly arbitrary) reference frame
        # of target_log_path to the same coordinate system as target_ply_path.
        target_log_to_ply_align_txt_path=TARGET_LOG_TO_PLY_ALIGN_TXT_PATH,
        source_log_path=SOURCE_LOG_PATH,  # Camera positions in the same reference frame as source_ply_path
        target_log_path=TARGET_LOG_PATH,  # Camera positions with (possibly) an arbitrary coordinate system
        # Transformation matrix used as a pre-alignment of source_ply_path to target_ply_path if
        # source_log_path or target_log_path are None (otherwise, this is ignored).
        source_ply_to_ply_align_txt_path=SOURCE_PLY_TO_PLY_ALIGN_TXT_PATH,
        # If True, then the three steps of registration refinement are skipped. The pre-alignment
        # matrix is still applied if it is not None, though.
        skip_refinement=SKIP_REFINEMENT,
        plot_stretch=PLOT_STRETCH,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Automate the setup running misc/tools/tanksandtemples_evaluator.py"
    )

    parser.add_argument("json_path", type=str, help="run args")
    parser.add_argument("--replace", type=str, help="change $")
    parser.add_argument("--verbose", "-v", action="store_true")

    args = parser.parse_args()
    if DEBUG:
        args.verbose = True

    assert isfile(args.json_path), f"invalid file path: '{args.json_path}'"

    main(args)

# {
#     "scene_name": null,
#     "output_folder": null,
#     "dtau_threshold": 0.5,
#     "source_ply_path": null,
#     "target_ply_path": null,
#     "target_crop_json_path": null,
#     "target_log_to_ply_align_txt_path": null,
#     "source_log_path": null,
#     "target_log_path": null,
#     "source_ply_to_ply_align_txt_path": null,
#     "skip_refinement": false,
#     "plot_stretch": 5
# }

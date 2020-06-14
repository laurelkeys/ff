import os
import sys
import argparse

import ff

# from vendor.tartanair_tools.evaluation import *
# from vendor.TanksAndTemples.python_toolbox import *
from wrappers.meshroomy import MeshroomParser, MeshroomTransform

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim

# NOTE ATE is well-suited for measuring the performance of visual SLAM systems, in contrast,
#      RPE is well-suited for measuring the drift of visual odometry systems (e.g. the drift per second)

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


def parse_timestamp(file_path):
    return os.path.splitext(os.path.basename(file_path))[0].split("_")[1]  # HACK


###############################################################################
###############################################################################


def main(args):
    assert os.path.isfile(
        args.meshroom_cameras_file_path
    ), f"File not found: '{args.meshroom_cameras_file_path}'"
    assert os.path.isfile(
        args.airsim_recording_file_path
    ), f"File not found: '{args.airsim_recording_file_path}'"

    ##
    ## Meshroom
    ##

    views, poses = MeshroomParser.parse_cameras(args.meshroom_cameras_file_path)
    views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

    record_lines = []
    for view_id, view in views_dict.items():
        timestamp = os.path.splitext(os.path.basename(view.path))[0].split("_")[1]  # HACK

        pose = poses_dict[view.pose_id]
        position = MeshroomTransform.translation(pose.center)
        orientation = MeshroomTransform.rotation(pose.rotation, as_quaternion=True)

        line_str = make_record_line(timestamp, position, orientation)
        record_lines.append((timestamp, line_str))  # store a tuple

    # sort by timestamp and filter out the first element in the tuple
    meshroom_record = ["timestamp tx ty tz qx qy qz qw"]
    meshroom_record.extend([_[1] for _ in sorted(record_lines)])

    ##
    ## AirSim
    ##

    with open(args.airsim_recording_file_path) as f:
        airsim_record = [line.rstrip() for line in f]

    ##
    ## Evaluation
    ##

    print(len(meshroom_record))
    print(len(airsim_record))


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "meshroom_cameras_file_path", type=str, help="Path to Meshroom's cameras.json"
    )
    parser.add_argument(
        "airsim_recording_file_path", type=str, help="Path to AirSim's .log formatted file"
    )
    args = parser.parse_args()

    main(args)


###############################################################################
###############################################################################

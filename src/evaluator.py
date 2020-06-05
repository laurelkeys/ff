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


def parse_timestamp(file_path):
    return os.path.splitext(os.path.basename(file_path))[0].split("_")[1]  # HACK


def main(args):
    assert os.path.isfile(args.cameras_file_path), f"File not found: '{args.cameras_file_path}'"

    views, poses = MeshroomParser.parse_cameras(args.cameras_file_path)
    views_dict, poses_dict = MeshroomParser.extract_views_and_poses(views, poses)

    for view_id, view in views_dict.items():
        print(f"{view_id=}")
        position = MeshroomTransform.translation(poses_dict[view.pose_id].center)
        orientation = MeshroomTransform.rotation(poses_dict[view.pose_id].rotation, as_quaternion=True)
        line_str = make_record_line(parse_timestamp(view.path), position, orientation)
        print(line_str)
        break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("cameras_file_path", type=str, help="Path to Meshroom's cameras.json")
    # TODO get AirSim's data
    args = parser.parse_args()

    main(args)

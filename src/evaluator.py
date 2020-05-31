import ff

from vendor.tartanair_tools.evaluation import *
from vendor.TanksAndTemples.python_toolbox import *

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


if __name__ == "__main__":
    pass

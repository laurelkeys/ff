from airsim.types import YawMode, DrivetrainType


# FIXME work in progress..
class MoveArgs:
    def __init__(
        self,
        # https://microsoft.github.io/AirSim/apis/#drivetrain
        drivetrain: DrivetrainType,
        # https://microsoft.github.io/AirSim/apis/#yaw_mode
        yaw_mode: YawMode,
        # https://microsoft.github.io/AirSim/apis/#lookahead-and-adaptive_lookahead
        lookahead: int = -1,  # How far to look ahead on the path (default -1 means auto)
        adaptive_lookahead: bool = True,  # Whether to apply adaptive lookahead (1=yes, 0=no)
    ):
        self.drivetrain = drivetrain
        self.yaw_mode = yaw_mode
        self.lookahead = lookahead
        self.adaptive_lookahead = 1 if adaptive_lookahead else 0

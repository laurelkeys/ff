import os
import time
import argparse

from typing import List, Optional, NamedTuple, cast

import ff
import airsim

from ds import Rect, Rgba, Controller
from ie.airsimy import reset, connect
from airsim.types import Pose, Vector3r
from airsim.client import MultirotorClient

RECORD: bool = True

AUGMENT_PATHS: bool = True
AUGMENT_PATHS_MAX_DIST: float = 2

PLOT_SHOW: bool = False
PLOT_CLEAR: bool = True

PLOT_LINE_LIST: bool = False
PLOT_LINE_STRIP: bool = not PLOT_LINE_LIST

PLOT_POINT_SIZE: float = 10.0 / 2  # 10.0 default
PLOT_LINE_THICKNESS: float = 5.0 / 2  # 5.0 default

PLOT_DURATION: Optional[float] = None  # persistent if None


def _plot_duration():
    return {"is_persistent": True} if PLOT_DURATION is None else {"duration": PLOT_DURATION}


class Plotter(NamedTuple):
    client: MultirotorClient

    # NOTE this doesn't include the following methods:
    # simPlotArrows, simPlotStrings simPlotTransforms, simPlotTransformsWithNames

    def flush_persistent(self):
        if PLOT_CLEAR:
            self.client.simFlushPersistentMarkers()

    def points(self, points: List[Vector3r], color: Rgba, size: float = PLOT_POINT_SIZE):
        if PLOT_SHOW:
            self.client.simPlotPoints(points, color, size, **_plot_duration())

    def lines(self, points: List[Vector3r], color: Rgba, thickness: float = PLOT_LINE_THICKNESS):
        if PLOT_SHOW:
            if PLOT_LINE_STRIP:
                self.client.simPlotLineStrip(points, color, thickness, **_plot_duration())
            if PLOT_LINE_LIST:
                self.client.simPlotLineList(points, color, thickness, **_plot_duration())


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    assert os.path.isfile(args.roi), f"Invalid file path: '{args.roi}'"

    with open(args.roi, "r") as f:
        args.rect = Rect.from_dump(f.read())

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        ff.input_or_exit("\nPress [enter] to connect to AirSim ")


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: MultirotorClient, args: argparse.Namespace) -> None:
    reset(client)

    rect = cast(Rect, args.rect)
    initial_pose = cast(Pose, client.simGetVehiclePose())
    closest_corner = rect.closest_corner(initial_pose.position)

    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)
        ff.log_info(f"Closest corner {ff.to_xyz_str(closest_corner)}")

    start_pos = Vector3r(*ff.to_xyz_tuple(closest_corner))  # NOTE make sure we copy it by value
    start_pos.z_val += args.z_offset
    start_pos.y_val += args.y_offset
    start_pos.x_val += args.x_offset

    plot = Plotter(client)
    plot.flush_persistent()
    plot.points([start_pos], Rgba.Red)

    if args.teleport:
        ff.log(f"Teleporting to {ff.to_xyz_str(start_pos)}...")
        new_pose = Pose(start_pos, initial_pose.orientation)
        Controller.teleport(client, to=new_pose, ignore_collison=True)
    else:
        ff.log(f"Flying to {ff.to_xyz_str(start_pos)}...")
        client.moveToPositionAsync(*ff.to_xyz_tuple(start_pos), velocity=5).join()

    ff.log("Capturing grid...")
    time.sleep(2)

    z = Vector3r(0, 0, args.altitude_shift)
    path = [z + Vector3r(*ff.to_xyz_tuple(corner)) for corner in rect.zigzag(lanes=args.zigzags)]
    if AUGMENT_PATHS:
        path = Controller.augment_path(path, AUGMENT_PATHS_MAX_DIST)

    plot.points(path, Rgba.White)
    plot.lines(path, Rgba.Green)

    try:
        if RECORD:
            client.startRecording()
        Controller.fly_path(client, path)
    except Exception as e:
        raise e
    finally:
        if RECORD:
            client.stopRecording()


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    preflight(args)  # setup
    client = connect(ff.SimMode.Multirotor)
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
    finally:
        ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("roi", type=str, help="Path to the region of interest (ROI) file")
    parser.add_argument("--clock", type=float, default=1.0, help="Change AirSim's clock speed")
    parser.add_argument("--teleport", action="store_true", help="Teleport instead of flying")
    parser.add_argument("--altitude_shift", "-alt", type=float, default=0)
    parser.add_argument("--zigzags", "-zz", type=float, default=4)
    parser.add_argument("--z_offset", type=float, default=0)
    parser.add_argument("--y_offset", type=float, default=0)
    parser.add_argument("--x_offset", type=float, default=0)

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

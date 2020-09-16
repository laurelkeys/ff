# https://www.dji.com/br/ground-station-pro
# https://www.pix4d.com/product/pix4dcapture
# https://heighttech.nl/flight-planning-software/

# TODO try using UE4's specific API calls for plotting

from __future__ import annotations

import json
import time
import argparse

from enum import Enum
from typing import List

from pynput import keyboard

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim
finally:
    from airsim.types import Vector3r


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    pass


###############################################################################
## fly (called after connecting) ##############################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    # verify we are in "ComputerVision" mode
    if (sim_mode := ff.curr_sim_mode()) != ff.SimMode.ComputerVision:
        assert False, f"Please change the SimMode from '{sim_mode}' to 'ComputerVision'"

    zone = Rect(Vector3r(), Vector3r(0, 10, 0), Vector3r(10, 0, 0))

    # repeat the first coordinate to close the line strip
    client.simPlotLineStrip(points=zone.corners(repeat_first=True), is_persistent=True)

    # NOTE available keys, not used by AirSim:
    # ,----------------------------------------------------------------------------------------.
    # |  ` |     |     |     |  4  |  5  |  6  |  7  |  8  |  9  |     |  -* |  =  |           |
    # |----------------------------------------------------------------------------------------+
    # |  Tab   |     |     |     |     |     |  Y  |  U  |  I* |  O  |  P  |  [  |  ]  |       |
    # |----------------------------------------------------------------------------------------+
    # |  CpsLck |     |     |     |     |  G  |  H  |  J  |  K*  |  L  |     |  '  |    Enter  |
    # |----------------------------------------------------------------------------------------+
    # |   Shift   |  Z  |  X  |  C  |  V  |     |  N  |     |  ,  |  .  |     |      Shift     |
    # `----------------------------------------------------------------------------------------'

    edit_mode: EditMode = EditMode.TRANSLATING

    # define keyboard callbacks
    swap_mode_key = keyboard.Key.shift_l
    save_rect_key = keyboard.Key.shift_r

    def on_press(key):
        nonlocal edit_mode, swap_mode_key, zone

        client.simPrintLogMessage("ROI coordinates: ", message_param=str(zone))

        if key == swap_mode_key:
            edit_mode = EditMode.next(edit_mode)
            client.simPrintLogMessage("Current edit mode: ", message_param=edit_mode.name)

        elif key == save_rect_key:
            # TODO prefix by the UE4 env name:
            roi_filename = '-'.join(("roi", time.strftime(r"%Y%m%d-%H%M%S")))
            with open(roi_filename, 'w') as f:
                json.dump(zone, f)
            client.simPrintLogMessage(f"Saved ROI coordinates to '{roi_filename}'")

        else:
            try:
                # NOTE AirSim uses NED coordinates
                # TODO refactor the if-statements below with a dict (?)
                if edit_mode == EditMode.TRANSLATING:
                    if   key.char == "z": zone.center.y_val += 1  # right
                    elif key.char == "x": zone.center.y_val -= 1  # left
                    elif key.char == "c": zone.center.x_val += 1  # front
                    elif key.char == "v": zone.center.x_val -= 1  # back
                    else: return
                elif edit_mode == EditMode.SCALING:
                    if   key.char == "z": zone.half_width *= 1.1  # inc. horizontally
                    elif key.char == "x": zone.half_width *= 0.9  # dec. horizontally
                    elif key.char == "c": zone.half_height *= 1.1  # inc. vertically
                    elif key.char == "v": zone.half_height *= 0.9  # dec. vertically
                    else: return
                else: return
                # FIXME calling this too many times causes UE4 to crash..
                #       see "./images/simPlotLineStrip crash.png" for more info
                client.simFlushPersistentMarkers()
                client.simPlotLineStrip(points=zone.corners(repeat_first=True), is_persistent=True)
            except: pass


    def on_release(key):
        if key == keyboard.Key.esc:
            return False  # stop the listener

    # collect events untill released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        ff.log("Press [esc] to quit")
        listener.join()

    # TODO save the circle position so that it can be used to fly a drone later

    ff.log("Done")


###############################################################################
## main #######################################################################
###############################################################################


class EditMode(Enum):
    SCALING = 0
    TRANSLATING = 1

    @staticmethod
    def next(edit_mode: EditMode) -> EditMode:
        edit_modes = list(EditMode)
        return edit_modes[(edit_mode.value + 1) % len(edit_modes)]


class Rect:
    def __init__(
        self, center: airsim.Vector3r, width: airsim.Vector3r, height: airsim.Vector3r
    ) -> None:
        self.center = center
        self.half_width = width / 2.0
        self.half_height = height / 2.0

    def corners(self, repeat_first: bool = False) -> List[airsim.Vector3r]:
        corners = [
            self.center - self.half_width - self.half_height,
            self.center + self.half_width - self.half_height,
            self.center + self.half_width + self.half_height,
            self.center - self.half_width + self.half_height,
        ]
        return corners if not repeat_first else corners + [corners[0]]

    def __str__(self) -> str:
        return f"Rect({', '.join([ff.to_xyz_str(_) for _ in (self.center, self.half_width, self.half_height)])})"


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

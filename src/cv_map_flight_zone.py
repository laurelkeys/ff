# https://www.dji.com/br/ground-station-pro
# https://www.pix4d.com/product/pix4dcapture
# https://heighttech.nl/flight-planning-software/

from __future__ import annotations

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
    corners = zone.corners()
    corners.append(corners[0])  # repeat the first coordinate to close the line strip

    client.simPlotLineStrip(points=corners, duration=5)

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

    def on_press(key):
        nonlocal edit_mode, swap_mode_key
        if key == swap_mode_key:
            edit_mode = EditMode.next(edit_mode)
            client.simPrintLogMessage("Current edit mode: ", message_param=edit_mode.name)

    def on_release(key):
        print("{0} released".format(key))
        if key == keyboard.Key.esc:
            return False  # stop the listener

    # collect events untill released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        ff.log("Press [esc] to quit")
        listener.join()

    # TODO plot a circle (maybe get its initial position form args)
    # https://github.com/microsoft/AirSim/pull/2304
    # https://github.com/microsoft/AirSim/pull/2506

    # TODO scale the circle using the keyboard
    # TODO move the circle using the keyboard
    # TODO switch between "input types": scaling / moving

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

    def corners(self) -> List[airsim.Vector3r]:
        return [
            self.center - self.half_width - self.half_height,
            self.center + self.half_width - self.half_height,
            self.center + self.half_width + self.half_height,
            self.center - self.half_width + self.half_height,
        ]


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

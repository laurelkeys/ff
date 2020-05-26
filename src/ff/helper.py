import os
import sys
import json
import subprocess

from .sim import SimMode
from .defaults import Default


###############################################################################
###############################################################################


def add_airsim_to_path(airsim_path):
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))


###############################################################################
###############################################################################


def curr_sim_mode(settings_path=Default.SETTINGS_PATH):
    with open(settings_path, "r") as settings_file:
        settings = json.load(settings_file)
    return settings["SimMode"]


def change_sim_mode(new_sim_mode, settings_path=Default.SETTINGS_PATH):
    """ Returns a tuple `(changed_sim_mode: bool, curr_sim_mode: str)`. """

    assert new_sim_mode in SimMode._list_all, f"\ninvalid SimMode '{new_sim_mode}'\n"

    with open(settings_path, "r") as settings_file:
        settings = json.load(settings_file)

    sim_mode = settings["SimMode"]
    if sim_mode != new_sim_mode:
        settings["SimMode"] = new_sim_mode
        with open(settings_path, "w") as settings_file:
            json.dump(settings, settings_file, indent=2)
        return True, sim_mode

    return False, sim_mode


###############################################################################
###############################################################################

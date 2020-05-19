import os
import sys
import json
import subprocess

from .sim import SimMode
from .constants import Default


def add_airsim_to_path(airsim_path):
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))


###############################################################################
###############################################################################


def create_symbolic_link(airsim_path=Default.AIRSIM_CLIENT_PATH, verbose=False):
    assert os.path.exists(
        os.path.join(airsim_path, "client.py")
    ), f"\nexpected '{os.path.join(airsim_path, 'client.py')}' does not exist\n"

    symlink_cmds = ["ln", "-s", airsim_path, "airsim"]
    if verbose:
        symlink_cmds.append("--verbose")
    subprocess.run(symlink_cmds)


###############################################################################
###############################################################################


def change_sim_mode(new_sim_mode, settings_file_path=Default.SETTINGS_PATH):
    assert new_sim_mode in SimMode._list_all, f"\ninvalid SimMode '{new_sim_mode}'\n"

    with open(settings_file_path, "r") as settings_file:
        settings = json.load(settings_file)

    sim_mode = settings["SimMode"]
    if sim_mode != new_sim_mode:
        settings["SimMode"] = new_sim_mode
        with open(settings_file_path, "w") as settings_file:
            json.dump(settings, settings_file, indent=2)
        return True, sim_mode

    return False, sim_mode

import os
import sys
import json

from glob import glob
from os.path import join

from .sim import SimMode
from .defaults import Default

###############################################################################
###############################################################################


def add_airsim_to_path(airsim_path):
    assert os.path.exists(join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))


###############################################################################
###############################################################################


def curr_sim_mode(settings_path=Default.AIRSIM_SETTINGS_PATH):
    with open(settings_path, "r") as settings_file:
        settings = json.load(settings_file)
    return settings["SimMode"]


def change_sim_mode(new_sim_mode, settings_path=Default.AIRSIM_SETTINGS_PATH):
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


def possible_env_paths(env_root, exts=["uproject", "sln", "exe"]):
    """ Searches for valid environment files with the following patterns:
        - `env_root/*.ext`
        - `env_root/*/*.ext`
    """
    env_paths = []
    for ext in exts:
        env_paths.extend(glob(join(env_root, f"*.{ext}")))
        env_paths.extend(glob(join(env_root, "*", f"*.{ext}")))
    return env_paths


###############################################################################
###############################################################################


def input_or_exit(prompt: str) -> str:
    """ Simple wrapper around `input()` to catch `KeyboardInterrupt` and exit. """
    try:
        return input(prompt)
    except KeyboardInterrupt:
        exit()


###############################################################################
###############################################################################


def settings_str_from_dict(settings: dict) -> str:
    """ Converts a `dict` representing AirSim's settings.json to the JSON
        string that can be passed to it through the `--settings` argument.
    """
    # NOTE for some reason (at least on Windows), passing the settings string
    #      directly in the command line doesn't seem to work, so this is used
    #      instead, to correctly create one for launcher.py's methods (#2824)
    return json.dumps(settings).replace('"', '\\"')


# TODO def settings_str_from_file(json_path: str) -> dict:


###############################################################################
###############################################################################

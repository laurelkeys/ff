import os
import glob
import subprocess

import psutil

from .defaults import Default


###############################################################################
###############################################################################


class LaunchEnvArgs:
    """ Wraps `argparse` arguments to be used in `launch_env()`. """

    def __init__(self, args):
        self.verbose = args.verbose
        self.env_path = self._make_env_path(args.env_root, args.env_name)
        # self.edit = args.edit
        self.devenv_exe = args.devenv_exe
        self.ue4editor_exe = args.ue4editor_exe

    def _make_env_path(self, env_root, env_name):
        """ Searches for a valid environment. The first match will be used:
            1. `env_name.ext`
            2. `env_name/*.ext`
            3. `env_root/env_name/*.ext`
            4. `env_root/*/env_name.ext`\n
            Note: if `env_name` contains an extension, only 1 and 4 are checked,
                  otherwise, the tested extensions are: "uproject", "sln", "exe".
        """
        _, ext = os.path.splitext(env_name)
        if ext is not None:
            # 1. Check if `env_name` is a valid path to an environment file.
            if os.path.isfile(env_name):
                return env_name

            # 4. Check if `env_root/*/env_name` is a valid path to an environment file.
            matches = glob.glob(f"{env_root}/*/{env_name}.{ext}")
            if matches:
                return matches[0]

        else:
            exts = ["uproject", "sln", "exe"]

            # 1. Check if `env_name` is a valid path to an environment file.
            for ext in exts:
                matches = glob.glob(f"{env_name}.{ext}")
                if matches:
                    return matches[0]

            # 2. Check if `env_name/` is a valid folder, with an environment file inside.
            if os.path.isdir(env_name):
                for ext in exts:
                    matches = glob.glob(f"{env_name}/*.{ext}")
                    if matches:
                        return matches[0]

            # 3. Check if `env_root/env_name/` is a valid folder, with an environment file inside.
            if os.path.isdir(os.path.join(env_root, env_name)):
                for ext in exts:
                    matches = glob.glob(f"{env_root}/{env_name}/*.{ext}")
                    if matches:
                        return matches[0]

            # 4. Check if `env_root/*/env_name` is a valid path to an environment file.
            for ext in exts:
                matches = glob.glob(f"{env_root}/*/{env_name}.{ext}")
                if matches:
                    return matches[0]

        assert False, f"No environment file was found for env_root={env_root}, env_name={env_name}"

###############################################################################
###############################################################################


def launch_env(args):
    assert isinstance(args, LaunchEnvArgs), type(args)


###############################################################################
###############################################################################

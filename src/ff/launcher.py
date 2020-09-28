import os
import subprocess

from glob import glob
from os.path import join

import psutil

from .helper import possible_env_paths
from .logger import log_info, log_warning, log_debug
from .defaults import Default

###############################################################################
###############################################################################


class LaunchEnvArgs:
    """ Wraps `argparse` arguments to be used in `launch_env()`. """

    def __init__(self, args):
        env_root = Default.ENV_ROOT_ALIASES.get(args.env_root, args.env_root)

        if not args.env_name:
            self.env_path = self._try_to_make_env_path(env_root)
        elif args.edit:
            env_name, env_name_ext = os.path.splitext(args.env_name)
            assert env_name_ext in [None, ".sln"], args.env_name
            self.env_path = self._make_env_path(env_root, f"{env_name}.sln")
        else:
            self.env_path = self._make_env_path(env_root, args.env_name)

        if self.env_path.endswith("uproject"):
            assert os.path.isfile(args.ue4editor_exe), args.ue4editor_exe
        elif self.env_path.endswith("sln"):
            assert os.path.isfile(args.devenv_exe), args.devenv_exe

        self.ue4editor_exe = args.ue4editor_exe
        self.devenv_exe = args.devenv_exe
        self.verbose = args.verbose

    def __iter__(self):
        return iter((self.env_path, self.ue4editor_exe, self.devenv_exe, self.verbose))

    def _try_to_make_env_path(self, env_root):
        """ Searches for a valid environment given only env_root.
            The first match is returned:
            1. `env_root/*.ext`
            2. `env_root/*/*.ext`\n
            Note: this always prints a warning since it should be avoided.
        """
        # NOTE searches for .uproject, .sln and .exe
        possibilities = possible_env_paths(env_root)
        log_warning("env_name=None, trying to find a valid environment file in env_root..")
        assert possibilities, f"No environment was found for {env_root=}"
        if len(possibilities) > 1:
            lpad = len(str(len(possibilities)))  # left padding
            print(
                f"\nMultiple environment files were found for {env_root=}"
                + "\nChoosing the first one from the following:\n"
                + "\n".join([f" {i+1:{lpad}}. {env}" for i, env in enumerate(possibilities)])
                + "\n"
            )
        return possibilities[0]

    def _make_env_path(self, env_root, env_name):
        """ Searches for a valid environment. The first match is returned:
            1. `env_name.ext`
            2. `env_name/*.ext`
            3. `env_root/env_name/*.ext`
            4. `env_root/*/env_name.ext`\n
            Note: if `env_name` contains an extension, only 1 and 4 are checked.
                  Otherwise, the tested extensions are: "uproject", "sln", "exe".
        """
        _, ext = os.path.splitext(env_name)
        if ext in [".uproject", ".sln", ".exe"]:
            if os.path.isfile(env_name):
                return env_name # 1

            matches = glob(join(env_root, "*", env_name))
            if matches: return matches[0] # 4

        else:
            exts = ["uproject", "sln", "exe"]

            for ext in exts:
                matches = glob(f"{env_name}.{ext}")
                if matches: return matches[0] # 1

            if os.path.isdir(env_name):
                for ext in exts:
                    matches = glob(join(env_name, f"*.{ext}"))
                    if matches: return matches[0] # 2

            if os.path.isdir(join(env_root, env_name)):
                for ext in exts:
                    matches = glob(join(env_root, env_name, f"*.{ext}"))
                    if matches: return matches[0] # 3

            for ext in exts:
                matches = glob(join(env_root, "*", f"{env_name}.{ext}"))
                if matches: return matches[0] # 4

        assert False, f"No environment file was found for {env_root=}, {env_name=}"


###############################################################################
###############################################################################


def launch_env_from_args(args):
    launch_env(*LaunchEnvArgs(args))


def launch_env(env_path, ue4editor_exe=None, devenv_exe=None, verbose=True, **kwargs):
    """ Use `launch_env(*LaunchEnvArgs(args))` for convenience.\n
        Note: `res: (int, int)`, `windowed: bool` and `settings: str` can be passed
              through `**kwargs` for "uproject" and "exe" files.
    """
    _, ext = os.path.splitext(env_path)
    if ext == ".uproject":
        run_cmds, pid = _run_env(
            env_path, ext,
            env_proc="UE4Editor.exe",
            ue4editor_exe_path=ue4editor_exe,
            **kwargs
        )
    elif ext == ".sln":
        run_cmds, pid = _run_env(
            env_path, ext,
            env_proc="devenv.exe",  # Visual Studio
            devenv_exe_path=devenv_exe,
            **kwargs
        )
    elif ext == ".exe":
        run_cmds, pid = _run_env(
            env_path, ext,
            env_proc=os.path.basename(env_path),
            **kwargs
        )
    else:
        assert False, f"Unexpected extension in {env_path=}"

    if run_cmds:
        print("Launching environment... (this may take a few seconds)")
        if verbose:
            # log_info(f"pid={pid}")
            log_info(f"run_cmds={' '.join(run_cmds)}\n")

    return pid


def _run_env(env_path, env_ext, env_proc=None, **kwargs):
    assert os.path.isfile(env_path), f"File doesn't exist, {env_path=}"

    if env_proc is not None:
        already_running = [
            p.as_dict(attrs=['pid', 'name'])
            for p in psutil.process_iter()
            if env_proc.lower() in p.name().lower()
        ]

        if already_running:
            print(f"'{os.path.basename(env_path)}' is already running:")
            for p in already_running:  # there should (usually) only be one
                print(f" - name='{p['name']}', pid={p['pid']}")
            return [], already_running[0]['pid']

    run_cmds = _build_run_cmds(env_path, env_ext, **kwargs)
    p = subprocess.Popen(run_cmds, shell=True, stdout=subprocess.PIPE)

    log_debug(f"pid={p.pid}")
    return run_cmds, p.pid


def _build_run_cmds(
    env_path, env_ext,
    ue4editor_exe_path=None, devenv_exe_path=None,
    res=(1280, 720), windowed=True,
    settings=None  # NOTE use helper.py's methods to create this
):
    if env_ext == ".uproject":
        assert os.path.isfile(ue4editor_exe_path), ue4editor_exe_path
        cmds = [ue4editor_exe_path, env_path, "-game"]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        if windowed:
            cmds.append("-windowed")
        if settings is not None:
            cmds.extend(["--settings", f"{settings}"])

    elif env_ext == ".sln":
        assert os.path.isfile(devenv_exe_path), devenv_exe_path
        cmds = [devenv_exe_path, env_path]

    elif env_ext == ".exe":
        cmds = [env_path]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        if windowed:
            cmds.append("-windowed")
        if settings is not None:
            cmds.extend(["--settings", f"{settings}"])

    else:
        assert False, env_path

    return cmds


###############################################################################
###############################################################################

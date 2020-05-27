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
        if args.edit:
            env_name, env_name_ext = os.path.splitext(args.env_name)
            assert env_name_ext in [None, ".sln"], args.env_name
            self.env_path = self._make_env_path(args.env_root, f"{env_name}.sln")
        else:
            self.env_path = self._make_env_path(args.env_root, args.env_name)

        if self.env_path.endswith("uproject"):
            assert os.path.isfile(args.ue4editor_exe), args.ue4editor_exe
        elif self.env_path.endswith("sln"):
            assert os.path.isfile(args.devenv_exe), args.devenv_exe

        self.ue4editor_exe = args.ue4editor_exe
        self.devenv_exe = args.devenv_exe
        self.verbose = args.verbose

    def __iter__(self):
        return iter((self.env_path, self.ue4editor_exe, self.devenv_exe, self.verbose))

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
        if ext is not None:
            if os.path.isfile(env_name):
                return env_name # 1

            matches = glob.glob(f"{env_root}/*/{env_name}")
            if matches: return matches[0] # 4

        else:
            exts = ["uproject", "sln", "exe"]

            for ext in exts:
                matches = glob.glob(f"{env_name}.{ext}")
                if matches: return matches[0] # 1

            if os.path.isdir(env_name):
                for ext in exts:
                    matches = glob.glob(f"{env_name}/*.{ext}")
                    if matches: return matches[0] # 2

            if os.path.isdir(os.path.join(env_root, env_name)):
                for ext in exts:
                    matches = glob.glob(f"{env_root}/{env_name}/*.{ext}")
                    if matches: return matches[0] # 3

            for ext in exts:
                matches = glob.glob(f"{env_root}/*/{env_name}.{ext}")
                if matches: return matches[0] # 4

        assert False, f"No environment file was found for env_root='{env_root}', env_name='{env_name}'"


###############################################################################
###############################################################################


def launch_env(env_path, ue4editor_exe=None, devenv_exe=None, verbose=True, **kwargs):
    """ Use `launch_env(*LaunchEnvArgs(args))` for convenience.\n
        Note: `res: (int, int)` and `windowed: bool` can be passed through `**kwargs`
              for "uproject" and "exe" files.
    """
    _, ext = os.path.splitext(env_path)
    if ext == ".uproject":
        run_cmds = _run_env(
            env_path, ext,
            env_proc="UE4Editor.exe",
            ue4editor_exe_path=ue4editor_exe,
            **kwargs
        )
    elif ext == ".sln":
        run_cmds = _run_env(
            env_path, ext,
            env_proc="devenv.exe",  # Visual Studio
            devenv_exe_path=devenv_exe,
            **kwargs
        )
    elif ext == ".exe":
        run_cmds = _run_env(
            env_path, ext,
            env_proc=os.path.basename(env_path),
            **kwargs
        )
    else:
        assert False, f"Unexpected extension in env_path='{env_path}'"

    if run_cmds:
        print("Launching environment... (this may take a few seconds)")
        if verbose:
            print("\n run_cmds=" + " ".join(run_cmds) + "\n")


def _run_env(env_path, env_ext, env_proc=None, **kwargs):
    assert os.path.isfile(env_path), f"File doesn't exist, env_path='{env_path}'"

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
            return []

    run_cmds = _build_run_cmds(env_path, **kwargs)
    subprocess.Popen(run_cmds, shell=True, stdout=subprocess.PIPE)
    return run_cmds


def _build_run_cmds(
    env_path, env_ext,
    ue4editor_exe_path=None, devenv_exe_path=None,
    res=(1280, 720), windowed=True,
):
    if ext == ".uproject":
        assert os.path.isfile(ue4editor_exe_path), ue4editor_exe_path
        cmds = [ue4editor_exe_path, env_path, "-game"]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        if windowed:
            cmds.append("-windowed")

    elif ext == ".sln":
        assert os.path.isfile(devenv_path), devenv_path
        cmds = [devenv_path, env_path]

    elif ext == ".exe":
        cmds = [env_path]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        if windowed:
            cmds.append("-windowed")

    else:
        assert False, env_path

    return cmds


###############################################################################
###############################################################################

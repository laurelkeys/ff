import os
import glob
import psutil
import subprocess

###############################################################################
###############################################################################


class Default:

    ENV_ROOT_ALIASES = {
        "doc": "D:\\Documents\\AirSim",                  # .exe files
        "dev": "D:\\dev\\AirSim\\Unreal\\Environments",  # .sln and .uproject files
        "custom": "D:\\dev\\UE4\\Custom Environments",   # .sln and .uproject files
    }

    ARGS = {
        "airsim_root": "D:\\dev\\AirSim",
        "env_root": ENV_ROOT_ALIASES['doc'],
        "devenv_exe": "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\Common7\\IDE\\devenv.exe",
        "unreal_editor_exe": "D:\\Program Files\\Epic Games\\UE_4.18\\Engine\\Binaries\\Win64\\UE4Editor.exe",
    }

    AIRSIM_CLIENT_PATH = os.path.join(ARGS['airsim_root'], "PythonClient", "airsim")


###############################################################################
###############################################################################


def launch_env(args):
    if args.verbose:
        print("Possible env folders: " + ", ".join(__possible_env_folders(args.env_root)) + "\n")

    if args.env_root in Default.ENV_ROOT_ALIASES:
        args.env_root = Default.ENV_ROOT_ALIASES[args.env_root]

    if len(args.env_name) == 0:
        env_folders = __possible_env_folders(
            args.env_root, exts=["*.sln"] if args.edit else ["*.exe", "*.uproject"]
        )
        assert env_folders, f"\nno environment folder was found in '{args.env_root}'\n"
        args.env_name = env_folders[0]
        if len(env_folders) > 1:
            print("Multiple environment folders were found:")
            print(" -", "\n - ".join(env_folders))
            print("Choosing the first one:", args.env_name + "\n")
        args.env_name = os.path.basename(os.path.dirname(args.env_name))

    env_dir = os.path.join(args.env_root, args.env_name)

    if args.edit:
        __run_env(
            env_path=os.path.join(env_dir, args.env_name + ".sln"),
            env_proc="devenv.exe",  # Visual Studio
            devenv_path=args.devenv_exe,
        )
    else:
        # NOTE the folder name might be different from its .exe file name
        env_exe_path = glob.glob(os.path.join(env_dir, "*.exe"))
        if env_exe_path:
            assert (
                len(env_exe_path) == 1
            ), f"\nexpected only one .exe file in '{env_dir}', found: {env_exe_path}\n"
            __run_env(
                env_path=env_exe_path[0], env_proc=os.path.basename(env_exe_path[0]),
            )
        else:  # assume it's a .uproject file
            assert os.path.isfile(
                args.unreal_editor_exe
            ), f"\n'{args.unreal_editor_exe}' doesn't exist\n"
            __run_env(
                env_path=os.path.join(env_dir, args.env_name + ".uproject"),
                env_proc="UE4Editor.exe",
                ue4editor_path=args.unreal_editor_exe,
            )


###############################################################################
###############################################################################


def __possible_env_folders(env_root, exts=["*.exe", "*.sln", "*.uproject"]):
    env_folders = []
    for ext in exts:
        env_folders.extend(glob.glob(os.path.join(env_root, "*", ext)))
    return env_folders


def __run_env(env_path, env_proc, **kwargs):
    env_name, env_ext = os.path.splitext(os.path.basename(env_path))
    assert os.path.isfile(env_path), f"\n'{env_path}' doesn't exist\n"

    already_running = [p for p in psutil.process_iter() if p.name() == env_proc]
    if already_running:
        print(f"{env_name} is already running:")
        for p in already_running:  # there should (usually) only be one
            print(f" - name={p.name()}, pid={p.pid}")  # p.name() == env_proc
        return

    run_cmds = __build_run_cmds(env_path, **kwargs)
    # print("\n$", " ".join(run_cmds) + "\n")

    if env_ext == ".exe":
        print("Launching environment...")
    else:
        print("Launching environment... (this may take a few seconds)")

    subprocess.Popen(run_cmds, shell=True, stdout=subprocess.PIPE)


def __build_run_cmds(env_path, res=(1280, 720), ue4editor_path=None, devenv_path=None):
    _, env_ext = os.path.splitext(os.path.basename(env_path))
    assert env_ext in [".exe", ".sln", ".uproject",], f"\ninvalid extension '{env_ext}'\n"

    if env_ext == ".exe":
        cmds = [env_path]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        cmds.append("-windowed")

    elif env_ext == ".sln":
        cmds = [devenv_path, env_path]
        # TODO use Visual Studio's devenv command to run after launching

    else:  # ".uproject"
        cmds = [ue4editor_path, env_path, "-game"]
        if res is not None:
            cmds.extend([f"-ResX={res[0]}", f"-ResY={res[1]}"])
        cmds.append("-windowed")

    return cmds

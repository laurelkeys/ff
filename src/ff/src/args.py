import argparse

from .defaults import Default

###############################################################################
###############################################################################


def add_arguments_to(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--airsim_root",
        type=str,
        default=Default.ARGS["airsim_root"],
        help="AirSim directory  (default: %(default)s)",
    )

    parser.add_argument(
        "--launch",
        dest="env_name",
        metavar="ENV_NAME",
        type=str,
        nargs="?",
        const="",
        help="Name of the folder (or .uproject file) that contains the environment to run",
        # NOTE first searches for a folder called `env_name` inside `env_root`,
        #      if none are found, looks into all folders under `env_root` for a
        #      .uproject file called `env_name`, i.e.:
        #      - 1st: `env_root`/`env_name`/*.uproject
        #      - 2nd: `env_root`/*/`env_name`.uproject
    )

    parser.add_argument(
        "--from",
        dest="env_root",
        metavar="ENV_ROOT",
        type=str,
        default=Default.ARGS["env_root"],
        help="Directory that contains the environment folder  (default: %(default)s)"
             "\nAliases: {"
             + ", ".join(
                 [f"'{alias}': {env_root}" for alias, env_root in Default.ENV_ROOT_ALIASES.items()]
             )
             + "}",
    )

    parser.add_argument(
        "--edit",
        action="store_true",
        help="Launch the specified environment's .sln in Visual Studio (instead of running its .uproject file)",
    )

    parser.add_argument(
        "--devenv_exe",
        type=str,
        default=Default.ARGS["devenv_exe"],
        help="Path to Visual Studio's devenv.exe, required to launch .sln environments  (default: %(default)s)",
    )

    parser.add_argument(
        "--ue4editor_exe",
        type=str,
        default=Default.ARGS["ue4editor_exe"],
        help="Path to Unreal Engine's UE4Editor.exe, required to launch .uproject environments  (default: %(default)s)",
    )

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    return parser


###############################################################################
###############################################################################

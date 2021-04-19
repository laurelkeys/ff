import time
import argparse

import ff
import airsim

from ie.airsimy import connect

###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    assert args.env_name is not None, "missing --launch option"

    ff.launch_env(*ff.LaunchEnvArgs(args))
    ff.input_or_exit("\nPress [enter] to connect to AirSim ")

    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    client = connect(ff.SimMode.ComputerVision)
    time.sleep(1)
    client.reset()

    ff.log("Done")


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="This script launches a specified environment, connects to AirSim"
        ", sleeps for 1 second then resets the client. Use it for testing connection."
    )
    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()
    main(args)

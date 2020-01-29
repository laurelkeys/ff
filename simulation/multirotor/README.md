## Basic script template
```python
import os
import sys
import argparse
import subprocess

try:
    import airsim
except:
    pass  # don't worry, it'll be imported later

###############################################################################
###############################################################################


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    # do (awesome) stuff here
    pass


###############################################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--airsim_root",
        type=str,
        default=os.path.join("D:", "dev", "AirSim"),
        help="AirSim directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory.",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    return parser


def import_airsim(airsim_path: str, create_symbolic_link: bool = False) -> None:
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        if not os.path.exists(client_path):
            print(f"\nWARNING: expected '{client_path}' does not exist\n")

        if create_symbolic_link:
            symlink_cmd = ["ln", "-s", airsim_path, "airsim"]
            if args.verbose:
                symlink_cmd.append("--verbose")
            subprocess.run(symlink_cmd)

            airsim_client_root = os.getcwd()
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # ModuleNotFoundError will be raised if the path is incorrect


def main(args: argparse.Namespace) -> None:
    try:
        airsim_path = airsim.__path__
    except:
        airsim_path = os.path.join(args.airsim_root, "PythonClient", "airsim")
        import_airsim(airsim_path, args.symbolic_link)
    finally:
        if args.verbose:
            airsim_path_str = f"'airsim' path: {airsim.__path__[0]}"
            print("-" * len(airsim_path_str))
            print(airsim_path_str)
            print("-" * len(airsim_path_str))

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
    finally:
        client.enableApiControl(False)


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
```
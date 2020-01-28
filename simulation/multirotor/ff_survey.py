import os
import sys
import argparse
import subprocess

try:
    import airsim
except:
    pass  # don't worry, it'll be imported later

###############################################################################

def fly(client: airsim.MultirotorClient, args):
    client.takeoffAsync()
    pass

###############################################################################

def get_parser():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--airsim_root",
        type=str,
        default="D:\\dev\\AirSim",
        help="AirSim directory  (default: %(default)s)",
    )
    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory.",
    )
    parser.add_argument(
        "--disable_api_on_exit",
        action="store_true",
        help="Disable API control on exit by calling client.enableApiControl(False).",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    return parser


def import_airsim(airsim_path, symbolic_link=False):
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        if not os.path.exists(client_path):
            print(f"\nWARNING: expected '{client_path}' does not exist\n")

        if symbolic_link:
            airsim_client_root = os.getcwd()
            ln_cmd = ["ln", "-s", airsim_path, "airsim"]
            if args.verbose:
                ln_cmd += ["--verbose"]
            subprocess.run(ln_cmd)
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # NOTE this will raise another ModuleNotFoundError if the path is incorrect


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


def main(args):
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

    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
    finally:
        if args.disable_api_on_exit:
            client.enableApiControl(False)

###############################################################################

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

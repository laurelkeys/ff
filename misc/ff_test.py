import os
import sys
import argparse
import subprocess

###############################################################################


def import_airsim(
    create_symbolic_link=False, airsim_path="D:\\dev\\AirSim\\PythonClient\\airsim"
):
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        if not os.path.exists(client_path):
            print(f"\nWARNING: expected '{client_path}' does not exist\n")

        if create_symbolic_link:
            airsim_client_root = os.getcwd()
            subprocess.run(["ln", "-s", airsim_path, "airsim"])
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # NOTE this will raise another ModuleNotFoundError if the path is incorrect


###############################################################################


def __test_import_was_successful():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    try:
        landed = client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            client.takeoffAsync().join()
        else:
            print("already flying...")
            client.hoverAsync().join()

        client.armDisarm(False)
    except KeyboardInterrupt:
        client.reset()  # NOTE avoid UE4 'fatal error' when finishing the script with Ctrl+C
        # client.enableApiControl(False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--symbolic_link", "-ln", action="store_true")
    args = parser.parse_args()

    import_airsim(args.symbolic_link)

    __test_import_was_successful()

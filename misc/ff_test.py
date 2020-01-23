import os
import sys
import argparse
import subprocess

###############################################################################

def import_airsim(create_symbolic_link=False,
                  airsim_path="D:\\dev\\AirSim\\PythonClient\\airsim"):
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        if create_symbolic_link:
            try:
                subprocess.check_call(["ln", "-s", airsim_path, "airsim"])
                sys.path.insert(0, os.getcwd())
            except subprocess.CalledProcessError:
                print(f"\nInvalid path to link to 'airsim' module: '{airsim_path}'\n")
        else:
            client_path = os.path.join(airsim_path, "client.py")
            if os.path.exists(client_path):
                sys.path.insert(0, os.path.dirname(airsim_path))
            else:
                print(f"\nInvalid path to 'airsim' module: '{os.path.dirname(airsim_path)}'"
                      f"\nCould not find: '{client_path}'\n")
        import airsim  # NOTE this will raise a ModuleNotFoundError if the path is incorrect

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

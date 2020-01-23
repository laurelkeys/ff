import os
import sys
import atexit
import argparse

try:
    import airsim

except ModuleNotFoundError as e:
    # TODO use argparse to create a symbolic link if the option '-ln' is passed
    airsim_path = "D:\\dev\\AirSim\\PythonClient\\airsim"
    client_path = os.path.join(airsim_path, "client.py")
    if os.path.exists(client_path):
        sys.path.insert(0, os.path.dirname(airsim_path))
    else:
        print(f"\nModuleNotFoundError: {e}\n")

    import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# reset the drone if we close the script with Ctrl+C
atexit.register(lambda: client.reset()) # FIXME UE4 is still raising 'fatal error'

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()

client.armDisarm(False)
# client.enableApiControl(False)

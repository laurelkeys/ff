import os
import sys
import logging

airsim_path = "D:\\dev\\AirSim\\PythonClient\\airsim"

client_path = os.path.join(airsim_path, "client.py")

if os.path.exists(client_path):
    sys.path.insert(0, os.path.dirname(airsim_path))
else:
    logging.warning(f"\n'airsim' module not found in folder '{os.path.dirname(airsim_path)}'\n")

# NOTE another option would be to create a symbolic link in the script directory pointing to the
#      AirSim module's path, since Python always places the script directory in sys.path, e.g.:
#      $ ln -s D:\dev\AirSim\PythonClient\airsim airsim
#
# import subprocess
# subprocess.run(["ln", "-s", airsim_path, "airsim"])

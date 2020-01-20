import os
import sys
import logging

airsim_path = "D:\\dev\\AirSim\\PythonClient\\airsim"

client_path = os.path.join(airsim_path, "client.py")

if os.path.exists(client_path):
    sys.path.insert(0, os.path.dirname(airsim_path))
else:
    logging.warning(f"'airsim' module not found in folder '{os.path.dirname(airsim_path)}'")
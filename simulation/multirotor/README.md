## Basic script template
```python
import os
import sys
import argparse

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_CLIENT_PATH)
    import airsim


###############################################################################
# preflight (called before connecting) ########################################
###############################################################################

def preflight(args: argparse.Namespace) -> None:
    # setup before connecting to AirSim
    pass

###############################################################################
# fly (called after connecting) ###############################################
###############################################################################

def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    # do (awesome) stuff here
    
    initial_pose = client.simGetVehiclePose()
    if args.verbose:
        ff.print_pose(initial_pose, airsim.to_eularian_angles)

    initial_state = client.getMultirotorState()
    if initial_state.landed_state == airsim.LandedState.Landed:
        print("[ff] Taking off")
        client.takeoffAsync(timeout_sec=8).join()
    else:
        client.hoverAsync().join() # airsim.LandedState.Flying

    client.reset()
    print("[ff] Drone reset")

###############################################################################
# main ########################################################################
###############################################################################

def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    if args.env_name is not None:
        ff.launch_env(args) # the --launch option was passed
        input("\nPress [enter] to connect to AirSim ")

    preflight(args) # setup
    client = connect_to_airsim()
    try:
        fly(client, args) # do stuff
    except KeyboardInterrupt:
        client.reset() # avoid UE4 'fatal error' when exiting with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset

def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client

###############################################################################
# argument parsing ############################################################
###############################################################################

def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="")

    # parser.add_argument(...)
    # ...

    ff.add_arguments_to(parser)
    return parser

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)
```
import os
import sys
import argparse

import ff

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim


###############################################################################
## preflight (called before connecting) #######################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    # setup before connecting to AirSim
    pass


###############################################################################
## fly (called after connecting) ##############################################
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
        client.hoverAsync().join()  # airsim.LandedState.Flying

    client.reset()
    print("[ff] Drone reset")


def get_xyz_xyzw_from(client_or_image_response, is_header_line=False):
    """ Reimplementation of AirSim's recording function.
        See: AirSim/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp#L554
    """
    if is_header_line:
        # NOTE AirSim uses WXYZ for quaternions, here we return XYZW
        #      we also omit the TimeStamp value as the first argument
        return "POS_X\tPOS_Y\tPOS_Z\tQ_X\tQ_Y\tQ_Z\tQ_W\t"

    if isinstance(client := client_or_image_response, airsim.MultirotorClient):
        kinematics = client.simGetGroundTruthKinematics()
        position = kinematics.position
        orientation = kinematics.orientation

    elif isinstance(image_response := client_or_image_response, airsim.ImageResponse):
        position = image_response.camera_position
        orientation = image_response.camera_orientation

    else:
        assert False, type(client_or_image_response)

    return [
        position.to_numpy_array(),  # [x, y, z]
        orientation.to_numpy_array()  # [x, y, z, w]
    ]

###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        ff.print_airsim_path(airsim.__path__)

    if args.env_name is not None:
        # the --launch option was passed
        ff.launch_env(*ff.LaunchEnvArgs(args))
        input("\nPress [enter] to connect to AirSim ")

    preflight(args)  # setup
    client = connect_to_airsim()
    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset


def connect_to_airsim() -> airsim.MultirotorClient:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fly viewpoints capturing camera position and orientation."
    )
    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

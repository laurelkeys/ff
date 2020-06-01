import os
import sys
import msvcrt
import argparse

import ff

from wrappers import OrbitNavigator

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

    take_pictures_loop(client)

    client.reset()
    print("[ff] Drone reset")


def take_pictures_loop(client):
    print("[ff] Press [space] to take pictures (or any other key to stop)")
    img_count = 0
    while True:
        if msvcrt.kbhit():
            if msvcrt.getch() != b" ":
                break  # https://stackoverflow.com/a/13207813

            response, *_ = client.simGetImages(
                [airsim.ImageRequest(ff.CameraName.front_center, airsim.ImageType.Scene)]
            )

            img_count += 1
            print(f"     {img_count} pictures taken", end="\r")

            airsim.write_file(
                os.path.join(args.output_folder, f"out_{img_count}.png"),
                response.image_data_uint8,
            )

            print(get_record_line_from(response))

    print()


def get_record_line_from(client_or_image_response):
    """ Modified implementation of AirSim's recording function.
        See: AirSim/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp#L554
    """
    # NOTE AirSim uses WXYZ for quaternions, here we return XYZW, also,
    #      while it uses the client's pose, there's a slight difference
    #      in the `position` captured by image_response, owing to the camera,
    #      and there can be large ones in `orientation` when using MaxDegreeOfFreedom

    if isinstance(client := client_or_image_response, airsim.MultirotorClient):
        state = client.getMultirotorState()
        return state.timestamp, *ff.xyz_xyzw_of_client(state)

    elif isinstance(image_response := client_or_image_response, airsim.ImageResponse):
        return image_response.time_stamp, *ff.xyz_xyzw_of_image(image_response)

    else:
        assert False, f"{type(client_or_image_response)=}"


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
    parser = argparse.ArgumentParser(description="")

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

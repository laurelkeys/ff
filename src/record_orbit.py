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

    navigator = OrbitNavigator(
        client,
        args.radius,
        args.altitude,
        args.speed,
        args.iterations,
        [float(_) for _ in args.center.split(',')],
        args.snapshots
    )
    navigator.start()

    take_pictures(client)

    client.reset()
    print("[ff] Drone reset")


def take_pictures(client):
    print("[ff] Press [space] to take pictures (or any other key to stop)")  # TODO
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
            # TODO save poses to .log file
            print("camera_position:", response.camera_position)  # Vector3r
            print("camera_orientation:", response.camera_orientation)  # Quaternionr
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
    parser = argparse.ArgumentParser(
        description="Fly in a circular orbit, capturing camera position (XYZ vector) and orientation (XYZW quaternion)."
    )

    parser.add_argument(
        "--radius", type=float, default=5,
        help="Radius in [m]  (default: %(default)d)"
    )
    parser.add_argument(
        "--altitude", type=float, default=1.5,
        help="Altitude in positive [m]  (default: %(default)d)"
    )
    parser.add_argument(
        "--speed", type=float, default=3,
        help="Speed in [m/s]  (default: %(default)d)"
    )
    parser.add_argument(
        "--center", default="0,-1",
        help="Direction vector x,y pointing to center of orbit  (default: %(default)s)"
    )
    parser.add_argument(
        "--iterations", type=float, default=2,
        help="Number of 360 degree orbits  (default: %(default)d)"
    )
    parser.add_argument(
        "--snapshots", type=float, default=0,
        help="Number of FPV snapshots to take during orbit  (default: %(default)d)"
    )

    ff.add_arguments_to(parser)
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

import os
import sys
import time
import argparse
import subprocess

from typing import Tuple

try:
    import airsim
except ModuleNotFoundError:
    pass  # don't worry, it'll be imported later

###############################################################################
###############################################################################


def preflight(args: argparse.Namespace) -> None:
    # setup before connecting to AirSim
    pass


###############################################################################
###############################################################################


def pose2str(pose: airsim.Pose) -> str:
    position_str = "position=({:.2f} {:.2f} {:.2f})".format(
        pose.position.x_val, pose.position.y_val, pose.position.z_val
    )
    orientation_str = "orientation=({:.2f} {:.2f} {:.2f} {:.2f})".format(
        pose.orientation.w_val,
        pose.orientation.x_val,
        pose.orientation.y_val,
        pose.orientation.z_val,
    )
    return f"Pose({position_str}, {orientation_str})"


def pose2args(pose: airsim.Pose) -> str:
    return "-x {:.2f} -y {:.2f} -z {:.2f}".format(
        pose.position.x_val, pose.position.y_val, pose.position.z_val
    )


def fly(client: airsim.MultirotorClient, args: argparse.Namespace) -> None:
    if args.z is not None and args.z > 0:
        print("Warning: AirSim uses NED coordinates, meaning +z is down")
        print("         (use negative z values to fly upwards)\n")

    scene_objects = client.simListSceneObjects(
        name_regex=args.name
    )  # use '.*' to list all

    if args.list_all:
        print(client.simListSceneObjects())

    if not scene_objects:
        print(f"[ff] No object found with name '{args.name}'")
        print(
            "[ff] Object names containing it:",
            client.simListSceneObjects(f".*{args.name}.*"),
        )
    elif len(scene_objects) > 1:
        print(f"[ff] Multiple objects found with name '{args.name}':", scene_objects)
    else:
        object_name = scene_objects[0]
        start_pose = client.simGetObjectPose(object_name)
        print("[ff] Starting pose:", pose2args(start_pose))
        if args.verbose:
            print("    ", pose2str(start_pose))

        new_pose = start_pose
        if args.absolute:
            new_pose.position.x_val = args.x
            new_pose.position.y_val = args.y
            new_pose.position.z_val = args.z
        else:
            new_pose.position.x_val += args.x
            new_pose.position.y_val += args.y
            new_pose.position.z_val += args.z
        print(f"[ff] Moving object '{object_name}'", end="")
        success = client.simSetObjectPose(object_name, pose=new_pose)
        print(" (successful)" if success else " (failed)")

        end_pose = client.simGetObjectPose(object_name)
        print("[ff] Ending pose:", pose2args(end_pose))
        if args.verbose:
            print("    ", pose2str(end_pose))

        if not success:
            print("\nWarning: Make sure the object has Mobility = 'Movable' in UE4")


###############################################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Change scene object's position (uses NED coordinates)"
        # NOTE pose is in NED coordinates in SI units with its origin at Player Start
    )

    parser.add_argument(
        "name",
        type=str,
        default="Cube_BBox",
        help="Name of a scene object with Mobility = Movable  (default: %(default)s)",
        # NOTE https://docs.unrealengine.com/en-US/Engine/Actors/Mobility/index.html
    )

    parser.add_argument(
        "-x",
        type=float,
        default=0.0,
        help="Coordinate value in meters, where +x is north",
    )
    parser.add_argument(
        "-y",
        type=float,
        default=0.0,
        help="Coordinate value in meters, where +y is east",
    )
    parser.add_argument(
        "-z",
        type=float,
        default=0.0,
        help="Coordinate value in meters, where +z is down",
    )

    parser.add_argument(
        "--absolute",
        action="store_true",
        help="Use coordinate values as absolute positions,"
        " instead of relative offsets to the current object location",
    )

    parser.add_argument(
        "--list_all",
        action="store_true",
        help="List all scene objects",
    )

    parser.add_argument(
        "--airsim_root",
        type=str,
        default=os.path.join("D:", "dev", "AirSim"),
        help="AirSim directory  (default: %(default)s)",
    )

    parser.add_argument(
        "--symbolic_link",
        "-ln",
        action="store_true",
        help="Create a symbolic link to AirSim in the current directory.",
    )

    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Increase verbosity"
    )
    return parser


def import_airsim(airsim_path: str, create_symbolic_link: bool = False) -> None:
    global airsim
    try:
        import airsim
    except ModuleNotFoundError:
        client_path = os.path.join(airsim_path, "client.py")
        assert os.path.exists(
            client_path
        ), f"\nexpected '{client_path}' doesn't exist\n"

        if create_symbolic_link:
            symlink_cmd = ["ln", "-s", airsim_path, "airsim"]
            if args.verbose:
                symlink_cmd.append("--verbose")
            subprocess.run(symlink_cmd)

            airsim_client_root = os.getcwd()
        else:
            airsim_client_root = os.path.dirname(airsim_path)

        sys.path.insert(0, airsim_client_root)
        import airsim  # ModuleNotFoundError will be raised if the path is incorrect


def main(args: argparse.Namespace) -> None:
    try:
        airsim_path = airsim.__path__
    except NameError:
        airsim_path = os.path.join(args.airsim_root, "PythonClient", "airsim")
        import_airsim(airsim_path, create_symbolic_link=args.symbolic_link)
    finally:
        if args.verbose:
            path_str = f"'airsim' path: {airsim.__path__[0]}"
            print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    preflight(args)  # setup

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    try:
        fly(client, args)  # do stuff
    except KeyboardInterrupt:
        client.reset()  # avoid UE4 'fatal error' when exiting the script with Ctrl+C
        # NOTE client.enableApiControl(True) must be called after reset


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

import os
import sys
import json
import time
import msvcrt
import argparse

from ff import Default
from ff.types import to_xyz_tuple, to_xyzw_tuple, xyz_to_str, xyzw_to_str, angles_to_str

try:
    import airsim
except ModuleNotFoundError:
    airsim_path = Default.AIRSIM_CLIENT_PATH
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))
    import airsim


SETTINGS = "D:\\Documents\\AirSim\\settings.json"


def main(args: argparse.Namespace) -> None:
    # change SimMode
    with open(SETTINGS, "r+") as settings_file:
        settings = json.load(settings_file)
        sim_mode = settings["SimMode"]
        if sim_mode != "ComputerVision":
            print(f"\nChanging SimMode from '{sim_mode}' to 'ComputerVision'\n")
            settings["SimMode"] = "ComputerVision"

    client = airsim.MultirotorClient()
    client.confirmConnection()

    positions = []
    orientations = []
    print(f"[ff] Press [space] to store points (and [backspace] to quit)")

    ch = msvcrt.getch()
    while ch != b"\x08":
        if ch == b" ":
            pose = client.simGetVehiclePose()
            xyz = to_xyz_tuple(pose.position)
            xyzw = to_xyzw_tuple(pose.orientation)
            angles = airsim.to_eularian_angles(pose.orientation)  # (pitch, roll, yaw) tuple

            positions.append(xyz)
            orientations.append(angles if args.store_angles else xyzw)

            print(
                f"     Added position={xyz_to_str(xyz)}, "
                f"orientation={angles_to_str(angles) if args.store_angles else xyzw_to_str(xyzw)}",
                end="\r",
            )

        ch = msvcrt.getch()
    print()

    str_positions = ", ".join([xyz_to_str(position, 4, False) for position in positions])
    str_orientations = ", ".join(
        [
            angles_to_str(orientation, 6, False)
            if args.store_angles
            else xyzw_to_str(orientation, 6, False)
            for orientation in orientations
        ]
    )

    print(f"\nPositions: [{str_positions}]")
    print(f"\nOrientations: [{str_orientations}]")

    if args.output_file is not None:
        with open(args.output_file, "w") as f:
            f.write(f"Positions: [{str_positions}]")
            f.write(f"\nOrientations: [{str_orientations}]")

    # restore SimMode
    if sim_mode != "ComputerVision":
        settings["SimMode"] = sim_mode
        with open(SETTINGS, "r+") as settings_file:
            json.dump(settings, settings_file, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture viewpoints in 'ComputerVision' mode")
    parser.add_argument(
        "--store_angles",
        action="store_true",
        help="Store (pitch, roll, yaw) instead of quaternions for orientation",
    )
    parser.add_argument(
        "--output_file", type=str, help="Output file path to save the viewpoints to",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    args = parser.parse_args()

    main(args)

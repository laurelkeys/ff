import msvcrt
import argparse

import ff

from ff.types import xyz_to_str, xyzw_to_str, to_xyz_tuple, angles_to_str, to_xyzw_tuple

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_CLIENT_PATH)
    import airsim


def main(args):
    assert (
        ff.curr_sim_mode() == ff.SimMode.ComputerVision
    ), f"SimMode must be '{ff.SimMode.ComputerVision}'"

    client = airsim.MultirotorClient()
    client.confirmConnection()

    positions, orientations = [], []
    print(f"[ff] Press [space] to store points (and any other key to quit)\n")

    while True:
        if msvcrt.kbhit():
            if msvcrt.getch() != b" ":
                break  # https://stackoverflow.com/a/13207813

            pose = client.simGetVehiclePose()
            xyz = to_xyz_tuple(pose.position)
            xyzw = to_xyzw_tuple(pose.orientation)
            angles = airsim.to_eularian_angles(pose.orientation)  # (pitch, roll, yaw)

            positions.append(xyz)
            orientations.append(angles if args.store_angles else xyzw)

            orientation_str = angles_to_str(angles) if args.store_angles else xyzw_to_str(xyzw)
            print(f"     Added position={xyz_to_str(xyz)}, orientation={orientation_str}")
    print()

    str_positions = ", ".join(
        [xyz_to_str(position, 4, show_hints=False) for position in positions]
    )
    str_orientations = ", ".join(
        [
            angles_to_str(orientation, 6, show_hints=False)
            if args.store_angles
            else xyzw_to_str(orientation, 6, show_hints=False)
            for orientation in orientations
        ]
    )

    print(f"\n[ff] Done", end="")
    if args.output_file is not None:
        with open(args.output_file, "w") as f:
            f.write(f"Positions = [{str_positions}]")
            f.write(f"\nOrientations = [{str_orientations}]")
        print(f" (output saved to {args.output_file})")
    else:
        print(f"\n\nPositions = [{str_positions}]")
        print(f"\n\nOrientations = [{str_orientations}]")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture viewpoints in 'ComputerVision' mode")
    parser.add_argument(
        "--store_angles",
        "-a",
        action="store_true",
        help="Store (pitch, roll, yaw) instead of quaternions for orientation",
    )
    parser.add_argument(
        "--output_file", "-o", type=str, help="Output file path to save the viewpoints to",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    args = parser.parse_args()

    main(args)

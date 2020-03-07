import os, sys, time, msvcrt, argparse
from typing import List

import ff

try:
    import airsim
except ModuleNotFoundError:
    airsim_path = ff.Default.AIRSIM_CLIENT_PATH
    assert os.path.exists(os.path.join(airsim_path, "client.py")), airsim_path
    sys.path.insert(0, os.path.dirname(airsim_path))
    import airsim


def to_xyz_tuple(vector3r):
    return (vector3r.x_val, vector3r.y_val, vector3r.z_val)


def to_xyzw_tuple(quaternionr):
    return (quaternionr.x_val, quaternionr.y_val, quaternionr.z_val, quaternionr.w_val)


def xyz_to_str(xyz, n=2, show_hints=True):
    if show_hints:
        return f"(x={xyz[0]:.{n}f}, y={xyz[1]:.{n}f}, z={xyz[2]:.{n}f})"
    return f"({xyz[0]:.{n}f}, {xyz[1]:.{n}f}, {xyz[2]:.{n}f})"


def xyzw_to_str(xyzw, n=2, show_hints=True):
    if show_hints:
        return f"(x={xyzw[0]:.{n}f}, y={xyzw[1]:.{n}f}, z={xyzw[2]:.{n}f}, w={xyzw[3]:.{n}f})"
    return f"({xyzw[0]:.{n}f}, {xyzw[1]:.{n}f}, {xyzw[2]:.{n}f}, {xyzw[3]:.{n}f})"


def angles_to_str(angles, n=4, show_hints=True):
    if show_hints:
        return f"(pitch={angles[0]:.{n}f}, roll={angles[1]:.{n}f}, yaw={angles[2]:.{n}f})"
    return f"({angles[0]:.{n}f}, {angles[1]:.{n}f}, {angles[2]:.{n}f})"


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        path_str = f"'airsim' path: {airsim.__path__[0]}"
        print("-" * len(path_str), path_str, "-" * len(path_str), sep="\n")

    # TODO check (and possibly change) settings.json mode to ComputerVision

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

    str_positions = ", ".join(
        [xyz_to_str(position, 4, False) for position in positions]
    )
    str_orientations = ", ".join(
        [angles_to_str(orientation, 6, False) if args.store_angles else xyzw_to_str(orientation, 6, False) for orientation in orientations]
    )

    print(f"\nPositions: [{str_positions}]")
    print(f"\nOrientations: [{str_orientations}]")

    if args.output_file is not None:
        with open(args.output_file, 'w') as f:
            f.write(f"Positions: [{str_positions}]")
            f.write(f"\nOrientations: [{str_orientations}]")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--store_angles",
        action="store_true",
        help="Store (pitch, roll, yaw) instead of quaternions for orientation",
    )
    parser.add_argument(
        "--output_file",
        type=str,
        help="Save viewpoints to output file",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")
    args = parser.parse_args()

    main(args)

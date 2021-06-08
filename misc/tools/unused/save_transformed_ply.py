import argparse

from io_ply import read_ply, write_ply

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modify a given point cloud and save it.")
    parser.add_argument("input_path", type=str, help="Path to a .PLY file")
    parser.add_argument("output_path", type=str, help="Path to save the modified .PLY file")
    parser.add_argument("--offset", type=float, nargs=3, help="Offset added to all points ")
    parser.add_argument("--scale", type=float, help="Scale added to all points ")
    args = parser.parse_args()

    if args.offset is not None or args.scale is not None:
        input_ply_df = read_ply(args.input_path)
        assert "mesh" not in input_ply_df

        comments = f"Generated with save_transformed_ply.py from {args.input_path}"

        tx, ty, tz = [0, 0, 0] if args.offset is None else args.offset
        s = 1.0 if args.scale is None else args.scale

        # NOTE see uavmvs_parse_traj.py's convert_uavmvs_to_airsim_position()
        ty = -ty
        tz = -tz

        print(input_ply_df["points"][["x", "y", "z"]])
        input_ply_df["points"][["x"]] = input_ply_df["points"][["x"]].apply(lambda x: (x + tx) * s)
        input_ply_df["points"][["y"]] = input_ply_df["points"][["y"]].apply(lambda y: (y + ty) * s)
        input_ply_df["points"][["z"]] = input_ply_df["points"][["z"]].apply(lambda z: (z + tz) * s)
        print(input_ply_df["points"][["x", "y", "z"]])

        write_ply(args.output_path, points=input_ply_df["points"], as_text=False, comments=comments)

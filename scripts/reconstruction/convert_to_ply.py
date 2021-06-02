import os
import json
import argparse

from typing import List

import numpy as np
import open3d as o3d


def convert_to_ply(input, output, np_points_from_lines_fn):
    assert os.path.isfile(input), f"Invalid input path: '{input}'"

    if output is None:
        output = f"{os.path.splitext(os.path.basename(input))[0]}.ply"
    elif not output.endswith(".ply"):
        assert os.path.isdir(output), f"Invalid output path: '{output}'"
        output = os.path.join(output, f"{os.path.splitext(os.path.basename(input))[0]}.ply")

    # Parse the camera poses into a `float64` array of shape `(num_points, 3)`.
    with open(input, "r") as f:
        np_points = np_points_from_lines_fn(f.readlines())

    # NOTE to convert data points between numpy and open3d use:
    #   |
    #   |   # from numpy to open3d
    #   |   pcd.points = open3d.utility.Vector3dVector(np_points)
    #   |
    #   |   # from open3d to numpy
    #   |   np_points = np.asarray(pcd.points)
    #
    # http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
    # http://www.open3d.org/docs/release/python_api/open3d.utility.html#open3d-utility

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np_points))

    return pcd, output


def np_points_from_airsim_rec(lines: List[str]) -> np.ndarray:
    def np_pos_from(line: str) -> np.ndarray:
        _, pos_x, pos_y, pos_z, *_ = line.rstrip("\n").split("\t")
        return np.fromiter(map(float, [pos_x, pos_y, pos_z]), dtype=float)

    # NOTE skip the header "TimeStamp POS_X POS_Y POS_Z Q_W Q_X Q_Y Q_Z ImageFile"
    return np.array([np_pos_from(line) for line in lines[1:]], dtype=float)


def np_points_from_cameras_sfm(lines: List[str]) -> np.ndarray:
    def np_pos_from(pose: dict) -> np.ndarray:
        center = pose["pose"]["transform"]["center"]
        return np.fromiter(map(float, center), dtype=float)

    poses_json = json.loads("".join(lines))["poses"]
    return np.array([np_pos_from(pose) for pose in poses_json])


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    point_clouds = []

    def save_ply(input, output, np_points_from_lines_fn):
        nonlocal point_clouds
        pcd, output_path = convert_to_ply(input, output, np_points_from_lines_fn)
        point_clouds.append(pcd)
        o3d.io.write_point_cloud(output_path, pcd)
        print(f"{pcd} Saved to '{output_path}'.")

    if args.rec is not None:
        save_ply(args.rec, args.ply or "converted_rec.ply", np_points_from_airsim_rec)

    if args.sfm is not None:
        save_ply(args.sfm, args.ply or "converted_sfm.ply", np_points_from_cameras_sfm)

    if point_clouds and args.view:
        o3d.visualization.draw_geometries(point_clouds)


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Converts the camera positions from AirSim's recording log"
        " or Meshroom's reconstruction file to point cloud data in PLY format."
    )

    parser.add_argument("--rec", type=str, help="Path to the input airsim_rec.txt")
    parser.add_argument("--sfm", type=str, help="Path to the input cameras.sfm")
    parser.add_argument("--ply", type=str, help="Path to the output file")
    parser.add_argument("--view", action="store_true", help="Visualize the point clouds")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)


# NOTE some useful methods of `PointCloud` are:
#
# PointCloud.rotate(R, center) – Apply rotation to the geometry coordinates and normals.
#   R (numpy.ndarray[float64[3, 3]]) – The rotation matrix
#   center (numpy.ndarray[float64[3, 1]]) – Rotation center used for transformation
#
# PointCloud.scale(scale, center) – Apply scaling to the geometry coordinates.
#   scale (float) – The scale parameter that is multiplied to the points/vertices of the geometry
#   center (numpy.ndarray[float64[3, 1]]) – Scale center used for transformation
#
# http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.transform
# http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.translate
# http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.uniform_down_sample

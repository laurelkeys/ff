import os
import argparse
from typing import Optional, Tuple

import numpy as np
import open3d as o3d


def no_ext_basename(p):
    return os.path.splitext(os.path.basename(p))[0]


def convert_to_ply(input, output, np_pos_from_line):
    assert os.path.isfile(input), f"Invalid input path: '{input}'"

    if output is None:
        output = f"{no_ext_basename(input)}.ply"
    elif not output.endswith(".ply"):
        assert os.path.isdir(output), f"Invalid output path: '{output}'"
        output = os.path.join(output, f"{no_ext_basename(input)}.ply")

    # Parse the camera poses into a `float64` array of shape `(num_points, 3)`.
    with open(input, "r") as f:
        # NOTE skip the column header "TimeStamp POS_X POS_Y POS_Z Q_W Q_X Q_Y Q_Z ImageFile"
        np_points = np.array([np_pos_from_line(line) for line in f.readlines()[1:]], dtype=float)

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


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    point_clouds = []

    if args.rec is not None:
        def np_pos_from_airsim(line: str) -> np.ndarray:
            _, pos_x, pos_y, pos_z, *_ = line.rstrip("\n").split("\t")
            return np.fromiter(map(float, [pos_x, pos_y, pos_z]), dtype=float)

        rec_pcd, rec_output_path = convert_to_ply(args.rec, args.ply, np_pos_from_airsim)

        point_clouds.append(rec_pcd)
        o3d.io.write_point_cloud(rec_output_path, rec_pcd)
        print(f"{rec_pcd} Saved to '{rec_output_path}'.")

    if args.sfm is not None:
        def np_pos_from_meshroom(line: str) -> np.ndarray:
            raise NotImplementedError

        # NOTE if args.rec is not None and args.ply.endswith(".ply")
        #      then the output PLY file will be overwritten... FIXME
        sfm_pcd, sfm_output_path = convert_to_ply(args.sfm, args.ply, np_pos_from_meshroom)

        point_clouds.append(sfm_pcd)
        o3d.io.write_point_cloud(sfm_output_path, sfm_pcd)
        print(f"{sfm_pcd} Saved to '{sfm_output_path}'.")

    if point_clouds:
        o3d.visualization.draw_geometries(point_clouds)


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Converts the camera positions from an AirSim recording log,"
        " or from a Meshroom reconstruction file, to point cloud data in PLY format."
    )

    parser.add_argument("--rec", type=str, help="Path to the input airsim_rec.txt")
    parser.add_argument("--sfm", type=str, help="Path to the input cameras.sfm")
    parser.add_argument("--ply", type=str, help="Path to the output file")

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

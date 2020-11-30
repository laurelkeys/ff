import os
import argparse

from enum import Enum

import open3d as o3d


def no_ext_basename(p):
    return os.path.splitext(os.path.basename(p))[0]


class SamplingMethod(Enum):
    Uniform = "uniform"
    Poisson = "poisson"


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    input_path = args.triangle_mesh
    assert os.path.isfile(input_path), f"Invalid input path: '{input_path}'"
    triangle_mesh = o3d.io.read_triangle_mesh(input_path)

    print("> Triangle mesh")
    print(f"  {len(triangle_mesh.vertices)} vertices")
    print(f"  {len(triangle_mesh.triangles)} triangles")
    if args.view:
        o3d.visualization.draw_geometries([triangle_mesh], mesh_show_wireframe=True)

    # Set the number of points to be sampled
    number_of_points = args.points_per_triangle * len(triangle_mesh.triangles)

    if args.method == SamplingMethod.Uniform.value:
        point_cloud = triangle_mesh.sample_points_uniformly(number_of_points)
    elif args.method == SamplingMethod.Poisson.value:
        point_cloud = triangle_mesh.sample_points_poisson_disk(
            number_of_points,
            init_factor=args.init_factor,
            # FIXME `init_factor * number_of_points` is the number of points of an initial
            # uniformly sampled point cloud, that will then be used for sample elimination
        )
    else:
        assert False, args.method

    print("> Point cloud")
    print(f"  {len(point_cloud.points)} points")  # == number_of_points
    if args.view:
        o3d.visualization.draw_geometries([triangle_mesh, point_cloud], mesh_show_wireframe=True)
        o3d.visualization.draw_geometries([point_cloud])

    # Save the sampled point cloud in PLY format
    output_path = args.point_cloud

    if output_path is None:
        output_path = os.path.dirname(input_path)
    if not output_path.endswith(".ply"):
        assert os.path.isdir(output_path), f"Invalid output path: '{output_path}'"
        output_path = os.path.join(output_path, f"{no_ext_basename(input_path)}_point_cloud.ply")

    o3d.io.write_point_cloud(output_path, point_cloud)


###############################################################################
## argument parsing ###########################################################
###############################################################################


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Converts a 3D triangle mesh to a point cloud.")

    parser.add_argument("--triangle_mesh", "-tm", type=str, help="Path to the input PLY file")
    parser.add_argument("--point_cloud", "-pc", type=str, help="Path to the output PLY file")
    parser.add_argument(
        "--points_per_triangle",
        type=int,
        default=3,
        # FIXME this is only accurate for uniform sample if we have similar-sized triangles
        help="Average number of points in the final point cloud per triangle in the mesh  (default: %(default)d)",
    )
    parser.add_argument(
        "--method",
        type=str,
        default=SamplingMethod.Uniform.value,
        choices=[_.value for _ in SamplingMethod],
        help="Function used to sample points from the mesh  (default: %(default)s)",
    )
    parser.add_argument(
        "--init_factor",
        type=int,
        default=2,
        help="Factor that multiplies the average number of points per triangle for an initial uniformly sampled"
             " point cloud that is used for sample elimination when method='poisson'  (default: %(default)d)",
    )
    parser.add_argument("--view", action="store_true", help="Visualize the mesh and point cloud")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)


# http://www.open3d.org/docs/release/tutorial/geometry/mesh.html#Sampling
# http://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html#open3d.geometry.TriangleMesh.sample_points_poisson_disk
# http://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html#open3d.geometry.TriangleMesh.sample_points_uniformly

import os
import argparse

from enum import Enum

import open3d as o3d


class SamplingMethod(Enum):
    # http://www.open3d.org/docs/release/tutorial/geometry/mesh.html#Sampling
    Uniform = "uniform"  # http://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html#open3d.geometry.TriangleMesh.sample_points_uniformly
    Poisson = "poisson"  # http://www.open3d.org/docs/release/python_api/open3d.geometry.TriangleMesh.html#open3d.geometry.TriangleMesh.sample_points_poisson_disk


###############################################################################
## main #######################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    if args.verbose:
        print(f"points_per_triangle = {args.points_per_triangle}")
        if args.method == SamplingMethod.Poisson.value:
            print(f"method = {args.method} (init_factor = {args.init_factor})")
        else:
            print(f"method = {args.method}")
        print()

    input_path = args.triangle_mesh
    assert os.path.isfile(input_path), f"Invalid input path: '{input_path}'"
    triangle_mesh = o3d.io.read_triangle_mesh(input_path)

    print("Triangle mesh")
    print(f"> {len(triangle_mesh.vertices)} vertices")
    print(f"> {len(triangle_mesh.triangles)} triangles")
    if args.view:
        o3d.visualization.draw_geometries([triangle_mesh], mesh_show_wireframe=True)

    # Set the number of points to be sampled
    number_of_points = args.points_per_triangle * len(triangle_mesh.triangles)

    if args.method == SamplingMethod.Uniform.value:
        point_cloud = triangle_mesh.sample_points_uniformly(number_of_points)
    elif args.method == SamplingMethod.Poisson.value:
        if args.verbose:
            print("\nPerforming sample elimination for Poisson Disk Sampling..")
        # NOTE `init_factor * number_of_points` is the number of points of an initial
        # uniformly sampled point cloud that will then be used for sample elimination
        point_cloud = triangle_mesh.sample_points_poisson_disk(
            number_of_points, init_factor=args.init_factor
        )
    else:
        assert False, args.method

    print("\nPoint cloud")
    print(f"> {len(point_cloud.points)} points")  # == number_of_points
    if args.view:
        o3d.visualization.draw_geometries([triangle_mesh, point_cloud], mesh_show_wireframe=True)
        o3d.visualization.draw_geometries([point_cloud])

    # Save the sampled point cloud in PLY format
    output_path = args.point_cloud

    if output_path is None:
        output_path = os.path.dirname(input_path)
    if not output_path.endswith(".ply"):
        assert os.path.isdir(output_path), f"Invalid output path: '{output_path}'"
        output_path = os.path.join(
            output_path, f"{os.path.splitext(os.path.basename(input_path))[0]}_point_cloud.ply"
        )

    o3d.io.write_point_cloud(output_path, point_cloud, write_ascii=True)
    print(f"\nSaved output to '{output_path}'")


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
        help="Factor by which the average number of points per triangle is multiplied to create an initial uniformly"
        " sampled point cloud that is then used for sample elimination when method='poisson'  (default: %(default)d)",
    )
    parser.add_argument("--view", action="store_true", help="Visualize the mesh and point cloud")
    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    main(args)

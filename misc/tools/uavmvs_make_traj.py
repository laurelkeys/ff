import os
import argparse


# ASPECT = 0.66  # = 4000 / 6000
# FOCAL = 0.867  # = 15 / 17.3

ALT  =  15  # flying altitude
ELEV = -15  # elevation for overlap planning
F = 80      # forward overlap percentage
S = 80      # side overlap percentage

MAX_D = 30  # maximum distance to surface (optimize_trajectory)
MIN_D =  5  # minimum distance to surface (optimize_trajectory)

MAX_DR = 35 # maximum distance to surface (evaluate_trajectory)


def add_opt(args, arg, name, is_bool=False):
    if arg is not None:
        arg = ",".join(map(str, arg)) if isinstance(arg, list) else arg
        args.append(f"--{name}" + ("" if is_bool else f"={arg}"))


def run_app(app, *args):
    run_str = f"{app} {' '.join(map(str, *args))}"
    print(f"\n./{run_str}")  # XXX os.system(run_str)


def _convert_mesh(
    in_mesh, out_mesh,
    transform=None, invert=None, clean=None,
    show_info=None, ascii=None,
    delete_faces=None, delete_vnormals=None, delete_vcolors=None, delete_values=None, delete_confidences=None,
):
    """ Conversion app for meshes a la image magicks convert. """
    args = [in_mesh, out_mesh]

    add_opt(args, transform, "transform")
    add_opt(args, invert, "invert", is_bool=True)
    add_opt(args, clean, "clean", is_bool=True)
    add_opt(args, show_info, "show-info", is_bool=True)
    add_opt(args, ascii, "ascii", is_bool=True)
    add_opt(args, delete_faces, "delete-faces", is_bool=True)
    add_opt(args, delete_vnormals, "delete-vnormals", is_bool=True)
    add_opt(args, delete_vcolors, "delete-vcolors", is_bool=True)
    add_opt(args, delete_values, "delete-values", is_bool=True)
    add_opt(args, delete_confidences, "delete-confidences", is_bool=True)

    run_app("convert-mesh", args)  # mesh_tools/convert.cpp


def _generate_texture(
    in_mesh, out_prefix,
    octaves=None, persistence=None, noise_scale=None, factor=None, grid_scale=None, resolution=None,
):
    """ Generate noise textures for the mesh with simplex noise. """
    args = [in_mesh, out_prefix]

    add_opt(args, octaves, "octaves")          # 6
    add_opt(args, persistence, "persistence")  # 0.65
    add_opt(args, noise_scale, "noise-scale")  # 1.0
    add_opt(args, factor, "factor")            # 1.0
    add_opt(args, grid_scale, "grid-scale")    # 1.0
    add_opt(args, resolution, "resolution")    # 100.0

    run_app("generate_texture", args)  # generate_texture/generate_texture.cpp


def _generate_proxy_cloud(
    in_mesh, out_cloud,
    samples=None,
):
    """ Create point cloud by uniformly sampling the mesh surface. """
    args = [in_mesh, out_cloud]

    add_opt(args, samples, "samples")  # 100

    run_app("generate_proxy_cloud", args)  # generate_proxy_cloud/generate_proxy_cloud.cpp


def _generate_proxy_mesh(
    cloud, out_mesh,
    resolution=None, height_map=None, sample_cloud=None, min_distance=None,
):
    """ Generates a proxy geometry of the scene by interperting the point cloud
    as height map and extracting a 2.5D surface from it.
    WARNING: Assumes that the z axis corresponds to height """
    args = [cloud, out_mesh]

    add_opt(args, resolution, "resolution")      # -1.0
    add_opt(args, height_map, "height-map")
    add_opt(args, sample_cloud, "sample-cloud")
    add_opt(args, min_distance, "min-distance")  # 0.0

    run_app("generate_proxy_mesh", args)  # generate_proxy_mesh/generate_proxy_mesh.cpp


def _generate_trajectory(
    proxy_mesh, out_trajectory,
    forward_overlap=None, side_overlap=None, altitude=None, elevation=None, rotation=None,
    angles=None, focal_length=None, aspect_ratio=None, airspace_mesh=None,
):
    """ Generate trajectories. """
    args = [proxy_mesh, out_trajectory]

    add_opt(args, forward_overlap, "forward-overlap")  # 80.0
    add_opt(args, side_overlap, "side-overlap")        # 60.0
    add_opt(args, altitude, "altitude")                # 60.0
    add_opt(args, elevation, "elevation")              # 0.0
    add_opt(args, rotation, "rotation")                # 0

    add_opt(args, angles, "angles")                    # 0
    add_opt(args, focal_length, "focal-length")        # 0.86
    add_opt(args, aspect_ratio, "aspect-ratio")        # 0.66
    add_opt(args, airspace_mesh, "airspace-mesh")

    run_app("generate-trajectory", args)  # trajectory_tools/generate.cpp


def _optimize_trajectory(
    in_trajectory, proxy_mesh, proxy_cloud, airspace, out_trajectory,
    seed=None, min_distance=None, max_distance=None, focal_length=None, independence=None,
    max_iters=None,
):
    """ Optimize position and orientation of trajectory views. """
    args = [in_trajectory, proxy_mesh, proxy_cloud, airspace, out_trajectory]

    add_opt(args, seed, "seed")                  # 0
    add_opt(args, min_distance, "min-distance")  # 2.5
    add_opt(args, max_distance, "max-distance")  # 50.0
    add_opt(args, focal_length, "focal-length")  # 0.86
    add_opt(args, independence, "independence")  # 1.0
    add_opt(args, max_iters, "max-iters")        # 100

    run_app("optimize_trajectory", args)  # optimize_trajectory/optimize_trajectory.cu


def _shorten_trajectory(
    in_trajectory, out_trajectory,
):
    """ Searches for a short path through the input trajectories view positions
    by solving the corresponding TSP. """
    args = [in_trajectory, out_trajectory]

    run_app("shorten-trajectory", args)  # trajectory_tools/shorten.cpp


def _evaluate_trajectory(
    trajectory, proxy_mesh, proxy_cloud,
    reconstructability=None, observations=None, max_distance=None,
):
    """ Evaluate trajectory. """
    args = [trajectory, proxy_mesh, proxy_cloud]

    add_opt(args, reconstructability, "reconstructability")
    add_opt(args, observations, "observations")
    add_opt(args, max_distance, "max-distance")  # 80.0

    run_app("evaluate_trajectory", args)  # evaluate_trajectory/evaluate_trajectory.cu


def _interpolate_trajectory(
    in_trajectory, out_csv,
    transform=None, resolution=None, invert=None, trajectory=None, sequence=None,
):
    """ Interpolate and write out trajectory csv files. """
    args = [in_trajectory, out_csv]

    add_opt(args, transform, "transform")
    add_opt(args, resolution, "resolution")  # 1.0
    add_opt(args, invert, "invert", is_bool=True)
    add_opt(args, trajectory, "trajectory")
    add_opt(args, sequence, "sequence")

    run_app("interpolate-trajectory", args)  # trajectory_tools/interpolate.cpp


def _generate_guidance_volume(
    proxy_mesh, proxy_cloud, airspace_mesh, out_volume,  # XXX this generates super large files (GB)
    resolution=None,
    max_distance=None, min_altitude=None, max_altitude=None,
):
    """ Generate guidance volume. """
    args = [proxy_mesh, proxy_cloud, airspace_mesh, out_volume]

    add_opt(args, resolution, "resolution")      # 1.0
    add_opt(args, max_distance, "max-distance")  # 80.0
    add_opt(args, min_altitude, "min-altitude")  # 0.0
    add_opt(args, max_altitude, "max-altitude")  # 100.0

    run_app("generate_guidance_volume", args)  # generate_guidance_volume/generate_guidance_volume.cu


def _generate_initial_trajectory(
    guidance_volume, out_trajectory,
    seed=None, focal_length=None,
    num_views=None,
):
    """ Samples the guidance volume to create an initial trajectory. """
    args = [guidance_volume, out_trajectory]

    add_opt(args, seed, "seed")                  # 0
    add_opt(args, focal_length, "focal-length")  # 0.86
    add_opt(args, num_views, "num-views")        # 500

    run_app("generate_initial_trajectory", args)  # generate_initial_trajectory/generate_initial_trajectory.cpp


def _plan_trajectory(
    proxy_mesh, proxy_cloud, out_trajectory,
    min_distance=None, max_distance=None, min_altitude=None, max_altitude=None, max_velocity=None, focal_length=None,
    num_views=None,
):
    """ Plans a trajectory maximizing reconstructability. """
    args = [proxy_mesh, proxy_cloud, out_trajectory]

    add_opt(args, min_distance, "min-distance")  # 0.0
    add_opt(args, max_distance, "max-distance")  # 80.0
    add_opt(args, min_altitude, "min-altitude")  # 0.0
    add_opt(args, max_altitude, "max-altitude")  # 100.0
    add_opt(args, max_velocity, "max-velocity")  # 5.0
    add_opt(args, focal_length, "focal-length")  # 0.86
    add_opt(args, num_views, "num-views")        # 500

    run_app("plan_trajectory", args)  # plan_trajectory/plan_trajectory.cu


def _visualizer(
    mesh=None, volume=None, trajectory=None,
):
    """ Visualizer for meshes, volumes and trajectories. """
    args = []

    add_opt(args, mesh, "mesh")
    add_opt(args, volume, "volume")
    add_opt(args, trajectory, "trajectory")

    run_app("visualizer", args)  # visualizer/visualizer.cpp


class Uavmvs:
    """ Apps used in makeTraj.sh. """

    convert_mesh = _convert_mesh
    generate_texture = _generate_texture
    generate_proxy_cloud = _generate_proxy_cloud
    generate_proxy_mesh = _generate_proxy_mesh
    generate_trajectory = _generate_trajectory
    optimize_trajectory = _optimize_trajectory
    shorten_trajectory = _shorten_trajectory
    evaluate_trajectory = _evaluate_trajectory
    interpolate_trajectory = _interpolate_trajectory

    r"""
        $ convert-mesh $data $meshes/mesh.ply

        $ generate_texture --resolution=45 -n 50 -f 1.6 $meshes/mesh.ply $models/model

        $ generate_proxy_cloud gesox-mesh.ply gesox-cloud.ply --samples=25

        $ generate_proxy_mesh $root/$scenes/recon/gesox-cloud.ply $root/$scenes/recon/gesox-mesh.ply
        $ generate_proxy_mesh $root/$scenes/recon/gesox-cloud.ply $root/$scenes/recon/airspace.ply \
                              --min-distance=5.0

        $ generate-trajectory $root/$scenes/recon/gesox-mesh.ply oblique.traj \
                              --altitude=$alt --elevation=$elev --angles=0 \
                              --forward-overlap=$f --side-overlap=$s

        $ optimize_trajectory oblique.traj $root/$scenes/recon/gesox-mesh.ply \
                              $root/$scenes/recon/gesox-cloud.ply $root/$scenes/recon/airspace.ply \
                              oblique-opti.traj -m 10000 --max-distance=$maxD --min-distance=$minD

        $ shorten-trajectory oblique-opti.traj oblique-opti.traj

        $ evaluate_trajectory oblique-opti.traj $root/$meshes/mesh.ply \
                              $root/$scenes/recon/gesox-cloud.ply --max-distance=$maxDR -r recon.ply

        $ interpolate-trajectory oblique-opti.traj oblique_spline.csv
    """


PWD = os.path.dirname(os.path.abspath(__file__))


def path_to(*relative_path):
    # return os.path.join(*relative_path)
    return os.path.abspath(os.path.join(*relative_path))


def main(experiment_dir, verbose, **apps):
    assert not os.path.isdir(experiment_dir), experiment_dir

    # # # # # # # # # # # # # # # #

    def make_dir(root_path, dir_name):
        dir_path = path_to(root_path, dir_name)
        print("\n$ mkdir", dir_path)  # XXX os.makedirs(dir_path)
        return dir_path

    meshes_dir = make_dir(experiment_dir, "meshes")
    models_dir = make_dir(experiment_dir, "models")
    scenes_dir = make_dir(experiment_dir, "scenes")

    scenes_nadir_dir = make_dir(scenes_dir, "nadir")
    scenes_recon_dir = make_dir(scenes_dir, "recon")
    scenes_overlap_dir = make_dir(scenes_dir, f"overlap-{F}-{S}")
    scenes_overlap_plan_dir = make_dir(scenes_overlap_dir, "plan")

    # # # # # # # # # # # # # # # #

    proxy_mesh_path = path_to("Benchmark", "Housing", "ProxyMesh", "gesox-mesh.ply")
    airspace_path = path_to("Benchmark", "Housing", "ProxyMesh", "airspace.ply")

    proxy_cloud_path = path_to("gesox-cloud.ply")  # FIXME generate
    generated_trajectory_path = path_to("oblique.traj")
    optimized_trajectory_path = path_to("oblique-opti.traj")
    spline_trajectory_csv_path = path_to("oblique_spline.csv")

    # # # # # # # # # # # # # # # #

    # Make Textured version of ground-truth mesh.
    # _convert_mesh(
    #     in_mesh         = path_to("data.ply"),  # FIXME
    #     out_mesh        = path_to("mesh.ply"),  # FIXME
    # )

    # _generate_texture(
    #     in_mesh         = path_to("mesh.ply"),  # FIXME,
    #     out_prefix      = "model",  # FIXME,
    #     resolution      = 45,
    #     factor          = 1.6,
    # )

    # Note: if you only have a mesh, or it has many holes, first convert it
    # to a point cloud using this command:
    # _generate_proxy_cloud(
    #     in_mesh         = proxy_mesh_path,
    #     out_cloud       = proxy_cloud_path,
    #     samples         = 25,
    # )

    # This takes as input the point cloud representing the proxy model.
    _generate_proxy_mesh(
        cloud           = proxy_cloud_path,
        out_mesh        = proxy_mesh_path,
    )

    _generate_proxy_mesh(
        cloud           = proxy_cloud_path,
        out_mesh        = airspace_path,
        min_distance    = 5,
    )

    # # # # # # # # # # # # # # # #

    # This calculates the estimated density of cameras desired for path planning.
    _generate_trajectory(
        proxy_mesh      = proxy_mesh_path,
        out_trajectory  = generated_trajectory_path,
        altitude        = ALT,
        elevation       = ELEV,
        angles          = [0],
        forward_overlap = F,
        side_overlap    = S,
    )

    _optimize_trajectory(
        in_trajectory   = generated_trajectory_path,
        proxy_mesh      = proxy_mesh_path,
        proxy_cloud     = proxy_cloud_path,
        airspace        = airspace_path,
        out_trajectory  = optimized_trajectory_path,
        min_distance    = MIN_D,
        max_distance    = MAX_D,
        max_iters       = 10000,
    )

    _shorten_trajectory(
        in_trajectory   = optimized_trajectory_path,
        out_trajectory  = optimized_trajectory_path,
    )

    # This will visualize how good the reconstruction is estimated to be.
    # _evaluate_trajectory(
    #     trajectory      = optimized_trajectory_path,
    #     proxy_mesh      = path_to("mesh.ply"),  # FIXME
    #     proxy_cloud     = proxy_cloud_path,
    #     max_distance    = MAX_DR,
    #     reconstructability = path_to("recon.ply"),  # FIXME
    # )

    # This converts trajectory views to spline trajectory in csv format.
    _interpolate_trajectory(
        in_trajectory   = optimized_trajectory_path,
        out_csv         = path_to("oblique_spline.csv"),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Helper script for running uavmvs apps (based on makeTraj.sh)."
    )

    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--experiment_dir", type=str, default="experiment")

    args = parser.parse_args()

    main(**vars(args))

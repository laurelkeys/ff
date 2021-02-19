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


class Uavmvs:
    """Apps used in makeTraj.sh"""

    convert_mesh            = "convert-mesh"            # [OPTS] IN_MESH          OUT_MESH
    generate_texture        = "generate_texture"        # [OPTS] IN_MESH          OUT_PREFIX
    generate_proxy_cloud    = "generate_proxy_cloud"    # [OPTS] IN_MESH          OUT_CLOUD
    generate_proxy_mesh     = "generate_proxy_mesh"     # [OPTS] CLOUD            OUT_MESH
    generate_trajectory     = "generate-trajectory"     # [OPTS] PROXY_MESH       OUT_TRAJECTORY
    optimize_trajectory     = "optimize_trajectory"     # [OPTS] IN_TRAJECTORY    PROXY_MESH     PROXY_CLOUD AIRSPACE OUT_TRAJECTORY
    shorten_trajectory      = "shorten-trajectory"      # [OPTS] IN_TRAJECTORY    OUT_TRAJECTORY
    evaluate_trajectory     = "evaluate_trajectory"     # [OPTS] TRAJECTORY/SCENE PROXY_MESH     PROXY_CLOUD
    interpolate_trajectory  = "interpolate-trajectory"  # [OPTS] IN_TRAJECTORY    OUT_CSV

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


def _run_app(app, *args):
    run_str = f"{app} " + " ".join(map(str, *args))
    print("\n$", run_str)  # XXX os.system(run_str)


def convert_mesh(
    in_mesh, out_mesh,
    transform=None, invert=None, clean=None,
    show_info=None, ascii=None, delete_faces=None, delete_vnormals=None, delete_vcolors=None, delete_values=None, delete_confidences=None,
):
    """ Conversion app for meshes a la image magicks convert. """
    args = [in_mesh, out_mesh]

    def add_opt(arg, name):
        if arg is True:
            args.append(f"--{name}")

    if transform is not None:
        args.append(f"--transform={transform}")
    add_opt(invert, "invert")
    add_opt(clean, "clean")
    add_opt(show_info, "show-info")
    add_opt(ascii, "ascii")
    add_opt(delete_faces, "delete-faces")
    add_opt(delete_vnormals, "delete-vnormals")
    add_opt(delete_vcolors, "delete-vcolors")
    add_opt(delete_values, "delete-values")
    add_opt(delete_confidences, "delete-confidences")

    _run_app(Uavmvs.convert_mesh, args)


def generate_texture(
    in_mesh, out_prefix,
    octaves=None, persistence=None, noise_scale=None, factor=None, grid_scale=None, resolution=None,
):
    """ Generate noise textures for the mesh with simplex noise. """
    args = [in_mesh, out_prefix]

    def add_opt(arg, name):
        if arg is not None:
            args.append(f"--{name}={arg}")

    add_opt(octaves, "octaves")
    add_opt(persistence, "persistence")
    add_opt(noise_scale, "noise-scale")
    add_opt(factor, "factor")
    add_opt(grid_scale, "grid-scale")
    add_opt(resolution, "resolution")

    _run_app(Uavmvs.generate_texture, args)


def generate_proxy_cloud(
    in_mesh, out_cloud,
    samples=None,
):
    """ Create point cloud by uniformly sampling the mesh surface. """
    args = [in_mesh, out_cloud]

    if samples is not None:
        args.append(f"--samples={samples}")

    _run_app(Uavmvs.generate_proxy_cloud, args)


def generate_proxy_mesh(
    cloud, out_mesh,
    resolution=None, height_map=None, sample_cloud=None, min_distance=None,
):
    """ Generates a proxy geometry of the scene by interperting the point cloud
    as height map and extracting a 2.5D surface from it.
    WARNING: Assumes that the z axis corresponds to height """
    args = [cloud, out_mesh]

    def add_opt(arg, name):
        if arg is not None:
            args.append(f"--{name}={arg}")

    add_opt(resolution, "resolution")
    add_opt(height_map, "height-map")
    add_opt(sample_cloud, "sample-cloud")
    add_opt(min_distance, "min-distance")

    _run_app(Uavmvs.generate_proxy_mesh, args)


def generate_trajectory(
    proxy_mesh, out_trajectory,
    forward_overlap=None, side_overlap=None, altitude=None, elevation=None, rotation=None,
    angles=None, focal_length=None, aspect_ratio=None, airspace_mesh=None,
):
    """ Generate trajectories. """
    args = [proxy_mesh, out_trajectory]

    def add_opt(arg, name):
        if arg is not None:
            if isinstance(arg, list):
                arg = ",".join(map(str, arg))
            args.append(f"--{name}={arg}")

    add_opt(forward_overlap, "forward-overlap")
    add_opt(side_overlap, "side-overlap")
    add_opt(altitude, "altitude")
    add_opt(elevation, "elevation")
    add_opt(rotation, "rotation")

    add_opt(angles, "angles")
    add_opt(focal_length, "focal-length")
    add_opt(aspect_ratio, "aspect-ratio")
    add_opt(airspace_mesh, "airspace-mesh")

    _run_app(Uavmvs.generate_trajectory, args)


def optimize_trajectory(
    in_trajectory, proxy_mesh, proxy_cloud, airspace, out_trajectory,
    seed=None, min_distance=None, max_distance=None, focal_length=None, independence=None,
    max_iters=None,
):
    """ Optimize position and orientation of trajectory views. """
    args = [in_trajectory, proxy_mesh, proxy_cloud, airspace, out_trajectory]

    def add_opt(arg, name):
        if arg is not None:
            args.append(f"--{name}={arg}")

    add_opt(seed, "seed")
    add_opt(min_distance, "min-distance")
    add_opt(max_distance, "max-distance")
    add_opt(focal_length, "focal-length")
    add_opt(independence, "independence")
    add_opt(max_iters, "max-iters")

    _run_app(Uavmvs.optimize_trajectory, args)


def shorten_trajectory(
    in_trajectory, out_trajectory,
):
    """ Searches for a short path through the input trajectories view positions
    by solving the corresponding TSP. """
    args = [in_trajectory, out_trajectory]

    _run_app(Uavmvs.shorten_trajectory, args)


def evaluate_trajectory(
    trajectory, proxy_mesh, proxy_cloud,
    reconstructability=None, observations=None, max_distance=None,
):
    """ Evaluate trajectory. """
    args = [trajectory, proxy_mesh, proxy_cloud]

    def add_opt(arg, name):
        if arg is not None:
            args.append(f"--{name}={arg}")

    add_opt(reconstructability, "reconstructability")
    add_opt(observations, "observations")
    add_opt(max_distance, "max-distance")

    _run_app(Uavmvs.evaluate_trajectory, args)


def interpolate_trajectory(
    in_trajectory, out_csv,
    transform=None, resolution=None, invert=None, trajectory=None, sequence=None,
):
    """ Interpolate and write out trajectory csv files. """
    args = [in_trajectory, out_csv]

    def add_opt(arg, name):
        if arg is not None:
            args.append(f"--{name}={arg}")

    add_opt(transform, "transform")
    add_opt(resolution, "resolution")
    if invert is True:
            args.append(f"--invert")
    add_opt(trajectory, "trajectory")
    add_opt(sequence, "sequence")

    _run_app(Uavmvs.interpolate_trajectory, args)


def _path_to(*relative_path):
    # return os.path.join(*relative_path)
    return os.path.abspath(os.path.join(*relative_path))


def _setup_folder_structure(experiment_dir):
    def make_dir(root_path, dir_name):
        dir_path = _path_to(root_path, dir_name)
        print("\n$ mkdir", dir_path)  # XXX os.makedirs(dir_path)
        return dir_path

    meshes_dir = make_dir(experiment_dir, "meshes")
    models_dir = make_dir(experiment_dir, "models")
    scenes_dir = make_dir(experiment_dir, "scenes")

    scenes_nadir_dir = make_dir(scenes_dir, "nadir")
    scenes_recon_dir = make_dir(scenes_dir, "recon")
    scenes_overlap_dir = make_dir(scenes_dir, f"overlap-{F}-{S}")
    scenes_overlap_plan_dir = make_dir(scenes_overlap_dir, "plan")


PWD = os.path.dirname(os.path.abspath(__file__))


def main(experiment_dir, verbose, **apps):
    assert not os.path.isdir(experiment_dir), experiment_dir

    _setup_folder_structure(experiment_dir)

    proxy_mesh_path = _path_to("Benchmark", "Housing", "ProxyMesh", "gesox-mesh.ply")
    airspace_path = _path_to("Benchmark", "Housing", "ProxyMesh", "airspace.ply")

    proxy_cloud_path = _path_to("gesox-cloud.ply")  # FIXME generate
    generated_trajectory_path = _path_to("oblique.traj")
    optimized_trajectory_path = _path_to("oblique-opti.traj")
    spline_trajectory_csv_path = _path_to("oblique_spline.csv")

    # # # #

    # Make Textured version of ground-truth mesh.
    # convert_mesh(
    #     in_mesh         = _path_to("data.ply"),  # FIXME
    #     out_mesh        = _path_to("mesh.ply"),  # FIXME
    # )

    # generate_texture(
    #     in_mesh         = _path_to("mesh.ply"),  # FIXME,
    #     out_prefix      = "model",  # FIXME,
    #     resolution      = 45,
    #     factor          = 1.6,
    # )

    # Note: if you only have a mesh, or it has many holes, first convert it
    # to a point cloud using this command:
    # generate_proxy_cloud(
    #     in_mesh         = proxy_mesh_path,
    #     out_cloud       = proxy_cloud_path,
    #     samples         = 25,
    # )

    # This takes as input the point cloud representing the proxy model.
    generate_proxy_mesh(
        cloud           = proxy_cloud_path,
        out_mesh        = proxy_mesh_path,
    )

    generate_proxy_mesh(
        cloud           = proxy_cloud_path,
        out_mesh        = airspace_path,
        min_distance    = 5,
    )

    # # # #

    # This calculates the estimated density of cameras desired for path planning.
    generate_trajectory(
        proxy_mesh      = proxy_mesh_path,
        out_trajectory  = generated_trajectory_path,
        altitude        = ALT,
        elevation       = ELEV,
        angles          = [0],
        forward_overlap = F,
        side_overlap    = S,
    )

    optimize_trajectory(
        in_trajectory   = generated_trajectory_path,
        proxy_mesh      = proxy_mesh_path,
        proxy_cloud     = proxy_cloud_path,
        airspace        = airspace_path,
        out_trajectory  = optimized_trajectory_path,
        min_distance    = MIN_D,
        max_distance    = MAX_D,
        max_iters       = 10000,
    )

    shorten_trajectory(
        in_trajectory   = optimized_trajectory_path,
        out_trajectory  = optimized_trajectory_path,
    )

    # This will visualize how good the reconstruction is estimated to be.
    # evaluate_trajectory(
    #     trajectory      = optimized_trajectory_path,
    #     proxy_mesh      = _path_to("mesh.ply"),  # FIXME
    #     proxy_cloud     = proxy_cloud_path,
    #     max_distance    = MAX_DR,
    #     reconstructability = _path_to("recon.ply"),  # FIXME
    # )

    # This converts trajectory views to spline trajectory in csv format.
    interpolate_trajectory(
        in_trajectory       = optimized_trajectory_path,
        out_csv             = _path_to("oblique_spline.csv"),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Helper script for running uavmvs apps (based on makeTraj.sh)."
    )

    parser.add_argument("--verbose", action="store_true")

    parser.add_argument("--experiment_dir", type=str, default="experiment")

    args = parser.parse_args()

    main(**vars(args))

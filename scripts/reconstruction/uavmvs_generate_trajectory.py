import os
import argparse

try:
    from include_in_path import include, FF_PROJECT_ROOT

    include(FF_PROJECT_ROOT, "misc", "tools", "uavmvs_make_traj")
    import uavmvs_make_traj as uavmvs
    from uavmvs_make_traj import Uavmvs
except:
    raise


###############################################################################
###############################################################################


CWD = os.getcwd()
HERE = os.path.dirname(os.path.abspath(__file__))
HOME = os.path.expanduser("~")


def path_to(*relative_path):
    return os.path.abspath(os.path.join(*relative_path))


ASSETS_DIR = path_to(HERE, "assets")


def setup_args(args: argparse.Namespace) -> None:
    args.scene = path_to(args.scene)
    root, ext = os.path.splitext(args.scene)
    assert os.path.isfile(args.scene) and ext == ".ply"

    if args.scene_name is None:
        args.scene_name = os.path.basename(root)

    if args.output_dir is None:
        args.output_dir = path_to(ASSETS_DIR, args.scene_name)

    if args.make_dir:
        os.makedirs(args.output_dir, exist_ok=True)

    if args.verbose:
        print(f"# scene name: '{args.scene_name}'")
        print(f"# input file: '{args.scene}'")
        print(f"# output dir: '{args.output_dir}'")


###############################################################################
###############################################################################


# point samples per unit square
PROXY_CLOUD_SAMPLES = 25

# minimum distance from original samples
PROXY_MESH_MIN_D = 0.95
AIRSPACE_MIN_D = 3.5

# minimum/maximum distance to surface
OPTIMIZE_MIN_D = 2.5
OPTIMIZE_MAX_D = 50.0

# waypoints per meter for the interpolated trajectory
RESOLUTION = 0.5

# XXX camera focal length and sensor aspect ratio
FOCAL_LENGTH = 0.86
ASPECT_RATIO = 0.66

PLANAR_F = 80  # forward overlap in percent
PLANAR_S = 80  # side overlap in percent
PLANAR_ALT = 45  # XXX flying altitude
PLANAR_ELEV = 0  # elevation for overlap planning

SPATIAL_RES = 4  # XXX guidance volume resolution
SPATIAL_MIN_D = 60  # maximum distance to surface
SPATIAL_MIN_ALT = 40  # XXX minimum altitude
SPATIAL_MAX_ALT = 80  # XXX maximum altitude
SPATIAL_VIEWS = 200  # XXX number of views


def print_params(args: argparse.Namespace) -> None:
    print()
    print(f"# PROXY_CLOUD_SAMPLES = {PROXY_CLOUD_SAMPLES}")
    print("#")
    print(f"# PROXY_MESH_MIN_D = {PROXY_MESH_MIN_D}")
    print(f"# AIRSPACE_MIN_D = {AIRSPACE_MIN_D}")
    print("#")
    print(f"# OPTIMIZE_MIN_D = {OPTIMIZE_MIN_D}")
    print(f"# OPTIMIZE_MAX_D = {OPTIMIZE_MAX_D}")
    print("#")
    print(f"# FOCAL_LENGTH = {FOCAL_LENGTH}")
    print(f"# ASPECT_RATIO = {ASPECT_RATIO}")
    if args.planar:
        print("#")
        print(f"# PLANAR_F = {PLANAR_F}")
        print(f"# PLANAR_S = {PLANAR_S}")
        print(f"# PLANAR_ALT = {PLANAR_ALT}")
        print(f"# PLANAR_ELEV = {PLANAR_ELEV}")
    if args.spatial:
        print("#")
        print(f"# SPATIAL_RES = {SPATIAL_RES}")
        print(f"# SPATIAL_MIN_D = {SPATIAL_MIN_D}")
        print(f"# SPATIAL_MIN_ALT = {SPATIAL_MIN_ALT}")
        print(f"# SPATIAL_MAX_ALT = {SPATIAL_MAX_ALT}")
        print(f"# SPATIAL_VIEWS = {SPATIAL_VIEWS}")


###############################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    setup_args(args)
    if args.verbose:
        print_params(args)

    SCENE = args.scene_name
    INPUT_MESH = args.scene
    OUTPUT_DIR = args.output_dir

    PROXY_CLOUD = path_to(OUTPUT_DIR, f"{SCENE}-proxy_cloud.ply")
    PROXY_MESH = path_to(OUTPUT_DIR, f"{SCENE}-proxy_mesh.ply")
    AIRSPACE = path_to(OUTPUT_DIR, f"{SCENE}-airspace.ply")

    def planar(prefix="", ext=".traj"):
        return path_to(OUTPUT_DIR, f"{SCENE}-{prefix}_2D{ext}")

    def spatial(prefix="", ext=".traj"):
        return path_to(OUTPUT_DIR, f"{SCENE}-{prefix}_3D{ext}")

    # fmt:off
    def refine(in_trajectory, fn_out):
        Uavmvs.optimize_trajectory(
            in_trajectory   = in_trajectory,
            proxy_mesh      = PROXY_MESH,
            proxy_cloud     = PROXY_CLOUD,
            airspace        = AIRSPACE,
            out_trajectory  = fn_out("optimized"),
            min_distance    = OPTIMIZE_MIN_D,
            max_distance    = OPTIMIZE_MAX_D,
            max_iters       = 3000,
        )
        if args.verbose:
            # i oindices.size() avg_wrecon volume
            print(f"# > {fn_out('optimized', ext='.out')}")
        Uavmvs.shorten_trajectory(
            in_trajectory   = fn_out("optimized"),
            out_trajectory  = fn_out("shortened"),
        )
        Uavmvs.interpolate_trajectory(
            in_trajectory   = fn_out("shortened"),
            out_csv         = fn_out("interpolated", ext=".csv"),
            resolution      = RESOLUTION,
        )

    if args.verbose:
        print("\n#\n# geometric scene proxy\n#")

    CONVERTED_MESH = path_to(OUTPUT_DIR, f"{SCENE}-converted.ply")
    Uavmvs.convert_mesh(
        in_mesh         = INPUT_MESH,
        out_mesh        = CONVERTED_MESH,
    )
    INPUT_MESH = CONVERTED_MESH

    Uavmvs.generate_proxy_cloud(
        in_mesh         = INPUT_MESH,
        out_cloud       = PROXY_CLOUD,
        samples         = PROXY_CLOUD_SAMPLES,
    )
    Uavmvs.generate_proxy_mesh(
        cloud           = PROXY_CLOUD,
        out_mesh        = PROXY_MESH,
        min_distance    = PROXY_MESH_MIN_D,
    )
    Uavmvs.generate_proxy_mesh(
        cloud           = PROXY_CLOUD,
        out_mesh        = AIRSPACE,
        min_distance    = AIRSPACE_MIN_D,
    )

    if args.planar:
        if args.verbose:
            print("\n#\n# 1st (planar, 2D) trajectory\n#")

        Uavmvs.generate_trajectory(
            proxy_mesh      = PROXY_MESH,
            out_trajectory  = planar("initial"),
            altitude        = PLANAR_ALT,  # XXX
            elevation       = PLANAR_ELEV,
            angles          = [0],
            forward_overlap = PLANAR_F,
            side_overlap    = PLANAR_S,
            focal_length    = FOCAL_LENGTH,
            aspect_ratio    = ASPECT_RATIO,
            # airspace_mesh   = AIRSPACE,  # XXX
        )

        refine(planar("initial"), planar)

    if args.spatial:
        if args.verbose:
            print("\n#\n# 2nd (spatial, 3D) trajectory\n#")

        uavmvs._generate_guidance_volume(
            proxy_mesh      = PROXY_MESH,
            proxy_cloud     = PROXY_CLOUD,
            airspace_mesh   = AIRSPACE,
            out_volume      = spatial("guidance", ext=".vol"),  # XXX large file ~ GBs
            resolution      = SPATIAL_RES,  # XXX
            max_distance    = SPATIAL_MIN_D,
            min_altitude    = SPATIAL_MIN_ALT,  # XXX
            max_altitude    = SPATIAL_MAX_ALT,  # XXX
        )
        uavmvs._generate_initial_trajectory(
            guidance_volume = spatial("guidance", ext=".vol"),
            out_trajectory  = spatial("initial"),
            num_views       = SPATIAL_VIEWS,  # XXX
            focal_length    = FOCAL_LENGTH,
        )

        refine(spatial("initial"), spatial)
    # fmt:on


###############################################################################
###############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Call uavmvs apps to generate a flight trajectory."
    )

    parser.add_argument("scene", type=str, help="Path to the scene's .PLY mesh file")

    parser.add_argument("--scene_name", type=str, help="Name used for output files")
    parser.add_argument("--output_dir", type=str, help="Path to the output directory")
    parser.add_argument("--make_dir", action="store_true", help="Create the output directory")

    parser.add_argument("--planar", action="store_true", help="Generate planar trajectory")
    parser.add_argument("--spatial", action="store_true", help="Generate spatial trajectory")

    parser.add_argument("--verbose", "-v", action="store_true", help="Increase verbosity")

    main(args=parser.parse_args())


###############################################################################
###############################################################################

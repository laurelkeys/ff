import os
import argparse

from enum import Enum
from typing import Optional, NamedTuple

import numpy as np
import open3d as o3d

try:
    from include_in_path import include
except:
    pass
finally:
    include("..", "..", "misc", "uavmvs_make_traj")
    import uavmvs_make_traj as uavmvs
    from uavmvs_make_traj import Uavmvs, ALT, ELEV, F, S, MAX_D, MIN_D, MAX_DR


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
        args.output_dir = ASSETS_DIR
    args.output_dir = path_to(args.output_dir, args.scene_name)

    if args.make_dir:
        os.makedirs(args.output_dir, exist_ok=True)

    if args.verbose:
        print(f"# scene name: '{args.scene_name}'")
        print(f"# input file: '{args.scene}'")
        print(f"# output dir: '{args.output_dir}'")


###############################################################################
###############################################################################


def main(args: argparse.Namespace) -> None:
    setup_args(args)

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
            min_distance    = MIN_D,
            max_distance    = MAX_D,
            max_iters       = 10000,
        )
        Uavmvs.shorten_trajectory(
            in_trajectory   = fn_out("optimized"),
            out_trajectory  = fn_out("shortened"),
        )
        Uavmvs.interpolate_trajectory(
            in_trajectory   = fn_out("shortened"),
            out_csv         = fn_out("interpolated", ext=".csv"),
        )

    if args.verbose:
        print("\n#\n# geometric scene proxy\n#")

    # Uavmvs.convert_mesh(
    #     in_mesh         = INPUT_MESH,
    #     out_mesh        = path_to(OUTPUT_DIR, "{SCENE}-converted.ply"),
    # )
    # INPUT_MESH = path_to(OUTPUT_DIR, "convert_mesh.ply")

    Uavmvs.generate_proxy_cloud(
        in_mesh         = INPUT_MESH,
        out_cloud       = PROXY_CLOUD,
        samples         = 25,
    )
    Uavmvs.generate_proxy_mesh(
        cloud           = PROXY_CLOUD,
        out_mesh        = PROXY_MESH,
        min_distance    = 0.91,
    )
    Uavmvs.generate_proxy_mesh(
        cloud           = PROXY_CLOUD,
        out_mesh        = AIRSPACE,
        min_distance    = 3.6,
    )

    if args.planar:
        if args.verbose:
            print("\n#\n# 1st (planar, 2D) trajectory\n#")

        Uavmvs.generate_trajectory(
            proxy_mesh      = PROXY_MESH,
            out_trajectory  = planar("initial"),
            altitude        = ALT,
            elevation       = ELEV,
            angles          = [0],
            forward_overlap = F,
            side_overlap    = S,
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
            resolution      = 1,
            max_distance    = 35,
            min_altitude    = 10,
            max_altitude    = 50,
        )
        uavmvs._generate_initial_trajectory(
            guidance_volume = spatial("guidance", ext=".vol"),
            out_trajectory  = spatial("initial"),
            num_views       = 200,
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

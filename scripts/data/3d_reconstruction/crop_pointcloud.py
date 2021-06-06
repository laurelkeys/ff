from os.path import join, isfile, abspath, dirname

import numpy as np
import open3d as o3d

# HACK manually edited values...
HERE = dirname(abspath(__file__))

TARGET_PLY_PATH = join(HERE, "angel", "target.ply")
SOURCE_PLY_PATHS = [
    join(HERE, "angel", "v1_initial", "cv", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
    join(HERE, "angel", "v1_initial", "drone", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
    join(HERE, "angel", "v1_initial", "drone2", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
    join(HERE, "angel", "v1_shortened", "cv", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
    join(HERE, "angel", "v1_shortened", "drone", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
    join(HERE, "angel", "v1_shortened", "drone2", "meshroom", "cache", "StructureFromMotion", "sfm.ply"),
]

# NOTE these matrices align the Meshroom's coordinate system to AirSim's
# which is not the same as the target point cloud's (extracted from UE4)
ALIGN_TXT_PATHS = [
    # join(HERE, "angel", "v1_initial", "cv", "align.txt"),
    # join(HERE, "angel", "v1_initial", "drone", "align.txt"),
    # join(HERE, "angel", "v1_initial", "drone2", "align.txt"),
    # join(HERE, "angel", "v1_shortened", "cv", "align.txt"),
    # join(HERE, "angel", "v1_shortened", "drone", "align.txt"),
    # join(HERE, "angel", "v1_shortened", "drone2", "align.txt"),
]


def main():
    assert isfile(TARGET_PLY_PATH), TARGET_PLY_PATH
    target_pcd = o3d.io.read_point_cloud(TARGET_PLY_PATH)

    source_pcds = []
    for source_ply_path in SOURCE_PLY_PATHS:
        assert isfile(source_ply_path), source_ply_path
        source_pcds.append(o3d.io.read_point_cloud(source_ply_path))

    if ALIGN_TXT_PATHS:
        assert len(ALIGN_TXT_PATHS) == len(SOURCE_PLY_PATHS)
        for i, align_txt_path in enumerate(ALIGN_TXT_PATHS):
            source_pcds[i].transform(np.loadtxt(align_txt_path))

    geometries = source_pcds  # + [target_pcd]
    o3d.visualization.draw_geometries(geometries)


if __name__ == "__main__":
    main()

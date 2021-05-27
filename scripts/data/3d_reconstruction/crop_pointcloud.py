from os.path import join, isfile, abspath, dirname

import open3d as o3d

# HACK manually edited values...
HERE = dirname(abspath(__file__))

TARGET_PLY_PATH = join(HERE, "angel", "target.ply")
SOURCE_PLY_PATHS = [
    join(HERE, "angel", "v1_initial", "cv", "meshroom", "source.ply"),
    join(HERE, "angel", "v1_initial", "drone", "meshroom", "source.ply"),
    join(HERE, "angel", "v1_shortened", "cv", "meshroom", "source.ply"),
    join(HERE, "angel", "v1_shortened", "drone", "meshroom", "source.ply"),
]


def main():
    assert isfile(TARGET_PLY_PATH), TARGET_PLY_PATH
    target_pcd = o3d.io.read_point_cloud(TARGET_PLY_PATH)

    source_pcds = []
    for source_ply_path in SOURCE_PLY_PATHS:
        assert isfile(source_ply_path), source_ply_path
        source_pcds.append(o3d.io.read_point_cloud(source_ply_path))

    geometries = source_pcds  # + [target_pcd]
    o3d.visualization.draw_geometries(geometries)


if __name__ == "__main__":
    main()

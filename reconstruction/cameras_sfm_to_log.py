import os
import json
import argparse

import numpy as np
import open3d as o3d


# ref.: https://github.com/alicevision/meshroom/issues/655
#       https://github.com/alicevision/meshroom/issues/300
#       https://github.com/alicevision/meshroom/blob/develop/meshroom/ui/reconstruction.py#L305



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert camera.sfm data into the Tanks and Temples .log file format"
    )

    parser.add_argument("filepath", nargs="?", default="camera.json")

    args = parser.parse_args()

    # NOTE .sfm is actually a JSON
    _, ext = os.path.splitext(args.filepath)
    assert ext.lower() in [".sfm", ".json"]
    assert os.path.isfile(args.filepath)

    trajectory_matrices = []

    with open(args.filepath, 'r') as f:
        cameras = json.load(f)
        for camera_pose in cameras['poses']:
            pose_id = camera_pose['poseId']
            pose_transform = camera_pose['pose']['transform']

            # rotation = np.array(
            #     [float(_) for _ in pose_transform['rotation']]
            # ).reshape((3, 3), order="F")

            # 3x3 (column-major) rotation matrix
            rotation = np.array(
                [float(_) for _ in pose_transform['rotation']]
            ).reshape((3, 3))
            rotation[:, 1:] *= -1

            # camera center in world coordinates
            center = np.array([float(_) for _ in pose_transform['center']])

            # homogeneous transformation matrix
            mat = np.identity(4)
            mat[:3, :3] = rotation
            mat[:3,  3] = center

            trajectory_matrices.append(mat)
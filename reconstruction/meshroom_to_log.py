import os
import glob
import json
import argparse
import collections

import numpy as np

# ref.:
#  [1] https://www.tanksandtemples.org/tutorial/
#  [2] https://colmap.github.io/format.html#images-txt
#  [3] https://github.com/colmap/colmap/blob/dev/src/estimators/pose.h#L125
#  [4] https://github.com/alicevision/meshroom/wiki/Using-known-camera-positions
#  [5] https://github.com/colmap/colmap/blob/dev/scripts/python/read_write_model.py


# FIXME rename, so it's not confused with trajectory_io
class CameraPose:
    def __init__(self, pose_id, image_path, log_matrix):
        self.id = pose_id
        self.image_path = image_path
        self.log_matrix = log_matrix


def write_SfM_log(T, i_map, filename):
    with open(filename, 'w') as f:
        for i, traj in enumerate(T):
            metadata = i_map[i]
            pose = traj.tolist()
            f.write(' '.join(map(str, metadata)) + '\n')
            f.write('\n'.join(' '.join(
                map('{0:.12f}'.format, pose[i])
            ) for i in range(4)))
            f.write('\n')


def convert_Meshroom_to_log(filename, logfile_out, input_images, formatp):
    input_images_list = glob.glob(f"{input_images}/*.{formatp}")
    input_images_list.sort()
    n_of_images = len(input_images_list)

    T, i_map, TF, i_mapF = [], [], [], []

    views = {}
    camera_poses = []
    with open(filename, 'r') as sfm_file:
        sfm_data = json.load(sfm_file)

        for view in sfm_data['views']:
            views[view['poseId']] = view['path'] # NOTE equal to the 'viewId'

        for camera_pose in sfm_data['poses']:
            pose_id = camera_pose['poseId']
            pose_transform = camera_pose['pose']['transform']

            # 3x3 (column-major) rotation matrix
            rotation = np.array(
                [float(_) for _ in pose_transform['rotation']]
            ).reshape((3, 3))
            rotation[:, 1:] *= -1 # ref.: [2]

            # camera center in world coordinates
            center = np.array([float(_) for _ in pose_transform['center']])

            # homogeneous transformation matrix
            mat = np.identity(4)
            mat[:3, :3] = rotation
            mat[:3,  3] = center

            camera_poses.append(CameraPose(pose_id, views[pose_id], mat))

    for pose in camera_poses:
        A = np.matrix(pose.log_matrix)
        T.append(A.I)
        image_name =  os.path.basename(pose.image_path)
        matching = [i for i, s in enumerate(input_images_list) if image_name in s]
        i_map.append([pose.id, matching[0], 0])

    for k in range(n_of_images):
        try:
            # find the k-th view id
            view_id = [i for i, item in enumerate(i_map) if k == item[1]][0]
            i_mapF.append(np.array([k, k, 0], dtype='int'))
            TF.append(T[view_id])
        except IndexError:
            # assign the identity matrix to the k-th view id
            # as the log file needs an entry for every image
            i_mapF.append(np.array([k, -1, 0], dtype='int'))
            TF.append(np.identity(4))

    write_SfM_log(TF, i_mapF, logfile_out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert Meshroom .sfm data into the Tanks and Temples .log file format"
    )

    parser.add_argument("in_sfm_fname", help="Input .sfm filename")
    parser.add_argument("out_log_fname", help="Output .log filename")
    parser.add_argument("images_folder", help="Input images folder path")
    parser.add_argument("--formatp", default="jpg", help="Images format")

    args = parser.parse_args()

    # NOTE .sfm is actually a JSON
    _, ext = os.path.splitext(args.in_sfm_fname)
    assert ext.lower() in [".sfm", ".json"]
    assert os.path.isfile(args.in_sfm_fname)
    assert os.path.isdir(args.images_folder)

    convert_Meshroom_to_log(
        args.in_sfm_fname,
        args.out_log_fname,
        args.images_folder, args.formatp
    )

    # e.g.: python meshroom_to_log.py models\Monstree6\Meshroom\publish\cameras.json models\Monstree6\pointcloud\Monstree6_Meshroom_SfM.log models\Monstree6\images\

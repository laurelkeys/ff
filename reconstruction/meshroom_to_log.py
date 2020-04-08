import os
import glob
import json
import argparse
import collections

import numpy as np
import open3d as o3d

# ref.:
#  [1] https://colmap.github.io/format.html#images-txt
#  [2] https://github.com/colmap/colmap/blob/dev/src/estimators/pose.h#L125
#  [3] https://github.com/colmap/colmap/blob/dev/scripts/python/read_write_model.py

IDENTITY_4x4 = np.identity(4)

# ref.: [3] (NOTE only 'id', 'qvec', 'tvec' and 'name' are needed)
COLMAPImage = collections.namedtuple("COLMAPImage", [
    "id", "qvec", "tvec", "camera_id", "name", "xys", "point3D_ids"
])

class CameraPose:
    def __init__(self, camera_pose):
        self.id = camera_pose['poseId'] # NOTE equal to the 'viewId'
        self.rotation = camera_pose['pose']['transform']['rotation']
        self.center = camera_pose['pose']['transform']['center']

    def np_rotation(self):
        # 3x3 (column-major) rotation matrix
        rotation = np.array(
            [float(_) for _ in self.rotation]
        ).reshape((3, 3))
        rotation[:, 1:] *= -1 # ref.: [1]
        return rotation

    def np_center(self):
        # camera center in world coordinates
        center = np.array(
            [float(_) for _ in self.center]
        )
        return center

    def np_log_matrix(self):
        # homogeneous transformation matrix
        mat = np.identity(4)
        mat[:3, :3] = self.np_rotation()
        mat[:3,  3] = self.np_center()
        return mat



def quat2rotmat(qvec):
    return np.array([
        [1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
         2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
         2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]],

        [2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
         1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
         2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]],

        [2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
         2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
         1 - 2 * qvec[1]**2 - 2 * qvec[2]**2]
    ])


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

    views = []
    camera_poses = []
    observations = []
    with open(filename, 'r') as sfm_file:
        sfm_data = json.load(sfm_file)

        for view in sfm_data['views']:
            views.append((
                views['viewId'],
                views['path'],
            ))
            # assert views['viewId'] == views['poseId']

        for camera_pose in sfm_data['poses']:
            camera_poses.append(CameraPose(camera_pose))

        for landmark in sfm_data['structure']:
            ids = []
            for observation_id in landmark['observations']:
                ids.append(observation_id['observationId']) # NOTE equal to 'poseId'

            observations.append((
                landmark['landmarkId'],
                landmark['X'],
                ids,
            ))

    images = [] # FIXME make an Image list like COLMAP's

    for im in images:
        # im = Image(im)
        qvec = im[1] # im.qvec
        rotmat = quat2rotmat(qvec)
        translation = im[2] # im.tvec
        w = np.zeros((4, 4))
        w[ 3,  3] = 1
        w[:3, :3] = rotmat
        w[:3,  3] = translation
        A = np.matrix(w)
        T.append(A.I)
        image_name = im[4] # im.name
        matching = [i for i, s in enumerate(input_images_list) if image_name in s]
        ii = im[0] # im.id
        i_map.append([ii, matching[0], 0])

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
            TF.append(IDENTITY_4x4)

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
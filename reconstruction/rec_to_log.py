import os
import glob
import json
import argparse
import collections

import numpy as np
import pandas as pd
import quaternion


# FIXME rename, so it's not confused with trajectory_io
# TODO extract common functionalities to abstract this and meshroom_to_log.py
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


def convert_rec_to_log(in_rec_fpath, out_log_fpath, image_folder):
    input_images_list = glob.glob(f"{image_folder}/*.png") # NOTE AirSim saves images as .png
    input_images_list.sort()
    n_of_images = len(input_images_list)

    T, i_map, TF, i_mapF = [], [], [], []

    camera_poses = []
    for entry in pd.read_csv(in_rec_fpath, delim_whitespace=True).itertuples():
        timestamp = entry.TimeStamp
        pos = np.array([entry.POS_X, entry.POS_Y, entry.POS_Z])  # (x, y, z)
        orien = np.array([entry.Q_W, entry.Q_X, entry.Q_Y, entry.Q_Z])  # (w, x, y, z) quaternion
        imagefile = entry.ImageFile  # name of the corresponding image file

        # FIXME triple-check this
        rotation = quaternion.as_rotation_matrix(orien)
        #rotation[:, 1:] *= -1
        center = pos
        mat = np.identity(4)
        mat[:3, :3] = rotation
        mat[:3,  3] = center
        
        camera_poses.append(CameraPose(timestamp, imagefile, mat))

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

    write_SfM_log(TF, i_mapF, out_log_fpath)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert and AirSim recording airsim_rec.txt file into the Tanks and Temples .log file format"
    )

    parser.add_argument("in_rec_fpath", help="Path to the input airsim_rec.txt file")
    parser.add_argument("out_log_fpath", help="Path to save the .log output (with filename")
    parser.add_argument("images_folder", nargs="?", help="Input images folder path")

    args = parser.parse_args()

    if args.image_folder is None:
        args.image_folder = os.path.join(os.path.dirname(args.in_rec_fpath), "images")

    convert_rec_to_log(args.in_rec_fpath, args.out_log_fpath, args.image_folder)

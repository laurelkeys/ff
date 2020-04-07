import glob
import argparse

import numpy as np
import open3d as o3d


IDENTITY_4x4 = np.identity(4)


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

    # # FIXME
    # for x in range(n_of_views):
    #     # FIXME
    #     w = np.zeros((4,4))
    #     w[ 3,  3] = 1
    #     w[:3, :3] = r
    #     w[:3,  3] = translation
    #     A = matrix(w)
    #     T.append(A.I)
    #     # FIXME

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
    parser.add_argument("--formatp", default="png", help="Images format")

    args = parser.parse_args()

    convert_Meshroom_to_log(
        args.in_sfm_fname,
        args.out_log_fname,
        args.images_folder, args.formatp
    )
import os
import sys
import json
import numpy as np

#########################################

def del_key_inplace(obj, bad_key):
    ''' Remove entries of `bad_key` from `obj` (inplace)\n
        ref.: https://stackoverflow.com/a/20692955
    '''
    if isinstance(obj, dict):
        for key in list(obj.keys()):
            if key == bad_key:
                del obj[key]
            else:
                del_key_inplace(obj[key], bad_key)
    elif isinstance(obj, list):
        for i in reversed(range(len(obj))):
            if obj[i] == bad_key:
                del obj[i]
            else:
                del_key_inplace(obj[i], bad_key)
    else:
        # neither a dict nor a list, do nothing
        pass
    return obj

def del_keys_inplace(obj, bad_keys):
    for bad_key in bad_keys:
        del_key_inplace(obj, bad_key)
    return obj

#########################################

# FIXME (unfinished)

if __name__ == "__main__":
    airsim_data = {}
    image_files = []
    with open(sys.argv[1], 'r') as airsim_rec_file:
        first = True
        for line in airsim_rec_file:
            if first:
                first = False
                continue
            TimeStamp, POS_X, POS_Y, POS_Z, Q_W, Q_X, Q_Y, Q_Z, ImageFile = line.split('\t')
            ImageFile = ImageFile[:-1] # ignore \n
            image_files.append(ImageFile)
            airsim_data[ImageFile] = [
                int(TimeStamp), # int
                POS_X, POS_Y, POS_Z, # float
                Q_W, Q_X, Q_Y, Q_Z # float
            ]

    with open(sys.argv[2], 'r') as cameras_file:
        cameras = json.loads(cameras_file.read())

    unused_keys = ['version', 'featuresFolders', 'matchesFolders', 'intrinsics', 'metadata', 'resectionId']
    del_keys_inplace(cameras, unused_keys)

    # -> views              -> poses
    #     |-> viewId            |-> poseId
    #     |-> poseId            |-> pose
    #     |-> intrinsicId           |-> transform
    #     |-> path                      |-> rotation
    #     |-> width                     |-> center
    #     |-> height                |-> locked

    views = cameras['views'] # 'viewId', 'poseId', 'intrinsicId', 'path', 'width', 'height'
    poses = cameras['poses'] # 'poseId', 'pose'

    meshroom_data = {}
    print("timestamp tx ty tz qx qy qz qw")
    for view_data, pose_data in zip(views, poses):
        assert view_data['poseId'] == pose_data['poseId']

        transform = pose_data['pose']['transform']
        rotation = [float(_) for _ in transform['rotation']]
        center = [float(_) for _ in transform['center']]

        ##
        R = rotation
        R = np.array([
            R[0], -R[1], -R[2],
            R[3], -R[4], -R[5],
            R[6], -R[7], -R[8],
        ]).reshape((3, 3))
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0.00000001:
            s = 2 * np.sqrt(trace + 1)
            Qt = np.array([
                0.25 * s,
                (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
            ])
        else:
            s_next = [1, 2, 0]
            i = 0
            if R[1, 1] > R[0, 0]: i = 1
            if R[2, 2] > R[i, i]: i = 2
            j = s_next[i]
            k = s_next[j]

            s = 2 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1)
            axis = [None, None, None]
            axis[i] = 0.25 * s
            axis[j] = (R[j, i] + R[i, j]) / s
            axis[k] = (R[k, i] + R[i, k]) / s
            Qt = np.array([
                (R[k, j] - R[j, k]) / s,
                axis[0],
                axis[1],
                axis[2],
            ])
        ##

        tx, ty, tz = center
        qw, qx, qy, qz = Qt

        meshroom_data[view_data['path']] = [
            tx, ty, tz,
            qx, qy, qz, qw,
        ]

    for full_path, data in meshroom_data.items():
        timestamp, *_ = airsim_data[os.path.basename(full_path)]
        meshroom_data[full_path].insert(0, timestamp)

    prefix = "D:/Documents/AirSim/2020-05-21-21-02-46/images/" # FIXME
    for image_file in image_files:
        print('\t'.join(str(_) for _ in meshroom_data[prefix + image_file]))

# NOTE
# - timestamp (float):
#     gives the number of seconds since the Unix epoch.
# - tx ty tz (3 floats):
#     give the position of the optical center of the color camera
#     with respect to the world origin as defined by the motion capture system.
# - qx qy qz qw (4 floats):
#     give the orientation of the optical center of the color camera in form of a unit quaternion
#     with respect to the world origin as defined by the motion capture system.

# ref.:
# https://code.woboq.org/qt5/qtbase/src/gui/math3d/qquaternion.cpp.html
# https://github.com/alicevision/meshroom/blob/bc1eb83d92048e6f888c4762c7ffcaab50395da6/meshroom/ui/reconstruction.py#L293
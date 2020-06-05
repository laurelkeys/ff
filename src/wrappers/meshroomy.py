import json

import numpy as np
import quaternion

###############################################################################
###############################################################################


def qt_from_rotation_matrix(rot):
    """ See https://code.woboq.org/qt5/qtbase/src/gui/math3d/qquaternion.cpp.html """
    assert rot.shape == (3, 3), rot

    if (trace := rot.trace()) > 0.00000001:
        s = 2 * np.sqrt(trace + 1)
        scalar = 0.25 * s
        axis = np.array(
            [
                (rot[2, 1] - rot[1, 2]) / s,
                (rot[0, 2] - rot[2, 0]) / s,
                (rot[1, 0] - rot[0, 1]) / s,
            ]
        )

    else:
        s_next = [1, 2, 0]
        # i = max([(_, rot[_, _]) for _ in range(1, 3)], key=lambda _: _[1])[1]
        i = 0
        if rot[1, 1] > rot[0, 0]:
            i = 1
        if rot[2, 2] > rot[i, i]:
            i = 2
        j = s_next[i]
        k = s_next[j]
        s = 2 * np.sqrt(rot[i, i] - rot[j, j] - rot[k, k] + 1)
        scalar = (rot[k, j] - rot[j, k]) / s
        axis = np.zeros(shape=(3,))
        axis[i] = 0.25 * s
        axis[j] = (rot[j, i] + rot[i, j]) / s
        axis[k] = (rot[k, i] + rot[i, k]) / s

    return np.array([scalar, *axis])


###############################################################################
###############################################################################


class MeshroomParser:
    @staticmethod
    def parse_cameras(cameras_file_path):
        """ Parses `cameras.json`, converted from `StructureFromMotion -> outputViewAndPoses`.\n
            Returns a tuple `(views, poses)`.
        """
        with open(cameras_file_path, "r") as cameras_file:
            cameras = json.loads(cameras_file.read())  # { version featuresFolders matchesFolders views intrinsics poses }
        views = cameras["views"]  # { viewId poseId intrinsicId resectionId path width height metadata }
        poses = cameras["poses"]  # { poseId pose { transform { rotation center } locked } }
        return views, poses

    @staticmethod
    def extract_transforms(poses):
        """ Returns a list of `(pose_id, rotation, center)` tuples. """
        transforms = []
        for poses_dict in poses:
            transforms.append(MeshroomParser.Pose.extract_from(poses_dict))
        return transforms

    class Pose:
        def __init__(self, poses_dict):
            self.pose_id, self.rotation, self.center = Pose.extract_from(poses_dict)

        @staticmethod
        def extract_from(poses_dict):
            """ Returns a `(pose_id, rotation, center)` tuple. """
            pose_id = poses_dict["poseId"]
            pose_dict = poses_dict["pose"]
            rotation, center = [list(map(float, _)) for _ in pose_dict["transform"].values()]
            return pose_id, rotation, center


class MeshroomTransform:
    """ See https://github.com/alicevision/meshroom/blob/develop/meshroom/ui/reconstruction.py """

    @staticmethod
    def translation(T, as_column_vector=False):
        """ T[0], T[1], T[2] """
        assert len(T) == 3, T
        if as_column_vector:
            return np.array(T).reshape((-1, 1))
        return np.array(T)

    @staticmethod
    def rotation(R, as_quaternion=True):
        """ R[0], -R[1], -R[2],\n
            R[3], -R[4], -R[5],\n
            R[6], -R[7], -R[8]
        """
        assert len(R) == 9, R
        rot = np.array([R[0], -R[1], -R[2], R[3], -R[4], -R[5], R[6], -R[7], -R[8],])
        if as_quaternion:
            # TODO compare results:
            return qt_from_rotation_matrix(rot)
            # return quaternion.from_rotation_matrix(rot)
        return rot

    @staticmethod
    def pose(R, T):
        """ R[0], -R[1], -R[2], T[0],\n
            R[3], -R[4], -R[5], T[1],\n
            R[6], -R[7], -R[8], T[2],\n
             0  ,   0  ,   0  ,  1
        """
        return np.vstack(
            (
                np.hstack(
                    (
                        MeshroomTransform.rotation(R, as_quaternion=False),
                        MeshroomTransform.translation(T, as_column_vector=True),
                    )
                ),
                [0, 0, 0, 1],
            )
        )


###############################################################################
###############################################################################

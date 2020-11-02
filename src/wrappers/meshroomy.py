import json

import numpy as np

###############################################################################
###############################################################################


class Quaternion:
    class XYZW:
        @staticmethod
        def from_rotation_matrix(rot):
            w, x, y, z = Quaternion.WXYZ.from_rotation_matrix(rot)
            return np.array([x, y, z, w])

    class WXYZ:
        @staticmethod
        def from_rotation_matrix(rot):
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
            Returns a tuple of lists `(views, poses)`.
        """
        with open(cameras_file_path, "r") as cameras_file:
            cameras = json.loads(cameras_file.read())  # { version featuresFolders matchesFolders views intrinsics poses }
        views = cameras["views"]  # { viewId poseId intrinsicId resectionId path width height metadata }
        poses = cameras["poses"]  # { poseId pose { transform { rotation center } locked } }
        return views, poses

    @staticmethod
    def extract_views_and_poses(views=None, poses=None):
        """ Returns two dictionaries, mapping:
            - `view_id` to `View` (if `views` is not None, else `{}`)
            - `pose_id` to `Pose` (if `poses` is not None, else `{}`)
            Note: `view_id == pose_id`.
        """
        _poses = {}
        for poses_dict in poses or []:
            pose_id, rotation, center = MeshroomParser.Pose.extract_from(poses_dict)
            _poses[pose_id] = MeshroomParser.Pose(rotation, center)

        _views = {}
        for views_dict in views or []:
            view_id, pose_id, path, width, height = MeshroomParser.View.extract_from(views_dict)
            _views[view_id] = MeshroomParser.View(pose_id, path, width, height)
            if _poses:
                assert pose_id in _poses, f"Unexpected poseId value for {views_dict=}"
            assert pose_id == view_id, f"Differing viewId and poseId in {views_dict=}"

        return _views, _poses

    class View:
        def __init__(self, pose_id, path, width, height):
            """ E.g.: `_, pose_id, path, width, height = extract_from(views_dict)` """
            self.pose_id = pose_id
            self.path = path
            self.width = width
            self.height = height

        @staticmethod
        def extract_from(views_dict):
            """ Returns a `(view_id, pose_id, path, width, height)` tuple. """
            view_id = views_dict["viewId"]
            pose_id = views_dict["poseId"]
            assert view_id == pose_id, views_dict
            path = views_dict["path"]
            width = views_dict["width"]
            height = views_dict["height"]
            return view_id, pose_id, path, width, height

    class Pose:
        def __init__(self, rotation, center):
            """ E.g.: `_, rotation, center = extract_from(poses_dict)` """
            self.rotation = rotation
            self.center = center

        def __repr__(self) -> str:
            return f"Pose(rotation={self.rotation}, center={self.center})"

        @staticmethod
        def extract_from(poses_dict):
            """ Returns a `(pose_id, rotation, center)` tuple. """
            pose_id = poses_dict["poseId"]
            pose_dict = poses_dict["pose"]
            rotation, center = [list(map(float, _)) for _ in pose_dict["transform"].values()]
            return pose_id, rotation, center


###############################################################################
###############################################################################


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
        rot = np.array(
            [[R[0], -R[1], -R[2]],
             [R[3], -R[4], -R[5]],
             [R[6], -R[7], -R[8]]]
        )
        if as_quaternion:
            return Quaternion.XYZW.from_rotation_matrix(rot)
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

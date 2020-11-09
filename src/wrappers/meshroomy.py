import os
import json

from typing import List

import numpy as np


###############################################################################
###############################################################################


class MeshroomParser:
    @staticmethod
    def parse_cameras(cameras_file_path):
        """ Parses `cameras.json`, converted from `StructureFromMotion > outputViewAndPoses`.\n
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


# TODO group View and Pose into this single class (and possibly remove them)
# https://github.com/alicevision/meshroom/blob/develop/meshroom/ui/reconstruction.py#L1100
# https://github.com/alicevision/meshroom/blob/develop/meshroom/ui/reconstruction.py#L169
class MeshroomViewpoint:
    """ Wraps the attributes of an input image (and its camera) in the context of a reconstruction. """

    def __init__(
        self, view_id: int, pose_id: int,
        # view
        path: str, width: int, height: int,
        # pose transform
        rotation: List[float], center: List[float]
    ):
        assert view_id == pose_id, f"Expected viewId={view_id} to be equal to poseId={pose_id}"
        assert os.path.isfile(path), f"Invalid image file path: '{path}'"

        self.id = view_id
        self.image_path = os.path.abspath(path)
        self.image_width = width
        self.image_height = height

        # # Store the rotation as a 3x3 numpy matrix and center as a numpy array.
        # self.rotation: np.ndarray = MeshroomTransform.rotation(rotation, as_xywz_quaternion=False)
        # self.center: np.ndarray = MeshroomTransform.translation(center)

        assert len(rotation) == 9, rotation
        assert len(center) == 3, center
        self.rotation = rotation
        self.center = center


###############################################################################
###############################################################################


class MeshroomTransform:
    """ Converts Meshroom's pose values into numpy arrays.

        See https://github.com/alicevision/meshroom/blob/develop/meshroom/ui/reconstruction.py
    """

    # TODO remove `as_column_vector` and `as_xywz_quaternion` and fix the scripts that use them
    # TODO prefix `translation`, `rotation` and `pose` with `parse_`

    @staticmethod
    def translation(T: List[float], as_column_vector: bool = False) -> np.ndarray:
        """ Get the camera translation as a numpy array.

            T[0], T[1], T[2]
        """
        assert len(T) == 3, T
        if as_column_vector:
            return np.array(T).reshape((-1, 1))
        return np.array(T)

    @staticmethod
    def rotation(R: List[float], as_xywz_quaternion: bool = True) -> np.ndarray:
        """ Get the camera rotation as a XYZW quaternion, or simply convert it to a 3x3 matrix.

            R[0], -R[1], -R[2],\n
            R[3], -R[4], -R[5],\n
            R[6], -R[7], -R[8]
        """
        assert len(R) == 9, R
        matrix = np.array([[R[0], -R[1], -R[2]], [R[3], -R[4], -R[5]], [R[6], -R[7], -R[8]]])
        if as_xywz_quaternion:
            return MeshroomQuaternion.XYZW.from_rotation_matrix(matrix)
        return matrix

    @staticmethod
    def unparse_rotation(R: np.ndarray) -> List[float]:
        """ Converts a 3x3 rotation matrix back into Meshroom's list representation of it. """
        assert R.shape == (3, 3), R
        R[:, 1:] *= -1  # undo the negation of the middle and last rows
        return R.flatten().tolist()  # NOTE flattens in row-major order

    @staticmethod
    def pose(R: List[float], T: List[float]) -> np.ndarray:
        """ Get the camera pose of as a 4x4 transformation matrix.

            R[0], -R[1], -R[2], T[0],\n
            R[3], -R[4], -R[5], T[1],\n
            R[6], -R[7], -R[8], T[2],\n
             0  ,   0  ,   0  ,  1
        """
        return np.vstack(
            (
                np.hstack(
                    (
                        MeshroomTransform.rotation(R, as_xywz_quaternion=False),
                        MeshroomTransform.translation(T, as_column_vector=True),
                    )
                ),
                [0, 0, 0, 1],
            )
        )


###############################################################################
###############################################################################


class MeshroomQuaternion:
    """ Mimics some of Qt's QQuaternion class functionality, which is used internally by Meshroom,
        but represents values as numpy arrays for convenience.

        See https://code.woboq.org/qt5/qtbase/src/gui/math3d/qquaternion.cpp.html
    """

    class XYZW:
        @staticmethod
        def to_rotation_matrix(Q: np.ndarray) -> np.ndarray:
            """ Creates a (row-major) 3x3 rotation matrix that corresponds to the quaternion. """
            x, y, z, w = Q
            return MeshroomQuaternion.WXYZ.to_rotation_matrix(np.array([w, x, y, z]))

        @staticmethod
        def from_rotation_matrix(R: np.ndarray) -> np.ndarray:
            """ Creates a quaternion that corresponds to the (row-major) 3x3 rotation matrix. """
            w, x, y, z = MeshroomQuaternion.WXYZ.from_rotation_matrix(R)
            return np.array([x, y, z, w])

    class WXYZ:
        @staticmethod
        def to_rotation_matrix(Q: np.ndarray) -> np.ndarray:
            """ See http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q54

                Note: the returned matrix is row-major.
            """
            assert Q.shape == (4, ), Q
            w, x, y, z = Q

            xx = x * x
            xy = x * y; yy = y * y
            xz = x * z; yz = y * z; zz = z * z
            xw = x * w; yw = y * w; zw = z * w

            return np.array(
                [
                    [1 - 2 * (yy + zz),     2 * (xy + zw),     2 * (xz - yw)],
                    [    2 * (xy - zw), 1 - 2 * (xx + zz),     2 * (yz + xw)],
                    [    2 * (xz + yw),     2 * (yz - xw), 1 - 2 * (xx + yy)],
                ]
            )

        @staticmethod
        def from_rotation_matrix(R: np.ndarray) -> np.ndarray:
            """ See http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q55

                Note: `R` is assumed to be row-major.
            """
            assert R.shape == (3, 3), R

            axis = np.zeros(shape=(3,))

            if (trace := R.trace()) > 0.00000001:
                s = 2 * np.sqrt(trace + 1)
                scalar  = 0.25 * s
                axis[0] = (R[1, 2] - R[2, 1]) / s
                axis[1] = (R[2, 0] - R[0, 2]) / s
                axis[2] = (R[0, 1] - R[1, 0]) / s

            else:
                # i = 0
                # if R[1, 1] > R[0, 0]: i = 1
                # if R[2, 2] > R[i, i]: i = 2
                i = max([(i, R[i, i]) for i in range(1, 3)], key=lambda _: _[1])[0]

                s_next = [1, 2, 0]  # map 0 to 1, 1 to 2 and 2 to 0
                j = s_next[i]
                k = s_next[j]

                s = 2 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1)
                axis[i] = 0.25 * s
                scalar  = (R[j, k] - R[k, j]) / s
                axis[j] = (R[i, j] + R[j, i]) / s
                axis[k] = (R[i, k] + R[k, i]) / s

            return np.array([scalar, *axis])


###############################################################################
###############################################################################

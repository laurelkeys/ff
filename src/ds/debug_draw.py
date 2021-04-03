import numpy as np


class DDraw:
    """A set of functions that generate a list of lines (represented as ndarray pairs)
    that can be used to later be drawn by Open3D, AirSim, etc."""

    @staticmethod
    def __normalize(vector):
        return vector / np.linalg.norm(vector)

    @staticmethod
    def __xyz_axis(origin, target, up):
        assert origin.shape == target.shape == up.shape == (3,)
        z_axis = DDraw.__normalize(origin - target)
        x_axis = DDraw.__normalize(np.cross(up, z_axis))
        y_axis = np.cross(z_axis, x_axis)
        # xyz_axis = (x_axis, y_axis, z_axis)
        # translation = np.array([-np.dot(_, origin) for _ in xyz_axis])
        # look_at = np.vstack(xyz_axis)
        # look_at = np.hstack((look_at, translation))
        # look_at = np.vstack((look_at, np.array([0, 0, 0, 1])))
        # assert look_at.shape == (4, 4)
        return x_axis, y_axis, z_axis

    @staticmethod
    def axis(origin, target, up, xyz_scaling=(1.0, 1.0, 1.0)):
        x_axis, y_axis, z_axis = DDraw.__xyz_axis(origin, target, up)
        sx, sy, sz = xyz_scaling

        return [(origin, sx * x_axis), (origin, sy * y_axis), (origin, sz * z_axis)]

    @staticmethod
    def frustum(origin, target, up, xyz_scaling=(1.0, 1.0, 1.0)):
        x_axis, y_axis, z_axis = DDraw.__xyz_axis(origin, target, up)
        sx, sy, sz = xyz_scaling

        xy_center = origin + sz * z_axis
        br, tr, tl, bl = (
            xy_center + sx * x_axis - sy * y_axis,  # bottom right
            xy_center + sx * x_axis + sy * y_axis,  # top right
            xy_center - sx * x_axis + sy * y_axis,  # top left
            xy_center - sx * x_axis - sy * y_axis,  # bottom left
        )

        return [
            (origin, br),
            (origin, tr),
            (origin, tl),
            (origin, bl),
            (br, tr),
            (tr, tl),
            (tl, bl),
            (bl, br),
        ]

from math import sqrt

###############################################################################
###############################################################################


def to_xyz_tuple(vector3r):
    return (vector3r.x_val, vector3r.y_val, vector3r.z_val)


def to_xyzw_tuple(quaternionr):
    return (quaternionr.x_val, quaternionr.y_val, quaternionr.z_val, quaternionr.w_val)


def xyz_to_str(xyz, n=2, show_hints=True):
    if show_hints:
        return f"(x={xyz[0]:.{n}f}, y={xyz[1]:.{n}f}, z={xyz[2]:.{n}f})"
    return f"({xyz[0]:.{n}f}, {xyz[1]:.{n}f}, {xyz[2]:.{n}f})"


def xyzw_to_str(xyzw, n=2, show_hints=True):
    if show_hints:
        return f"(x={xyzw[0]:.{n}f}, y={xyzw[1]:.{n}f}, z={xyzw[2]:.{n}f}, w={xyzw[3]:.{n}f})"
    return f"({xyzw[0]:.{n}f}, {xyzw[1]:.{n}f}, {xyzw[2]:.{n}f}, {xyzw[3]:.{n}f})"


def angles_to_str(angles, n=4, show_hints=True):
    if show_hints:
        return f"(pitch={angles[0]:.{n}f}, roll={angles[1]:.{n}f}, yaw={angles[2]:.{n}f})"
    return f"({angles[0]:.{n}f}, {angles[1]:.{n}f}, {angles[2]:.{n}f})"


def to_xyz_str(vector3r, n=2, show_hints=True):
    return xyz_to_str(to_xyz_tuple(vector3r), n, show_hints)


def to_xyzw_str(quaternionr, n=2, show_hints=True):
    return xyzw_to_str(to_xyzw_tuple(quaternionr), n, show_hints)


###############################################################################
###############################################################################


class Vec3:
    """ Simple 3D vector class to abstract operations
        with `airsim.GeoPoint` and `airsim.Vector3r`
    """

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "(%.4f, %.4f, %.4f)" % (self.x, self.y, self.z)

    def __eq__(self, v):
        if not isinstance(v, Vec3):
            return False
        return self.x == v.x and self.y == v.y and self.z == v.z

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def __add__(self, v):
        if not isinstance(v, Vec3):
            return Vec3(self.x + v, self.y + v, self.z + v)
        return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)

    def __sub__(self, v):
        return self + (-v)

    def __mul__(self, v):
        if not isinstance(v, Vec3):
            return Vec3(self.x * v, self.y * v, self.z * v)
        return Vec3(self.x * v.x, self.y * v.y, self.z * v.z)

    def __rmul__(self, v):
        return self * v

    def __div__(self, v):
        if not isinstance(v, Vec3):
            return Vec3(self.x / v, self.y / v, self.z / v)
        return Vec3(self.x / v.x, self.y / v.y, self.z / v.z)

    def __iter__(self):
        return iter((self.x, self.y, self.z))

    @staticmethod
    def cross(v1, v2):
        x = v1.y * v2.z - v1.z * v2.y
        y = v1.z * v2.x - v1.x * v2.z
        z = v1.x * v2.y - v1.y * v2.x
        return Vec3(x, y, z)

    @staticmethod
    def dot(v1, v2):
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    def length_squared(self):
        return Vec3.dot(self, self)

    def length(self):
        return sqrt(self.length_squared())

    def normalized(self):
        l = self.length()
        return Vec3(self.x / l, self.y / l, self.z / l)

    @staticmethod
    def from_GeoPoint(geopoint):
        """ `Vec3(latitude, longitude, altitude)` """
        return Vec3(geopoint.latitude, geopoint.longitude, geopoint.altitude)

    @staticmethod
    def from_Vector3r(vector3r):
        """ `Vec3(x_val, y_val, z_val)` """
        return Vec3(vector3r.x_val, vector3r.y_val, vector3r.z_val)

    @staticmethod
    def flip_z(v):
        """ `Vec3(x, y, -z)` """
        return Vec3(v.x, v.y, -v.z)

    def __getitem__(self, item):
        assert 0 <= item < 3
        return self.x if item == 0 else self.y if item == 1 else self.z

    def __len__(self):
        return 3

    @staticmethod
    def all_close(a, b, eps=1e-7):
        """ Returns true iff `a` and `b` are element-wise equal within `eps` tolerance """
        return abs(a.x - b.x) <= eps and abs(a.y - b.y) <= eps and abs(a.z - b.z) <= eps


###############################################################################
###############################################################################

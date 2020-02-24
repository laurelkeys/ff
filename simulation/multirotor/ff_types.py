from math import sqrt


class Vec3:
    ''' Simple 3D vector class to abstract operations with airsim.GeoPoint and airsim.Vector3r '''

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
            assert v != 0
            return Vec3(self.x / v, self.y / v, self.z / v)
        assert v.x != 0 and v.y != 0 and v.z != 0
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
        return sqrt(self.length_squared)

    def normalized(self):
        length = self.length()
        assert length != 0
        return self / length

    @staticmethod
    def from_GeoPoint(geopoint):
        ''' Vec3(latitude, longitude, altitude) '''
        return Vec3(
            geopoint.latitude,
            geopoint.longitude,
            geopoint.altitude
        )

    @staticmethod
    def from_Vector3r(vector3r):
        ''' Vec3(x_val, y_val, z_val) '''
        return Vec3(
            vector3r.x_val,
            vector3r.y_val,
            vector3r.z_val
        )

    @staticmethod
    def flip_z(v):
        ''' Vec3(x, y, -z) '''
        return Vec3(v.x, v.y, -v.z)

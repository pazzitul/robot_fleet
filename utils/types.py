class Point(object):
    """@Describe 3d double value point

    Attributes:
    x:
    y:
    z:
    """
    def __init__(self, x=0, y=0, z=0):
        super(Point, self).__init__()
        self._x = x
        self._y = y
        self._z = z


class Vector3d(object):
    """@Describes 3d vector

    Attributes:
        x:
        y:
        z:
    """
    def __init__(self, x=0, y=0, z=0):
        super(Vector3d, self).__init__()
        self._x = x
        self._y = y
        self._z = z


class Quaterniond(object):
    """@Quaternion with double value

    Attributes:
        x:
        y:
        z:
        w:
    """
    def __init__(self, x=0, y=0, z=0, w=1):
        super(Quaterniond, self).__init__()
        self._x = x
        self._y = y
        self._z = z
        self._w = w


class Pose(object):
    """@Contains position and orientation

    Attributes:
        position:
        orientation:
    """
    def __init__(self, vec3, quat):
        super(Pose, self).__init__()
        self._position = vec3
        self._orientation = quat


class Twist(object):
    """@twist contains linear and angular

    Attributes:
        linear:
        angular:
    """
    def __init__(self, lr, ar):
        super(Twist, self).__init__()
        self._linear = lr
        self._angular = ar

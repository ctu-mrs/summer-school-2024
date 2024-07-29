#!/usr/bin/env python3

import numpy as np

# MRS ROS messages
from mrs_msgs.msg import TrajectoryReference, Reference
from geometry_msgs.msg import Point as ROSPoint
from mrim_resources.msg import InspectionPoint, InspectionProblem

## | ------------------------- Classes ------------------------ |

# # #{ class Point
class Point:
    '''
    Data type class for 3D points.

    Basic usage:

        point     = Point(x, y, z)
        point2    = Point([x, y, z])
        x_smaller = point.x - 1.0

    '''

    def __init__(self, *args):
        # Point(x, y, z), or Point([x, y, z])
        if len(args) >= 1:
            self.x = args[0]
            self.y = args[1]
            self.z = args[2]
        else:
            print('[ERROR] Trying to create Point object from incorrect input values. Ending. Values:', args)
            exit(-5)

    def asArray(self):
        return np.array(self.asList())

    def asList(self):
        return [self.x, self.y, self.z]

    def asTuple(self):
        return (self.x, self.y, self.z)

    def norm(self):
        return np.linalg.norm(self.asList())

    def __str__(self):
        return '({:.2f}, {:.2f}, {:.2f})'.format(self.x, self.y, self.z)

    def __eq__(self, other):
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y and self.z == other.y
        return False

    def __add__(self, other):
        if isinstance (other, Point):
            return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        if isinstance (other, Point):
            return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        if isinstance (other, Point):
            return Point(self.x * other.x, self.y * other.y, self.z * other.z)
        elif isinstance (other, float):
            return Point(other * self.x, other * self.y, other * self.z)
    __rmul__ = __mul__

    def __truediv__(self, other):
        if isinstance (other, float):

            if np.abs(other) < 1e-8:
                print('[ERROR] Trying to create Point object from incorrect input values. Ending. Values:', args)
                exit(-5)

            return Point(self.x / other, self.y / other, self.z / other)

# # #}

# # #{ class Pose
class Pose:
    '''
    Data type class for poses (3D coordinates and heading).

    Basic usage:

        point     = Point(x, y, z)
        heading   = np.pi
        pose      = Pose(point, heading)

        pose2     = Pose(x, y, z, heading)
        pose3     = Pose([x, y, z, heading])
    '''

    def __init__(self, *args):
        # Point(x, y, z), float(heading)
        if len(args) == 2 and isinstance(args[0], Point) and (isinstance(args[1], float) or args[1] is None):
            self.point   = args[0]
            self.heading = args[1]
        # Point([x, y, z, heading])
        elif isinstance(args, (tuple, list)):
            self.point   = Point(args[0], args[1], args[2])
            self.heading = args[3]
        else:
            print('[ERROR] Trying to create Pose object from incorrect input values. Ending.')
            exit(-4)

    def asArray(self):
        return np.array(self.asList())

    def asList(self):
        return [self.point.x, self.point.y, self.point.z, self.heading]

    def __str__(self):
        return '({:.2f}, {:.2f}, {:.2f}, {:.2f})'.format(self.point.x, self.point.y, self.point.z, self.heading)

    def __eq__(self, other):
        if isinstance(other, Pose):
            return self.point == other.point and self.heading == other.heading
        return False

# # #}

# # #{ class Viewpoint
class Viewpoint:
    '''
    Data type class for viewpoints (Pose with index).

    Basic usage:

        pose     = Pose(x, y, z, heading)
        vp       = Viewpoint(0, pose)
        vp_type  = Viewpoint type: 's' for solar panel viewpoint, 't' for tower viewpoint
    '''

    def __init__(self, idx, pose, vp_type=None):
        self.idx  = idx
        self.pose = pose
        self.type = vp_type

    def __eq__(self, other):
        if isinstance(other, Viewpoint):
            return self.idx == other.idx
        return False

# # #}

# # #{ class Bounds
class Bounds:

    def __init__(self, point_min, point_max):
        self.point_min = point_min
        self.point_max = point_max

    def valid(self, point):
        '''
        Checks whether a point lies within the stored bounds.

        Parameters:
            point (Point): the point

        Returns:
            bool: True if in bounds, False otherwise
        '''

        point_list     = point.asList()
        point_min_list = self.point_min.asList()
        point_max_list = self.point_max.asList()

        for i in range(3):
            if point_list[i] < point_min_list[i] or point_list[i] > point_max_list[i]:
                return False
        return True

# # #}

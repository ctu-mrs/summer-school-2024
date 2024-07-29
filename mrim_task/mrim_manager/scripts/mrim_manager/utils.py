#!/usr/bin/python3

# #{ imports

import rospy
import math
import os
import numpy as np
import tf
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d
import sensor_msgs
import sensor_msgs.point_cloud2
import tf2_geometry_msgs
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, Vector3, Vector3Stamped, TransformStamped

# #} end of imports

# #{ HELPER FUNCTIONS

def boolToString(state):
    return "OK" if state else "FAILED"

def wrapAngle(angle):
    # wrap to interval <-pi, pi)
    return (angle + np.pi) % (2 * np.pi) - np.pi

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc. (https://stackoverflow.com/questions/13685386/
    matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to)

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def trajectoryToPathMsg(trajectory, frame):
    path = Path()
    path.header.frame_id = frame
    for k in range(len(trajectory.poses)):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame
        pose_stamped.pose.position = Point(trajectory.poses[k].x, trajectory.poses[k].y, trajectory.poses[k].z)
        path.poses.append(pose_stamped)

    return path

def geometryMsgsPointsToPclCloud(points):
    cloud = np.zeros((len(points), 3), dtype=np.dtype('float32'))
    cloud[...,0] = [p.x for p in points]
    cloud[...,1] = [p.y for p in points]
    cloud[...,2] = [p.z for p in points]
    return cloud

def trajectoryPointToPoseMsg(point):
    msg = Pose()
    msg.position = Point(point.x, point.y, point.z)
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point.heading)
    msg.orientation.x = quaternion[0]
    msg.orientation.y = quaternion[1]
    msg.orientation.z = quaternion[2]
    msg.orientation.w = quaternion[3]
    return msg

def uavStateMsgToTrajectoryPoint(uav_state):
    x_stamped = Vector3Stamped()
    x_stamped.vector = Vector3(1, 0, 0)
    t = TransformStamped()
    t.transform.rotation = uav_state.pose.orientation
    x_new = tf2_geometry_msgs.do_transform_vector3(x_stamped, t)

    if abs(x_new.vector.x) <= 1e-3 and abs(x_new.vector.y) <= 1e-3:
        print("Cannot extract quaternion to heading!")
        heading = None
    else:
        heading = math.atan2(x_new.vector.y, x_new.vector.x)

    return Vector4d(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z, heading)

def trajectoryToOdometryMsg(trajectory, trajectory_idx, frame):
    odom = Odometry()
    odom.header.frame_id = frame
    idx = min(trajectory_idx, len(trajectory.poses) - 1)
    odom.pose.pose.position = Point(trajectory.poses[idx].x, trajectory.poses[idx].y, trajectory.poses[idx].z)
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, trajectory.poses[idx].heading)
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]
    odom.twist.twist.linear = Vector3(trajectory.velocities[idx].x, trajectory.velocities[idx].y, trajectory.velocities[idx].z)
    odom.twist.twist.angular = Vector3(0.0, 0.0, trajectory.velocities[idx].heading)

    return odom

def getTransitionPointDist(point1, point2):
    return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z -point2.z)**2)

# #} end of HELPER FUNCTIONS

# #{ class Vector4d

class Vector4d:
    def __init__(self, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading

    def norm3d(self, vp2):
        return np.sqrt((self.x - vp2.x)**2 + (self.y - vp2.y)**2 + (self.z - vp2.z)**2)

    def diff3d(self, vp2):
        return [self.x - vp2.x, self.y - vp2.y, self.z - vp2.z]

    def derivation3d(self, vp2, ts):
        return [(self.x - vp2.x)/ts, (self.y -vp2.y)/ts, (self.z -vp2.z)/ts]

# #} end of class Vector4d

# #{ class DynamicConstraint

class DynamicConstraint:
    def __init__(self, speed, acceleration, jerk, snap):
        self.speed = speed
        self.acceleration = acceleration
        self.jerk = jerk
        self.snap = snap

# #} end of class DynamicConstraint

# #{ class Constraints

class Constraints:
    def __init__(self, horizontal, ascending, descending, heading):
        self.horizontal = horizontal
        self.ascending = ascending
        self.descending = descending
        self.heading = heading

# #} end of class Constraints

# #{ class Trajectory

class Trajectory:
    def __init__(self, waypoint_list, dt, trajectory_name):
        self.poses = waypoint_list
        self.dt = dt
        self.trajectory_name = trajectory_name
        self.velocities = self.getDerivation(self.poses, self.dt, True, True)
        self.accelerations = self.getDerivation(self.velocities, self.dt, False, False)
        self.jerks = self.getDerivation(self.accelerations, self.dt, False, False)
        self.snaps = self.getDerivation(self.jerks, self.dt, False, False)
        self.cummulative_length = self.getCummulativeLength(self.poses)
        self.abs_velocities = self.getNorms(self.velocities)
        self.abs_accelerations = self.getNorms(self.accelerations)
        self.length = self.cummulative_length[-1]
        self.time = len(self.poses)*dt
        self.min_obst_dist = -1
        self.min_mutual_dist = -1
        self.dynamics_ok = False

    def getDerivation(self, vector, dt, wrap_heading, first_derivation):
        derivatives = []
        derivatives.append(Vector4d(0.0, 0.0, 0.0, 0.0))

        for k in range(1, len(vector)-1):
            dx = (vector[k].x - vector[k-1].x)/dt
            dy = (vector[k].y - vector[k-1].y)/dt
            dz = (vector[k].z - vector[k-1].z)/dt
            if first_derivation:
                dheading = self.getHeadingDiff(vector[k-1].heading, vector[k].heading)/dt if wrap_heading else (vector[k].heading - vector[k-1].heading)/dt
            else:
                dheading = (vector[k].heading - vector[k-1].heading)/dt
            derivatives.append(Vector4d(dx, dy, dz, dheading))

        derivatives.append(Vector4d(0.0, 0.0, 0.0, 0.0))
        return derivatives

    def getNorms(self, vector):
        norms = []
        norms.append(0.0)

        for k in range(1, len(vector)):
            norms.append(np.sqrt(vector[k].x**2+vector[k].y**2+vector[k].z**2))

        return norms

    def getCummulativeLength(self, poses):
        lengths = []
        lengths.append(0.0)

        for k in range(1, len(poses)):
            dx = poses[k].x - poses[k-1].x
            dy = poses[k].y - poses[k-1].y
            dz = poses[k].z - poses[k-1].z
            lengths.append(lengths[k-1] + np.sqrt(dx**2+dy**2+dz**2))

        return lengths

    def getHeadingDiff(self, h1, h2):
        h1n = math.atan2(np.sin(h1), np.cos(h1))
        h2n = math.atan2(np.sin(h2), np.cos(h2))
        diff = h2n - h1n
        diffn = diff if abs(diff) < math.pi else -np.sign(diff) * (2*math.pi - abs(diff))
        return diffn

    def setStatistics(self, min_obst_dist, min_mutual_dist, dynamics_ok, overall_status):
        self.min_obst_dist = min_obst_dist
        self.min_mutual_dist = min_mutual_dist
        self.dynamics_ok = dynamics_ok
        self.overall_status = overall_status

# #} end of class Trajectory

# #{ class Viewpoint

class Viewpoint:
    def __init__(self, color_index, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading
        self.is_inspected = False
        self.color_index = color_index

# #} end of class Viewpoint

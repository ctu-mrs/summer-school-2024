#!/usr/bin/python3

# #{ imports

import rospy
import rosnode
import math
import numpy as np
import re
from copy import deepcopy
from sklearn.neighbors import KDTree
import sensor_msgs
import sensor_msgs.point_cloud2
from std_msgs.msg import ColorRGBA, Float32, Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, Vector3, TransformStamped
from mrs_msgs.msg import TrajectoryReference
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import *
from mrim_resources.msg import InspectionProblem
from mrim_manager.utils import *

# #} end of imports

# #{ class TaskMonitor

class TaskMonitor:
    def __init__(self, trajectories, pcl_kdtree, initial_uav_states, obst_dist_limit, mutual_dist_limit, dynamics_checks_ok):
        self.min_mutual_dists = [1e6, 1e6]
        self.min_obstacle_dists = [1e6, 1e6]
        self.velocities = [0, 0]
        self.accelerations = [0, 0]
        self.travelled_dists = [0, 0]
        self.overall_statuses = dynamics_checks_ok
        self.mission_time = 0.0
        self.start_time = 0.0
        self.final_time = 0.0
        self.pcl_kdtree = pcl_kdtree
        self.obst_dist_limit = obst_dist_limit
        self.mutual_dist_limit = mutual_dist_limit
        first_poses_received = False
        self.poses = []
        for uav_state in initial_uav_states:
            self.poses.append(uavStateMsgToTrajectoryPoint(uav_state))

    def getFinalTime(self):
        return (self.final_time - self.start_time).to_sec()

    def start(self):
        self.start_time = rospy.Time.now()

    def stop(self):
        self.final_time = rospy.Time.now()

    def update(self, uav_states):

        poses = []

        for uav_state in uav_states:
            poses.append(uavStateMsgToTrajectoryPoint(uav_state))

        self.updateDynamics(uav_states)

        self.updateTravelledDists(poses)

        mutual_dists = self.getMutualDists(poses)

        obst_dists = self.getObstacleDists(poses)

        mission_time = (rospy.Time.now() - self.start_time).to_sec()

        self.poses = poses

        for k in range(len(poses)):
            if self.min_mutual_dists[k] < self.mutual_dist_limit or self.min_obstacle_dists[k] < self.obst_dist_limit:
                self.overall_statuses[k] = False

        return mission_time, self.travelled_dists, obst_dists, mutual_dists, self.min_obstacle_dists,\
            self.min_mutual_dists, self.velocities, self.accelerations, self.overall_statuses, self.poses

    def getMutualDists(self,poses):

        # FIXME: curently expects only two UAVs

        mutual_dist = np.sqrt((poses[0].x - poses[-1].x)**2 + (poses[0].y - poses[-1].y)**2 + (poses[0].z - poses[-1].z)**2)
        mutual_dists = [mutual_dist, mutual_dist]
        for k in range(len(mutual_dists)):
            if mutual_dists[k] < self.min_mutual_dists[k]:
                self.min_mutual_dists[k] = mutual_dists[k]

        return [mutual_dist, mutual_dist]

    def getObstacleDists(self, poses):

        obstacle_dists = []
        for k in range(len(poses)):
            obstacle_dists.append(self.pcl_kdtree.getMinDist(poses[k]).item())

        for k in range(len(obstacle_dists)):
            if obstacle_dists[k] > -1 and obstacle_dists[k] < self.min_obstacle_dists[k]:
                self.min_obstacle_dists[k] = obstacle_dists[k]

        return obstacle_dists

    def updateTravelledDists(self, poses):
        for k in range(len(poses)):
            d = np.sqrt((poses[k].x - self.poses[k].x)**2 + (poses[k].y - self.poses[k].y)**2 + (poses[k].z - self.poses[k].z)**2)
            self.travelled_dists[k] += d

    def updateDynamics(self, uav_states):
        for k in range(len(uav_states)):
            v = uav_states[k].velocity.linear
            self.velocities[k] = np.sqrt(v.x**2 + v.y**2 + v.z**2)
            a = uav_states[k].acceleration.linear
            self.accelerations[k] = np.sqrt(a.x**2 + a.y**2 + a.z**2)

# #} end of class TaskMonitor

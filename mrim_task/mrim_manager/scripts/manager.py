#!/usr/bin/python3

# #{ imports

import rospy
import rosnode
import subprocess
import math
import signal
import os
import numpy as np
import tf
import re
from threading import RLock
from numpy import cos, sin
from copy import deepcopy
from sklearn.neighbors import KDTree
import sensor_msgs
import sensor_msgs.point_cloud2
from std_msgs.msg import ColorRGBA, Float32, Empty, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, Vector3, TransformStamped
from mrs_msgs.msg import TrajectoryReference, UavState
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import *
from mrim_resources.msg import InspectionProblem
from mrim_manager.visualizer import *
from mrim_manager.utils import *
from mrim_manager.task_monitor import *
from mrim_manager.python_plotter import *
# #} end of imports

# #{ signal_handler

def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)

# #} end of signal_handler

# #{ class Evaluator

class Evaluator:
    def __init__(self, inspection_problem, viewpoints_distance, allowed_dist_deviation, allowed_heading_deviation, allowed_pitch_deviation, horizontal_aovs):
        # load problem
        self.viewpoints_distance = viewpoints_distance
        self.inspection_problem = inspection_problem
        self.viewpoints = self.getInspectionViewpoints(inspection_problem)
        self.allowed_dist_dev = allowed_dist_deviation
        self.allowed_heading_dev = allowed_heading_deviation
        self.allowed_pitch_dev = allowed_pitch_deviation
        self.horizontal_aovs = horizontal_aovs
        self.start_poses = inspection_problem.start_poses
        self.zero_score = False

    def getScore(self):
        score = sum([vp.is_inspected for vp in self.viewpoints]) if not self.zero_score else 0
        return score

    def resetScore(self):
        self.zero_score = False
        for vp in self.viewpoints:
            vp.is_inspected = False

    def setZeroScore(self):
        self.zero_score = True

    def checkFinalPositions(self, poses):
        result = True
        for k in range(len(poses)):
            dist = np.sqrt((poses[k].x - self.start_poses[k].position.x)**2 + (poses[k].y - self.start_poses[k].position.y)**2 + (poses[k].z - self.start_poses[k].position.z)**2)
            if dist > 1.0: #TODO: add param if needed
                result = False

        return result

    def updateInspectionStatus(self, pose1, pose2):

        poses = [pose1, pose2]
        for k in range(len(self.inspection_problem.inspection_points)):
            for r in range(self.inspection_problem.number_of_robots):
                # add inspection points to be inspected by robot r
                if self.inspection_problem.robot_ids[r] in self.inspection_problem.inspection_points[k].inspectability: #future fix: fix for other idxs than 0 1
                    if not self.viewpoints[k].is_inspected:
                        self.viewpoints[k].is_inspected = self.isPointInspected(self.viewpoints[k], self.inspection_problem.inspection_points[k], poses[r], r)

    def isPointInspected(self, viewpoint, inspection_point, pose, robot_index):
        dev_xy = np.sqrt((inspection_point.position.x - pose.x)**2 + (inspection_point.position.y - pose.y)**2)
        dist_dev = abs(np.sqrt(dev_xy**2 + (inspection_point.position.z - pose.z)**2) - self.viewpoints_distance)
        heading = wrapAngle(math.atan2(inspection_point.position.y - pose.y, inspection_point.position.x - pose.x))
        heading_dev = abs(wrapAngle(heading - viewpoint.heading))
        is_in_hfov = abs(wrapAngle(pose.heading - heading)) < self.horizontal_aovs[robot_index]/2
        pitch_dev = abs(math.atan2(inspection_point.position.z - pose.z, dev_xy))
        return dist_dev <= self.allowed_dist_dev and heading_dev <= self.allowed_heading_dev and pitch_dev <= self.allowed_pitch_dev and is_in_hfov

    def inspectionPointToViewPoint(self, inspection_point):
        x       = inspection_point.position.x + self.viewpoints_distance * np.cos(inspection_point.inspect_heading)
        y       = inspection_point.position.y + self.viewpoints_distance * np.sin(inspection_point.inspect_heading)
        z       = inspection_point.position.z
        heading = np.unwrap([inspection_point.inspect_heading + np.pi])[0]
        color_index = inspection_point.inspectability[0] if len(inspection_point.inspectability) == 1 else 0

        return Viewpoint(color_index, x, y, z, heading)

    def getInspectionViewpoints(self, inspection_problem):

        viewpoints = []
        for ip in inspection_problem.inspection_points:
            viewpoints.append(self.inspectionPointToViewPoint(ip))

        return viewpoints

# #} end of class Evaluator

# #{ class PclKDTree

class PclKDTree:
    def __init__(self, points):
        self.initialized = False
        cloud = geometryMsgsPointsToPclCloud(points)
        self.initializeKDTree(cloud)

    def initializeKDTree(self, cloud):
        if cloud is not None:
            self.tree = KDTree(cloud)
            self.initialized = True
            rospy.loginfo("[MrimManager] KDTree initialized.")
        else:
            rospy.logwarn("[MrimManager] Cannot initialize KDTree, input cloud not provided.")

    def getMinDist(self, point):
        if not self.initialized:
            rospy.logwarn("[MrimManager] Cannot provide minimum distance from point. KDTree not initialized.")
            return -1

        point_x = [[point.x, point.y, point.z]]
        nearest_dist, nearest_ind = self.tree.query(point_x, k=1)
        return nearest_dist

# #} end of class PclKDTree

# #{ class MrimManager

class MrimManager:

    # #{ __init__()

    def __init__(self):

        rospy.init_node('mrim_manager', anonymous=True)

        # #{ LOAD PARAMS

        self.subscribe_trajectories = rospy.get_param('~trajectories/subscribe')
        trajectory_folder = rospy.get_param('~trajectories/loading/folder')
        trajectory_files = rospy.get_param('~trajectories/loading/files')
        visualization_trajectory = rospy.get_param('~visualization/python/trajectories')
        visualization_dynamics = rospy.get_param('~visualization/python/dynamics')
        visualization_mutual_dist = rospy.get_param('~visualization/python/mutual_distance')
        visualization_obstacle_dist = rospy.get_param('~visualization/python/obstacle_distance')
        visualization_rviz = rospy.get_param('~visualization/rviz/use')
        playback_speed = rospy.get_param('~visualization/rviz/playback_speed')
        trajectory_dt = rospy.get_param('~trajectories/dt')
        minimum_mutual_distance = rospy.get_param('~trajectories/min_distance/mutual')
        minimum_obstacle_distance = rospy.get_param('~trajectories/min_distance/obstacles')
        rviz_config = rospy.get_param('~rviz_config')
        self.print_info = rospy.get_param('~print_info')
        global_frame = rospy.get_param('~global_frame')
        viewpoint_distance = rospy.get_param('~viewpoints/distance')
        allowed_dist_dev = rospy.get_param('~viewpoints/inspection_limits/distance')
        allowed_heading_dev = rospy.get_param('~viewpoints/inspection_limits/heading')
        allowed_pitch_dev = rospy.get_param('~viewpoints/inspection_limits/pitch')
        horizontal_aov_1 = rospy.get_param('~cameras/red/angle_of_view/horizontal')
        vertical_aov_1 = rospy.get_param('~cameras/red/angle_of_view/vertical')
        fov_length_1 = rospy.get_param('~cameras/red/fov_length')
        horizontal_aov_2 = rospy.get_param('~cameras/blue/angle_of_view/horizontal')
        vertical_aov_2 = rospy.get_param('~cameras/blue/angle_of_view/vertical')
        fov_length_2 = rospy.get_param('~cameras/blue/fov_length')
        self.mission_time_limit = rospy.get_param('~mission/timeout')
        self.visualization_horizon_length = rospy.get_param("~visualization/rviz/horizon_length")
        run_type = rospy.get_param("~run_type")
        flight_always_allowed = rospy.get_param("~simulation/flight_always_allowed", False)
        uav_names = rospy.get_param("~uav_names")
        self.solution_time_constraint_hard = rospy.get_param("~solution_time_constraint/hard")
        self.solution_time_constraint_soft = rospy.get_param("~solution_time_constraint/soft")

        # #{ LOAD CONSTRAINTS

        constraint_type = rospy.get_param('~dynamic_constraints/controller')

        # ends if constraints cannot be loaded

        h_speed = rospy.get_param('~dynamic_constraints/max_velocity/x')
        h_acc = rospy.get_param('~dynamic_constraints/max_acceleration/x')
        h_jerk = rospy.get_param('~' + constraint_type + '/horizontal/jerk')
        h_snap = rospy.get_param('~' + constraint_type + '/horizontal/snap')

        va_speed = rospy.get_param('~dynamic_constraints/max_velocity/z')
        va_acc = rospy.get_param('~dynamic_constraints/max_acceleration/z')
        va_jerk = rospy.get_param('~' + constraint_type + '/vertical/ascending/jerk')
        va_snap = rospy.get_param('~' + constraint_type + '/vertical/ascending/snap')

        vd_speed = rospy.get_param('~dynamic_constraints/max_velocity/z')
        vd_acc = rospy.get_param('~dynamic_constraints/max_acceleration/z')
        vd_jerk = rospy.get_param('~' + constraint_type + '/vertical/descending/jerk')
        vd_snap = rospy.get_param('~' + constraint_type + '/vertical/descending/snap')

        heading_speed = rospy.get_param('~dynamic_constraints/max_heading_rate')
        heading_acc = rospy.get_param('~dynamic_constraints/max_heading_acceleration')
        heading_jerk = rospy.get_param('~' + constraint_type + '/heading/jerk')
        heading_snap = rospy.get_param('~' + constraint_type + '/heading/snap')
        self.constraints_violation_tolerance = rospy.get_param('~dynamic_constraints/tolerance')

        self.constraints = Constraints(DynamicConstraint(h_speed, h_acc, h_jerk, h_snap), DynamicConstraint(va_speed, va_acc, va_jerk, va_snap), DynamicConstraint(vd_speed, vd_acc, vd_jerk, vd_snap), DynamicConstraint(heading_speed, heading_acc, heading_jerk, heading_snap))

        # #} end of LOAD CONSTRAINTS

        # #} end of LOAD PARAMS

        # #{ ROS PUBLISHERS

        pub_cloud = rospy.Publisher('cloud_out', sensor_msgs.msg.PointCloud2, queue_size=1)
        pub_safety_area = rospy.Publisher('safety_area_out', Marker, queue_size=1)
        pub_inspection_points = rospy.Publisher('inspection_points_out', Marker, queue_size=1)
        pub_viewpoints = rospy.Publisher('view_points_out', MarkerArray, queue_size=1)
        pub_collisions = rospy.Publisher('collisions_out', Marker, queue_size=1)
        pub_start_positions = rospy.Publisher('starting_points_out', MarkerArray, queue_size=1)
        pub_start_arrows = rospy.Publisher('start_arrows_out', MarkerArray, queue_size=1)
        pub_score = rospy.Publisher('score_out', Float32, queue_size=1)
        pub_mission_time = rospy.Publisher('mission_time_out', Float32, queue_size=1)
        pub_remaining_time = rospy.Publisher('remaining_mission_time_out', Float32, queue_size=1)
        pub_solution_time = rospy.Publisher('solution_time_out', Float32, queue_size=1)
        pub_solution_penalty = rospy.Publisher('solution_penalty_out', Float32, queue_size=1)
        pub_playback_status = rospy.Publisher('playback_status_out', OverlayText, queue_size=1)
        pub_diagnostic = rospy.Publisher('diagnostics_out', OverlayText, queue_size=1)
        pub_fullscreen = rospy.Publisher('fullscreen_msg_out', OverlayText, queue_size=1)

        self.pub_trajectories_valid_status = rospy.Publisher('trajectories_valid_status_out', Bool, queue_size=1)

        # #} end of ROS PUBLISHERS

        # #{ INITIALIZATION OF VARIABLES AND SUBSCRIBERS

        self.trajectory1_subscribed = False
        self.trajectory2_subscribed = False
        self.inspection_problem_subscribed = False
        self.playback_paused = False
        self.mission_time_exceeded = False
        self.mission_started = False
        self.mission_finished = False
        self.uav_states_lock = RLock()
        self.uav_states = [None, None] # expects two UAVs only
        self.solution_time_start = -1.0
        self.solution_time_end = -1.0

        rospy.Subscriber("inspection_problem_in", InspectionProblem, self.callbackInspectionProblem)
        rospy.Subscriber("pause_playback_in", Empty, self.callbackPausePlayback)
        rospy.Subscriber("uav_state_1_in", UavState, self.callbackUavState1)
        rospy.Subscriber("uav_state_2_in", UavState, self.callbackUavState2)

        while not self.inspection_problem_subscribed:
            rospy.loginfo("[MrimManager] Waiting for inspection problem.")
            rospy.Rate(0.5).sleep()

        minimum_height = self.inspection_problem.min_height
        maximum_height = self.inspection_problem.max_height

        self.pcl_map = PclKDTree(self.inspection_problem.obstacle_points)

        self.evaluator_ = Evaluator(self.inspection_problem, viewpoint_distance, allowed_dist_dev, allowed_heading_dev, allowed_pitch_dev, [horizontal_aov_1, horizontal_aov_2])

        rospy.loginfo("[MrimManager] Starting trajectory checker.")

        self.trajectories = []

        rospy.Subscriber("trajectory_1_in", TrajectoryReference, self.callbackFirstTrajectory)
        rospy.Subscriber("trajectory_2_in", TrajectoryReference, self.callbackSecondTrajectory)

        # #} end of INITIALIZATION OF VARIABLES AND SUBSCRIBERS

        # #{ INITALIZATION OF PUBLISHERS

        cones_publishers = []
        path_publishers = []
        odometry_publishers = []
        horizon_publishers = []
        obst_dist_publishers = []
        mutual_dist_publishers = []
        vel_publishers = []
        acc_publishers = []
        jsk_diagnostics_publishers = []
        traveled_dist_publishers = []
        for k in range(self.inspection_problem.number_of_robots):
            jsk_diagnostics_publishers.append(rospy.Publisher('/visualization/diagnostics_jsk_' + str(k+1), OverlayText, queue_size=1))
            cones_publishers.append(rospy.Publisher('/visualization/cone_' + str(k+1) , Marker, queue_size=1))
            path_publishers.append(rospy.Publisher('/visualization/path_' + str(k+1), Path, queue_size=1))
            odometry_publishers.append(rospy.Publisher('/visualization/odom_' + str(k+1), Odometry, queue_size=1))
            horizon_publishers.append(rospy.Publisher('/visualization/horizon_' + str(k+1), PoseArray, queue_size=1))
            obst_dist_publishers.append(rospy.Publisher('/visualization/obstacle_distance_' + str(k+1), Float32, queue_size=1))
            mutual_dist_publishers.append(rospy.Publisher('/visualization/mutual_distance_' + str(k+1) , Float32, queue_size=1))
            vel_publishers.append(rospy.Publisher('/visualization/velocity_' + str(k+1), Float32, queue_size=1))
            acc_publishers.append(rospy.Publisher('/visualization/acceleration_' + str(k+1), Float32, queue_size=1))
            traveled_dist_publishers.append(rospy.Publisher('/visualization/traveled_dist_' + str(k+1), Float32, queue_size=1))


        # #} end of INITALIZATION OF PUBLISHERS

        # #{ INITIALIZATION OF RVIZ VISUALIZER

        self.visualizer_ = Visualizer(self.inspection_problem, jsk_diagnostics_publishers, cones_publishers,\
                                      pub_inspection_points, pub_viewpoints, pub_collisions, pub_start_positions, pub_start_arrows, pub_safety_area, pub_cloud,\
                                      path_publishers, odometry_publishers, horizon_publishers, obst_dist_publishers, mutual_dist_publishers,\
                                      vel_publishers, acc_publishers, traveled_dist_publishers, pub_mission_time, pub_remaining_time, pub_score,\
                                      pub_playback_status, pub_diagnostic, pub_solution_time, pub_solution_penalty, pub_fullscreen, global_frame)
        self.visualizer_.setFov1(fov_length_1, horizontal_aov_1, vertical_aov_2, 0.05, 0.0, 0.0, 1.0)
        self.visualizer_.setFov2(fov_length_2, horizontal_aov_2, vertical_aov_2, 0.05, 1.0, 0.0, 0.0)

        rospy.Timer(rospy.Duration(1), self.diagnosticsMsgCallback)
        self.diag_msg_lock = RLock()
        self.diag_msgs = []

        if visualization_rviz:
            tmp_rviz_config = "/tmp/mrim_manager.rviz"
            self.visualizer_.setRvizConfig(rviz_config, tmp_rviz_config, self.evaluator_.inspection_problem.number_of_inspection_points,\
                                           self.mission_time_limit, self.solution_time_constraint_soft, self.solution_time_constraint_hard,\
                                           uav_names[0], uav_names[1])
            rviz_proc = subprocess.Popen(['rviz', '-d', tmp_rviz_config], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.Rate(0.2).sleep()

        # #} end of INITIALIZATION OF RVIZ VISUALIZER

        # #{ TRAJECTORIES SUBSCRIPTION OR LOADING

        if self.subscribe_trajectories:
            while not (self.trajectory1_subscribed and self.trajectory2_subscribed):
                rospy.loginfo_throttle(10.0, "[MrimManager] Waiting for trajectories. Trajectory 1 subscribed = %d, trajectory 2 subscribed = %d.", self.trajectory1_subscribed, self.trajectory2_subscribed)
                solution_time = (rospy.Time.now() - self.solution_time_start).to_sec()
                solution_time_penalty = max(0.0, solution_time - self.solution_time_constraint_soft)
                self.visualizer_.publishSolutionTime(self.solution_time_constraint_hard - solution_time, solution_time_penalty)

                if solution_time > self.solution_time_constraint_hard:
                    self.visualizer_.publishFullScreenMsg("Maximum solution time exceeded.\n Takeoff not allowed.")
                    rospy.logerr("Maximum solution time exceeded.")
                    rospy.logerr("Killing mrim planner rosnode.")
                    os.system("rosnode kill /mrim_planner") #future fix: it disables publishing but the planning is still running
                    return

                rospy.Rate(5).sleep()

            rospy.loginfo("[MrimManager] Trajectories obtained. Solution time = %.2f.", (self.solution_time_end - self.solution_time_start).to_sec())

            self.trajectories.append(Trajectory(self.trajectory1, trajectory_dt, "UAV RED"))
            self.trajectories.append(Trajectory(self.trajectory2, trajectory_dt, "UAV BLUE"))

        else:
            self.trajectories = self.loadTrajectories(trajectory_folder, trajectory_files, trajectory_dt)

            if len(self.trajectories) == len(trajectory_files):
                rospy.loginfo("[MrimManager] All trajectories loaded successfully. Number of trajectories = %lu.", len(self.trajectories))
            else:
                rospy.logwarn("[MrimManager] Only %lu out of %lu were loaded.", len(self.trajectories), len(trajectory_files))

            if len(self.trajectories) == 0:
                rospy.logwarn("[MrimManager] No valid trajectory loaded. Nothing to check.")
                return

        self.visualizer_.setTrajectories(self.trajectories)

        idx = 1
        rospy.loginfo("[MrimManager] Trajectory indices (used in RVIZ visualization):")
        for trajectory in self.trajectories:
            rospy.loginfo("[MrimManager]  - Trajectory %d: %s", idx, trajectory.trajectory_name)
            idx += 1

        # #} end of TRAJECTORIES SUBSCRIPTION OR LOADING

        # #{ TRAJECTORIES CHECKING

        mutual_distances = self.getMutualDistances(self.trajectories)

        collisions_between_uavs = self.getCollisionsBetweenTrajectories(self.trajectories, minimum_mutual_distance)

        mutual_dists_ok_list = self.checkMutualDistances(mutual_distances, minimum_mutual_distance)

        uav_obstacle_distances = self.getUavObstacleDistances(self.trajectories)

        uav_obstacle_dists_ok_list = self.checkUavObstacleDistances(uav_obstacle_distances, minimum_obstacle_distance)

        dynamic_constraints_ok_list = self.checkDynamicConstraints(self.trajectories, self.constraints)

        safety_area_ok_list = self.checkSafetyArea(self.trajectories, False, minimum_height, maximum_height)

        overall_status = True

        for k in range(len(self.trajectories)):
            # rospy.loginfo("[MrimManager] ---------- Trajectory %s check list: ----------", self.trajectories[k].trajectory_name)
            trajectory_status = mutual_dists_ok_list[k] and dynamic_constraints_ok_list[k] and uav_obstacle_dists_ok_list[k] and safety_area_ok_list[k]
            # rospy.loginfo("[MrimManager] Overall status: %s (mutual distances: %s, dynamic constraints: %s, obstacle distance: %s, safety area: %s)", boolToString(trajectory_status), boolToString(mutual_dists_ok_list[k]), boolToString(dynamic_constraints_ok_list[k]), boolToString(uav_obstacle_dists_ok_list[min(k, len(uav_obstacle_dists_ok_list) -1)]), boolToString(safety_area_ok_list[k]))
            overall_status = overall_status and trajectory_status

            self.trajectories[k].setStatistics(min(uav_obstacle_distances[k]), min(mutual_distances[k])[0], dynamic_constraints_ok_list[k], trajectory_status)

        if not self.pcl_map.initialized:
            overall_status = False
            rospy.logerr("[MrimManager] UAV-to-obstacles distances were not checked due to missing input point cloud.")

        if overall_status:
            rospy.loginfo("[MrimManager] TRAJECTORY CHECKS: {:s}".format(boolToString(overall_status)))
        else:
            rospy.logerr("[MrimManager] TRAJECTORY CHECKS: {:s}".format(boolToString(overall_status)))

        # start overall status publishing
        self.overall_status = not run_type == 'uav' and (flight_always_allowed or overall_status)
        rospy.Timer(rospy.Duration(1), self.publishOverallStatus)

        # if overall_status:
            # rospy.loginfo("[MrimManager] ---------- Overall status: OK, all checks passed. ----------")
        # else:
            # rospy.logerr("[MrimManager] ---------- Overall status: FAILED, some checks failed, see script's output for details. ----------")


        # #} end of TRAJECTORIES CHECKING

        # #{ PYTHON VISUALIZATIONS

        py_plotter = PythonPlotter()

        if visualization_trajectory:
            py_plotter.plotPaths(self.trajectories)

        if visualization_mutual_dist:
            py_plotter.plotMutualDistances(self.trajectories, mutual_distances, trajectory_dt, minimum_mutual_distance)

        if visualization_obstacle_dist and self.pcl_map.initialized:
            py_plotter.plotUavObstacleDistances(self.trajectories, uav_obstacle_distances, trajectory_dt, minimum_obstacle_distance)

        if visualization_dynamics:
            py_plotter.plotDynamics(self.trajectories, self.constraints, trajectory_dt)

        # #} end of PYTHON VISUALIZATIONS

        # #{ RVIZ TRAJECTORIES AND STATISTICS VISUALIZATION

        rate = rospy.Rate(0.2)
        playback_speed = playback_speed if visualization_rviz else 1000 # avoid waiting for score if rViz not used 

        if run_type == 'offline':
            while not rospy.is_shutdown():
                self.visualizer_.publishObstacles()
                self.visualizer_.publishSafetyArea()
                self.visualizer_.publishPaths(self.trajectories)
                self.visualizer_.publishCollisions(self.trajectories, collisions_between_uavs)
                self.runOfflineTrajectoryPlayback(self.trajectories, odometry_publishers, playback_speed, trajectory_dt, uav_obstacle_distances,\
                                           mutual_distances, minimum_obstacle_distance, minimum_mutual_distance)
                self.evaluator_.resetScore()
                rate.sleep()

                if not visualization_rviz:
                    rospy.loginfo('[MrimManager] Killing ros nodes')
                    os.system("pkill roslaunch")
                    break

        elif run_type == 'simulation' or run_type == 'uav':

            if not overall_status:
                for k in range(len(self.trajectories)):
                    jsk_msg = self.visualizer_.generateJskMsg(self.trajectories[k].trajectory_name, self.trajectories[k].length, self.trajectories[k].time, \
                                                              self.trajectories[k].min_obst_dist.item(), self.trajectories[k].min_mutual_dist,\
                                                              self.trajectories[k].dynamics_ok)
                    self.visualizer_.publishJskMsg(jsk_msg, self.trajectories[k].overall_status, k)

                self.visualizer_.publishFullScreenMsg("------ Constraints violated! ------\n ------ Takeoff not allowed! ------")
                rate.sleep()

            if overall_status or (not run_type == 'uav' and flight_always_allowed):
                self.visualizer_.publishFullScreenMsg("")
                self.runSimulationMonitoring(self.trajectories, minimum_obstacle_distance, minimum_mutual_distance, dynamic_constraints_ok_list)
        else:
            rospy.logwarn('[MrimManager] Unexpected run type: %s', run_type)

        # #} end of RVIZ TRAJECTORIES AND STATISTICS VISUALIZATION

    # #} end of __init__()

    # #{ loadTrajectories()

    def loadTrajectories(self, folder, filenames, dt):

        trajectories = []
        for filename in filenames:
            if os.path.exists(os.path.join(folder, filename)):
                trajectories.append(self.loadTrajectory(os.path.join(folder, filename), filename.rpartition('.')[0], dt))
            else:
                rospy.logerr('[MrimManager] Trajectory file %s not found. Excluding file from checking.', os.path.join(folder, filename))

        return trajectories

    # #} end of loadTrajectories()

    # #{ loadTrajectory()

    def loadTrajectory(self, filepath, trajectory_name, dt):
        f = open(filepath, 'r')
        lines = f.readlines()
        waypoints = []
        ext = filepath.split(".")[-1]
        ext = filepath.split(".")[-1]
        for line in lines:
            x, y, z, heading = line.split(',' if ',' in line else ' ')
            waypoints.append(Vector4d(float(x), float(y), float(z), float(heading)))

        return Trajectory(waypoints, dt, trajectory_name)

    # #} end of loadTrajectory()

    # #{ TRAJECTORY CHECKING

    # #{ checkDynamicConstraints()

    def checkDynamicConstraints(self, trajectories, constraints):

        results = []

        for k in range(len(trajectories)):
            vels_xy = []
            accs_xy = []
            jerks_xy = []
            snaps_xy = []
            for m in range(len(trajectories[k].velocities)):
                vels_xy.append(np.sqrt(trajectories[k].velocities[m].x**2 + trajectories[k].velocities[m].y**2))
                accs_xy.append(np.sqrt(trajectories[k].accelerations[m].x**2 + trajectories[k].accelerations[m].y**2))
                jerks_xy.append(np.sqrt(trajectories[k].jerks[m].x**2 + trajectories[k].jerks[m].y**2))
                snaps_xy.append(np.sqrt(trajectories[k].snaps[m].x**2 + trajectories[k].snaps[m].y**2))

            vels_x = [vel.x for vel in trajectories[k].velocities]
            vels_y = [vel.y for vel in trajectories[k].velocities]
            vels_z = [vel.z for vel in trajectories[k].velocities]
            accs_x = [acc.x for acc in trajectories[k].accelerations]
            accs_y = [acc.y for acc in trajectories[k].accelerations]
            accs_z = [acc.z for acc in trajectories[k].accelerations]
            jerks_z = [jerk.z for jerk in trajectories[k].jerks]
            snaps_z = [snap.z for snap in trajectories[k].snaps]
            vels_heading = [vel.heading for vel in trajectories[k].velocities]
            accs_heading = [acc.heading for acc in trajectories[k].accelerations]
            jerks_heading = [jerk.heading for jerk in trajectories[k].jerks]
            snaps_heading = [snap.heading for snap in trajectories[k].snaps]

            # #{ LIMITING VALUES

            max_vel_xy = max(abs(np.array(vels_xy)))
            max_vel_x = max(abs(np.array(vels_x)))
            max_vel_y = max(abs(np.array(vels_y)))
            max_vel_desc = abs(min(np.array(vels_z)))
            max_vel_asc = abs(max(np.array(vels_z)))
            max_vel_heading = max(abs(np.array(vels_heading)))

            max_acc_xy = max(abs(np.array(accs_xy)))
            max_acc_x = max(abs(np.array(accs_x)))
            max_acc_y = max(abs(np.array(accs_y)))
            max_acc_desc = abs(min(np.array(accs_z)))
            max_acc_asc = abs(max(np.array(accs_z)))
            max_acc_heading = max(abs(np.array(accs_heading)))

            max_jerk_xy = max(abs(np.array(jerks_xy)))
            max_jerk_desc = abs(min(np.array(jerks_z)))
            max_jerk_asc = abs(max(np.array(jerks_z)))
            max_jerk_heading = max(abs(np.array(jerks_heading)))

            max_snap_xy = max(abs(np.array(snaps_xy)))
            max_snap_desc = abs(min(np.array(snaps_z)))
            max_snap_asc = abs(max(np.array(snaps_z)))
            max_snap_heading = max(abs(np.array(snaps_heading)))

            # #} end of LIMITING VALUES

            # #{ CONSTRAINTS CHECK

            # check constraints
            vel_xy_ok = max_vel_xy < constraints.horizontal.speed + self.constraints_violation_tolerance
            vel_x_ok = max_vel_x < constraints.horizontal.speed + self.constraints_violation_tolerance
            vel_y_ok = max_vel_y < constraints.horizontal.speed + self.constraints_violation_tolerance
            vel_desc_ok = max_vel_desc < constraints.descending.speed + self.constraints_violation_tolerance
            vel_asc_ok = max_vel_asc < constraints.ascending.speed + self.constraints_violation_tolerance
            vel_heading_ok = max_vel_heading < constraints.heading.speed + self.constraints_violation_tolerance

            acc_xy_ok = max_acc_xy < constraints.horizontal.acceleration + self.constraints_violation_tolerance
            acc_x_ok = max_acc_x < constraints.horizontal.acceleration + self.constraints_violation_tolerance
            acc_y_ok = max_acc_y < constraints.horizontal.acceleration + self.constraints_violation_tolerance
            acc_desc_ok = max_acc_desc < constraints.descending.acceleration + self.constraints_violation_tolerance
            acc_asc_ok = max_acc_asc < constraints.ascending.acceleration + self.constraints_violation_tolerance
            acc_heading_ok = max_acc_heading < constraints.heading.acceleration + self.constraints_violation_tolerance

            jerk_xy_ok = max_jerk_xy < constraints.horizontal.jerk
            jerk_desc_ok = max_jerk_desc < constraints.descending.jerk
            jerk_asc_ok = max_jerk_asc < constraints.ascending.jerk
            jerk_heading_ok = max_jerk_heading < constraints.heading.jerk

            snap_xy_ok = max_snap_xy < constraints.horizontal.snap
            snap_desc_ok = max_snap_desc < constraints.descending.snap
            snap_asc_ok = max_snap_asc < constraints.ascending.snap
            snap_heading_ok = max_snap_heading < constraints.heading.snap

            # #} end of CONSTRAINTS CHECK

            # #{ COMMAND LINE OUTPUTS

            ok_vel = vel_x_ok and vel_y_ok and vel_desc_ok and vel_asc_ok and vel_heading_ok
            ok_acc = acc_x_ok and acc_y_ok and acc_desc_ok and acc_asc_ok and acc_heading_ok
            ok_jerk = jerk_xy_ok and jerk_desc_ok and jerk_asc_ok and jerk_heading_ok
            ok_snap = snap_xy_ok and snap_desc_ok and snap_asc_ok and snap_heading_ok
            ok = ok_vel and ok_acc

            rospy.loginfo("[MrimManager] [{:s}] Dynamic constraints of: {:s}:".format(boolToString(ok), self.trajectories[k].trajectory_name))
            rospy.loginfo("[MrimManager]    speed: [{:s}]".format(boolToString(ok_vel)))
            rospy.loginfo("[MrimManager]      - [{:s}] horizontal x: {:.2f} (max: {:.2f}) m/s".format(boolToString(vel_x_ok), max_vel_x, constraints.horizontal.speed))
            rospy.loginfo("[MrimManager]      - [{:s}] horizontal y: {:.2f} (max: {:.2f}) m/s".format(boolToString(vel_y_ok), max_vel_y, constraints.horizontal.speed))
            rospy.loginfo("[MrimManager]      - [{:s}] descending: {:.2f} (max: {:.2f}) m/s".format(boolToString(vel_desc_ok), max_vel_desc, constraints.descending.speed))
            rospy.loginfo("[MrimManager]      - [{:s}] ascending:  {:.2f} (max: {:.2f}) m/s".format(boolToString(vel_asc_ok), max_vel_asc, constraints.ascending.speed))
            rospy.loginfo("[MrimManager]      - [{:s}] heading:    {:.2f} (max: {:.2f}) rad/s{:s}".format(boolToString(vel_heading_ok), max_vel_heading, constraints.heading.speed, '' if vel_heading_ok else ' => the task will run but precise control of the heading is not assured'))
            rospy.loginfo("[MrimManager]    acceleration: [{:s}]".format(boolToString(ok_acc)))
            rospy.loginfo("[MrimManager]      - [{:s}] horizontal x: {:.2f} (max: {:.2f}) m/s^2".format(boolToString(acc_x_ok), max_acc_x, constraints.horizontal.acceleration))
            rospy.loginfo("[MrimManager]      - [{:s}] horizontal y: {:.2f} (max: {:.2f}) m/s^2".format(boolToString(acc_y_ok), max_acc_y, constraints.horizontal.acceleration))
            rospy.loginfo("[MrimManager]      - [{:s}] descending: {:.2f} (max: {:.2f}) m/s^2".format(boolToString(acc_desc_ok), max_acc_desc, constraints.descending.acceleration))
            rospy.loginfo("[MrimManager]      - [{:s}] ascending:  {:.2f} (max: {:.2f}) m/s^2".format(boolToString(acc_asc_ok), max_acc_asc, constraints.ascending.acceleration))
            rospy.loginfo("[MrimManager]      - [{:s}] heading:    {:.2f} (max: {:.2f}) rad/s^2{:s}".format(boolToString(acc_heading_ok), max_acc_heading, constraints.heading.acceleration, '' if acc_heading_ok else ' => the task will run but precise control of the heading is not assured'))
            # rospy.loginfo("[MrimManager]    jerk: [{:s}]".format(boolToString(ok_jerk)))
            # rospy.loginfo("[MrimManager]      - [{:s}] horizontal: {:.2f} (max: {:.2f}) m/s^3".format(boolToString(jerk_xy_ok), max_jerk_xy, constraints.horizontal.jerk))
            # rospy.loginfo("[MrimManager]      - [{:s}] descending: {:.2f} (max: {:.2f}) m/s^3".format(boolToString(jerk_desc_ok), max_jerk_desc, constraints.descending.jerk))
            # rospy.loginfo("[MrimManager]      - [{:s}] ascending:  {:.2f} (max: {:.2f}) m/s^3".format(boolToString(jerk_asc_ok), max_jerk_asc, constraints.ascending.jerk))
            # rospy.loginfo("[MrimManager]      - [{:s}] heading:    {:.2f} (max: {:.2f}) rad/s^3".format(boolToString(jerk_heading_ok), max_jerk_heading, constraints.heading.jerk))
            # rospy.loginfo("[MrimManager]    snap: [{:s}]".format(boolToString(ok_snap)))
            # rospy.loginfo("[MrimManager]      - [{:s}] horizontal: {:.2f} (max: {:.2f}) m/s^4".format(boolToString(snap_xy_ok), max_snap_xy, constraints.horizontal.snap))
            # rospy.loginfo("[MrimManager]      - [{:s}] descending: {:.2f} (max: {:.2f}) m/s^4".format(boolToString(snap_desc_ok), max_snap_desc, constraints.descending.snap))
            # rospy.loginfo("[MrimManager]      - [{:s}] ascending:  {:.2f} (max: {:.2f}) m/s^4".format(boolToString(snap_asc_ok), max_snap_asc, constraints.ascending.snap))
            # rospy.loginfo("[MrimManager]      - [{:s}] heading:    {:.2f} (max: {:.2f}) rad/s^4".format(boolToString(snap_heading_ok), max_snap_heading, constraints.heading.snap))

            # #} end of COMMAN LINE OUTPUTS

            # constraints_check_successful = vel_xy_ok and vel_asc_ok and vel_desc_ok and vel_heading_ok and acc_xy_ok and acc_asc_ok and acc_desc_ok and acc_heading_ok and jerk_xy_ok and jerk_asc_ok and jerk_desc_ok and jerk_heading_ok and snap_xy_ok and snap_asc_ok and snap_desc_ok and snap_heading_ok
            translation_constraints_check_successful = vel_x_ok and vel_y_ok and vel_asc_ok and vel_desc_ok and  acc_x_ok and acc_y_ok and acc_asc_ok and acc_desc_ok 
            constraints_check_successful = translation_constraints_check_successful and vel_heading_ok and acc_heading_ok

            if not translation_constraints_check_successful:
                self.evaluator_.setZeroScore()

            # if constraints_check_successful:
            #     rospy.loginfo("[MrimManager] ##### Constraints for trajectory %s not violated. #####", trajectories[k].trajectory_name)
            # else:
            #     rospy.logerr("[MrimManager] ##### FAILED: Constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)

            if self.print_info:
                rospy.loginfo("[MrimManager] --------- Dynamics of trajectory %s: ----------", trajectories[k].trajectory_name)
                rospy.loginfo("[MrimManager] Max speed: horizontal = %.2f (m/s), descending = %.2f (m/s), ascending = %.2f (m/s), heading = %.2f (rad/s)", max_vel_xy, max_vel_desc, max_vel_asc, max_vel_heading)
                rospy.loginfo("[MrimManager] Max acceleration: horizontal = %.2f (m/s^2), descending = %.2f (m/s^2), ascending = %.2f (m/s^2), heading = %.2f (rad/s^2)", max_acc_xy, max_acc_desc, max_acc_asc, max_acc_heading)

            results.append(constraints_check_successful)

        return results

    # #} end of checkDynamicConstraints()

    # #{ checkSafetyArea()

    def checkSafetyArea(self, trajectories, safety_area, minimum_height, maximum_height):
        results = []
        ok_min, ok_max = True, True
        mins, maxs = [], []

        # TODO future fix: add safety area check based on the safety area from inspection problem
        for k in range(len(trajectories)):

            # rospy.loginfo("[MrimManager] ---------- Min and max height check for trajectory %s: ----------", trajectories[k].trajectory_name)

            z_list = [pose.z for pose in trajectories[k].poses]

            z_min = min(np.array(z_list))
            z_max = max(np.array(z_list))

            mins.append(z_min)
            maxs.append(z_max)

            result = True

            if z_min > minimum_height:
                pass
                # rospy.loginfo("[MrimManager] Minimum height constraints for trajectory %s not violated.", trajectories[k].trajectory_name)
            else:
                # rospy.logerr("[MrimManager] ##### FAILED: Minimum height constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)
                ok_min = False
                result = False

            if z_max < maximum_height:
                pass
                # rospy.loginfo("[MrimManager] Maximum height constraints for trajectory %s not violated.", trajectories[k].trajectory_name)
            else:
                # rospy.logerr("[MrimManager] ##### FAILED: Maximum height constraints for trajectory %s violated. #####", trajectories[k].trajectory_name)
                ok_max = False
                result = False

            results.append(result)

            if self.print_info:
                rospy.loginfo("[MrimManager] Height limits of trajectory %s:", trajectories[k].trajectory_name)
                rospy.loginfo("[MrimManager] Min height = %.2f m, max height = %.2f m", z_min, z_max)

        rospy.loginfo("[MrimManager] [{:s}] Min/max height:".format(boolToString(ok_min and ok_max)))
        for k in range(len(trajectories)):
            ok = results[k]
            rospy.loginfo("[MrimManager]    - [{:s}] {:s}: min|max = {:.2f}|{:.2f} (limit: {:.2f}|{:.2f}) m".format(boolToString(ok), self.trajectories[k].trajectory_name, mins[k], maxs[k], minimum_height, maximum_height))

        return results

    # #} end of checkSafetyArea()

    # #{ checkMutualDistances()

    def checkMutualDistances(self, min_dists_list, min_dist_allowed):

        result = []

        if min_dists_list == []:
            result.append(True)
            return result

        ok, min_dist = True, float('inf')

        for k in range(len(min_dists_list)):

            # rospy.loginfo("[MrimManager] ---------- Mutual distance check for trajectory %s: ----------", self.trajectories[k].trajectory_name)

            min_d = min(min_dists_list[k], key = lambda t: t[0])

            # rospy.loginfo("[MrimManager] Minimum distance of trajectory %s from other trajectories is %.2f (closest trajectory: %s).", self.trajectories[k].trajectory_name, min_d[0], self.trajectories[min_d[1]].trajectory_name)
            if min_d[0] < min_dist:
                min_dist = min_d[0]

            if (min_d[0] < min_dist_allowed):
                ok = False
                # rospy.logerr("[MrimManager] FAILED: Minimum distance of trajectory %s is below specified threshold (%.2f).", self.trajectories[k].trajectory_name, min_dist_allowed)
                result.append(False)
            else:
                result.append(True)

        rospy.loginfo("[MrimManager] [{:s}] Minimum UAV-to-UAV distance: {:.2f} m".format(boolToString(ok), min_dist))

        return result

    # #} end of checkMutualDistances()

    # #{ checkUavObstacleDistances()

    def checkUavObstacleDistances(self, dists_list, min_dist_allowed):

        result = []

        if dists_list == []:
            result.append(True)
            return result

        ok    = True
        dists = []

        for k in range(len(dists_list)):

            # rospy.loginfo("[MrimManager] ---------- UAV-obstacle distance check for trajectory %s: ----------", self.trajectories[k].trajectory_name)

            min_d = min(dists_list[k])
            dists.append(float(min_d[0]))

            # rospy.loginfo("[MrimManager] Minimum distance of trajectory %s from given pointcloud is %.2f.", self.trajectories[k].trajectory_name, min_d)

            if (min_d[0] < min_dist_allowed):
                ok = False
                # rospy.logerr("[MrimManager] FAILED: Minimum UAV-obstacle distance of trajectory %s is below specified threshold (%.2f).", self.trajectories[k].trajectory_name, min_dist_allowed)
                result.append(False)
            else:
                result.append(True)

        rospy.loginfo("[MrimManager] [{:s}] Minimum UAV-to-obstacle distance:".format(boolToString(ok)))
        for k in range(len(dists_list)):
            rospy.loginfo("[MrimManager]    - [{:s}] {:s}: {:.2f} m".format(boolToString(ok), self.trajectories[k].trajectory_name, dists[k]))

        return result

    # #} end of checkUavObstacleDistances()

    # #{ getMutualDistances()

    def getMutualDistances(self, trajectories):

        min_dists_list = []

        if len(trajectories) < 2:
            return min_dists_list

        for t in range(len(trajectories)):
            min_dists = []
            for k in range(len(trajectories[t].poses)):
                min_dist = 1e6
                min_idx = -1
                for t_r in range(len(trajectories)):
                    if t == t_r:
                        continue

                    idx = min(k, len(trajectories[t_r].poses) - 1)
                    dist = getTransitionPointDist(trajectories[t].poses[k], trajectories[t_r].poses[idx])
                    if dist < min_dist:
                        min_dist = dist
                        min_idx = t_r

                min_dists.append((min_dist, min_idx))

            min_dists_list.append(min_dists)

        return min_dists_list

    # #} end of getMutualDistances()

    # #{ getCollisionsBetweenTrajectories()

    def getCollisionsBetweenTrajectories(self, trajectories, min_dist):

        if len(trajectories) < 2:
            return {}

        collisions = {}

        min_len = min([len(t.poses) for t in trajectories])
        for t in range(len(trajectories)):
            for t_r in range(len(trajectories)):
                if t == t_r:
                    continue
                key = (t, t_r)
                if key not in collisions:
                    collisions[key] = []

                for i in range(min_len):
                    dist = getTransitionPointDist(trajectories[t].poses[i], trajectories[t_r].poses[i])
                    if dist < min_dist:
                        collisions[key].append(i)

        return collisions

    # #} end of getCollisionsBetweenTrajectories()

    # #{ getUavObstacleDistances()

    def getUavObstacleDistances(self, trajectories):

        dists_list = []

        if not self.pcl_map.initialized:
            rospy.logwarn("[MrimManager] PCL map not initialized. Cannot get UAV-obstacle distance")
            return dists_list

        for t in range(len(trajectories)):
            dists = []
            for pose in trajectories[t].poses:
                dists.append(self.pcl_map.getMinDist(pose))

            dists_list.append(dists)

        return dists_list

    # #} end of getUavObstacleDistances()


    # #} end of TRAJECTORIES CHECKING

    # #{ runOfflineTrajectoryPlayback()

    def runOfflineTrajectoryPlayback(self, trajectories, odometry_publishers, playback_speed, dt, obstacle_dists, mutual_dists, min_obst_dist, min_mutual_dist):
        rospy.loginfo_once("[MrimManager] Running trajectory playback.")
        playback_rate = rospy.Rate(playback_speed/dt)
        max_len = max(np.array([len(trajectory.poses) for trajectory in trajectories]))
        trajectory_idx = 0

        self.visualizer_.publishObstacles()
        self.visualizer_.publishSafetyArea()
        self.visualizer_.publishStartPositions()
        self.visualizer_.publishPlaybackStatus(self.playback_paused)

        solution_time, solution_time_penalty = self.checkMaximumSolutionTime()
        self.visualizer_.publishSolutionTime(self.solution_time_constraint_hard - solution_time, solution_time_penalty)

        while trajectory_idx < max_len:
            if not self.playback_paused:
                self.visualizer_.publishFullScreenMsg("")
                self.publishPlaybackOffline(trajectories, trajectory_idx, obstacle_dists, mutual_dists, min_obst_dist, min_mutual_dist)
                trajectory_idx += 1

            playback_rate.sleep()

        if not self.evaluator_.checkFinalPositions([trajectories[0].poses[-1], trajectories[1].poses[-1]]):
            with self.diag_msg_lock:
                self.diag_msgs.append("UAVs are not at their starting positions!")
            self.evaluator_.setZeroScore()

        rospy.loginfo('[MrimManager] FINAL SCORE: %d/%d, FINAL TIME: %.3f (trajectories: %.3f, solution penalty: %.3f)', self.evaluator_.getScore(), len(self.inspection_problem.inspection_points), (trajectory_idx - 1)*dt + solution_time_penalty, (trajectory_idx - 1)*dt, solution_time_penalty)
        self.visualizer_.publishFullScreenMsg("FINAL SCORE: {:d}/{:d}, FINAL TIME: {:.3f}".format(self.evaluator_.getScore(), len(self.inspection_problem.inspection_points), (trajectory_idx - 1)*dt + solution_time_penalty))
        self.playback_paused = True

    # #} end of runOfflineTrajectoryPlayback()

    # #{ runSimulationMonitoring()

    def runSimulationMonitoring(self, trajectories, minimum_obstacle_distance, minimum_mutual_distance, dynamic_constraints_ok_list):
        rospy.loginfo_once("[MrimManager] Running simulation monitoring.")

        self.visualizer_.publishObstacles()
        self.visualizer_.publishSafetyArea()
        self.visualizer_.publishStartPositions()
        self.visualizer_.publishPaths(trajectories)
        self.visualizer_.publishFullScreenMsg("")

        solution_time, solution_time_penalty = self.checkMaximumSolutionTime()
        self.visualizer_.publishSolutionTime(self.solution_time_constraint_hard - solution_time, solution_time_penalty)

        with self.uav_states_lock:
            self.task_monitor = TaskMonitor(trajectories, self.pcl_map, self.uav_states, minimum_obstacle_distance, minimum_mutual_distance, dynamic_constraints_ok_list)

        # init subscriber to have goal
        srv_start_monitoring = rospy.Service('start_monitoring_in', Trigger, self.startMonitoringCallback)

        while not self.mission_finished:
            if self.mission_started:
                with self.uav_states_lock:
                    mission_time, travelled_dists, obst_dists, mutual_dists, min_obst_dists, min_mutual_dists,\
                        velocities, accelerations, overall_statuses, poses = self.task_monitor.update(self.uav_states)

                self.publishPlaybackSimulation(trajectories, poses, obst_dists, mutual_dists, min_obst_dists,\
                                               min_mutual_dists, velocities, accelerations, travelled_dists, overall_statuses,\
                                               mission_time, minimum_obstacle_distance, minimum_mutual_distance)

            rospy.Rate(10).sleep() #TODO: tune to avoid missing inspection point due to too fast flight through the inspection point

        poses = []
        with self.uav_states_lock:
            for uav_state in self.uav_states:
                poses.append(uavStateMsgToTrajectoryPoint(uav_state))

        if not self.evaluator_.checkFinalPositions(poses):
            with self.diag_msg_lock:
                self.diag_msgs.append("UAVs are not at their starting positions!")
            self.evaluator_.setZeroScore()

        self.visualizer_.publishMissionStatistics(self.task_monitor.getFinalTime(), self.evaluator_.getScore(), self.mission_time_limit)
        self.visualizer_.publishFullScreenMsg("FINAL SCORE: {:d}, FINAL TIME: {:.3f}".format(self.evaluator_.getScore(), self.task_monitor.getFinalTime() + solution_time_penalty))
        rospy.loginfo('[MrimManager] FINAL SCORE: %d, FINAL TIME: %.3f (trajectories: %.3f, solution penalty: %.3f)', self.evaluator_.getScore(),
                      self.task_monitor.getFinalTime() + solution_time_penalty, self.task_monitor.getFinalTime(), solution_time_penalty)

    # #} end of runSimulationMonitoring()

    # #{ publishPlaybackOffline()

    def publishPlaybackOffline(self, trajectories, trajectory_idx, obstacle_dists, mutual_dists, min_obst_dist, min_mutual_dist):

        poses = []
        for k in range(len(trajectories)):
            t_idx = min(trajectory_idx, len(trajectories[k].poses) - 1)
            self.visualizer_.publishCone(trajectoryPointToPoseMsg(trajectories[k].poses[t_idx]), k)
            self.visualizer_.publishOdometry(trajectoryToOdometryMsg(trajectories[k], trajectory_idx, self.visualizer_.frame), k)
            self.visualizer_.publishUavStatistics(k, obstacle_dists[k][t_idx].item(), mutual_dists[k][t_idx][0], trajectories[k].abs_velocities[t_idx],\
                                               trajectories[k].abs_accelerations[t_idx], trajectories[k].cummulative_length[t_idx])
            jsk_msg = self.visualizer_.generateJskMsg(trajectories[k].trajectory_name, trajectories[k].length, trajectories[k].time, \
                                                       trajectories[k].min_obst_dist.item(), trajectories[k].min_mutual_dist, trajectories[k].dynamics_ok)
            self.visualizer_.publishJskMsg(jsk_msg, trajectories[k].overall_status, k)
            poses.append(trajectories[k].poses[t_idx])

            if obstacle_dists[k][t_idx].item() < min_obst_dist:
                self.evaluator_.setZeroScore()
                with self.diag_msg_lock:
                    msg = "Minimum obstacle distance violated!"
                    if not msg in self.diag_msgs:
                        self.diag_msgs.append(msg)

            if mutual_dists[k][t_idx][0] < min_mutual_dist:
                self.evaluator_.setZeroScore()
                with self.diag_msg_lock:
                    msg = "Minimum mutual distance violated!"
                    if not msg in self.diag_msgs:
                        self.diag_msgs.append(msg)

        if not self.mission_time_exceeded and trajectory_idx*trajectories[0].dt > self.mission_time_limit:
            with self.diag_msg_lock:
                self.diag_msgs.append("-- Mission time limit exceeded! --")
            self.mission_time_exceeded = True
            self.evaluator_.setZeroScore()

        self.evaluator_.updateInspectionStatus(poses[0], poses[-1]) # future fix: currently expect two trajectories only
        self.visualizer_.publishHorizon(self.trajectories, trajectory_idx, self.visualization_horizon_length)
        self.visualizer_.publishInspectionPoints(self.evaluator_.inspection_problem.inspection_points, self.evaluator_.viewpoints)
        self.visualizer_.publishViewPoints(self.evaluator_.inspection_problem.inspection_points, self.evaluator_.viewpoints)
        self.visualizer_.publishMissionStatistics(trajectory_idx*trajectories[0].dt, self.evaluator_.getScore(), self.mission_time_limit)

    # #} end of publishPlaybackOffline()

    # #{ publishPlaybackSimulation()

    def publishPlaybackSimulation(self, trajectories, poses, obstacle_dists, mutual_dists, min_obst_dists, min_mutual_dists, velocities, accelerations, travelled_dists, overall_statuses, mission_time, obstacle_dist_limit, mutual_dist_limit):

        for k in range(len(trajectories)):
            self.visualizer_.publishCone(trajectoryPointToPoseMsg(poses[k]), k)
            self.visualizer_.publishUavStatistics(k, obstacle_dists[k], mutual_dists[k], velocities[k], accelerations[k], travelled_dists[k])
            jsk_msg = self.visualizer_.generateJskMsg(trajectories[k].trajectory_name, trajectories[k].length, trajectories[k].time, \
                                                       min_obst_dists[k], min_mutual_dists[k], trajectories[k].dynamics_ok)
            self.visualizer_.publishJskMsg(jsk_msg, overall_statuses[k], k)

            if min_obst_dists[k] < obstacle_dist_limit:
                self.evaluator_.setZeroScore()
                with self.diag_msg_lock:
                    msg = "Minimum obstacle distance violated!"
                    if not msg in self.diag_msgs:
                        self.diag_msgs.append(msg)

            if min_mutual_dists[k] < mutual_dist_limit:
                self.evaluator_.setZeroScore()
                with self.diag_msg_lock:
                    msg = "Minimum mutual distance violated!"
                    if not msg in self.diag_msgs:
                        self.diag_msgs.append(msg)

        if not self.mission_time_exceeded and mission_time > self.mission_time_limit:
            with self.diag_msg_lock:
                self.diag_msgs.append("-- Mission time limit exceeded! --")
            self.mission_time_exceeded = True
            self.evaluator_.setZeroScore()

        self.evaluator_.updateInspectionStatus(poses[0], poses[-1]) # future fix: currently expect two trajectories only
        self.visualizer_.publishInspectionPoints(self.evaluator_.inspection_problem.inspection_points, self.evaluator_.viewpoints)
        self.visualizer_.publishViewPoints(self.evaluator_.inspection_problem.inspection_points, self.evaluator_.viewpoints)
        self.visualizer_.publishMissionStatistics(mission_time, self.evaluator_.getScore(), self.mission_time_limit)

    # #} end of publishPlaybackSimulation()

    # #{ checkMaximumSolutionTime()

    def checkMaximumSolutionTime(self):
        solution_time = (self.solution_time_end - self.solution_time_start).to_sec()
        solution_time_penalty = 0.0
        if solution_time > self.solution_time_constraint_hard:
            with self.diag_msg_lock:
                self.diag_msgs.append("Maximum solution time exceeded!")
            self.evaluator_.setZeroScore()
        elif solution_time > self.solution_time_constraint_soft:
            solution_time_penalty = solution_time - self.solution_time_constraint_soft

        return solution_time, solution_time_penalty

    # #} end of checkMaximumSolutionTime()

    # #{ publishOverallStatus()

    def publishOverallStatus(self, event):
        self.pub_trajectories_valid_status.publish(Bool(self.overall_status))

    # #} end of publishOverallStatus()

    # #{ CALLBACKS

    # #{ diagnosticsMsgCallback()

    def diagnosticsMsgCallback(self, event):
        msg = ""
        with self.diag_msg_lock:
            if len(self.diag_msgs) > 0:
                msg = self.diag_msgs[0]

        if not msg == "":
            r_on = rospy.Duration(1.0)
            r_off = rospy.Duration(0.3)
            for m in range(3):
                self.visualizer_.publishDiagnosticMsg(msg)
                rospy.sleep(r_on)
                self.visualizer_.publishDiagnosticMsg("")
                rospy.sleep(r_off)

            self.diag_msgs.remove(self.diag_msgs[0])

    # #} end of diagnosticsMsgCallback()

    # #{ startMonitoringCallback()

    def startMonitoringCallback(self, req):
        if not self.mission_started:
            self.mission_started = True
            self.task_monitor.start()
        elif not self.mission_finished:
            self.mission_finished = True
            self.task_monitor.stop()

    # #} end of startMonitoringCallback()

    # #{ TRAJECTORY CALLBACKS

    def callbackFirstTrajectory(self, msg):

        if not self.trajectory1_subscribed and self.trajectory2_subscribed:
            self.solution_time_end = rospy.Time.now()

        self.trajectory1_subscribed = True
        self.trajectory1 = []
        for point in msg.points:
            self.trajectory1.append(Vector4d(point.position.x, point.position.y, point.position.z, point.heading))

        rospy.loginfo("[MrimManager] Trajectory 1 of length %lu subscribed.", len(self.trajectory1))

    def callbackSecondTrajectory(self, msg):
        if not self.trajectory2_subscribed and self.trajectory1_subscribed:
            self.solution_time_end = rospy.Time.now()

        self.trajectory2_subscribed = True
        self.trajectory2 = []
        for point in msg.points:
            self.trajectory2.append(Vector4d(point.position.x, point.position.y, point.position.z, point.heading))

        rospy.loginfo("[MrimManager] Trajectory 2 of length %lu subscribed.", len(self.trajectory2))

    # #} end of TRAJECTORY CALLBACKS

    # #{ callbackInspectionProblem()

    def callbackInspectionProblem(self, msg):
        if not self.inspection_problem_subscribed:
            self.solution_time_start = rospy.Time.now()

        self.inspection_problem = msg
        self.inspection_problem_subscribed = True
        rospy.loginfo_once("[MrimManager] Inspection problem subscribed")

    # #} end of callbackInspectionproblem()

    # #{ callbackPausePlayback()

    def callbackPausePlayback(self, msg):
        self.playback_paused = not self.playback_paused
        self.visualizer_.publishPlaybackStatus(self.playback_paused)
        rospy.loginfo("[MrimManager] Trajectory playback pause status changed.")

    # #} end of callbackPausePlayback()

    # #{ callbackUavStates()

    def callbackUavState1(self, msg):
        self.uav_state1_subscribed = True
        with self.uav_states_lock:
            self.uav_states[0] = msg

    def callbackUavState2(self, msg):
        self.uav_state2_subscribed = True
        with self.uav_states_lock:
            self.uav_states[1] = msg

    # #} end of callbackUavStates()

    # #} end of CALLBACKS

# #} end of MRIM MANAGER

if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, signal_handler) # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
        mrim_manager = MrimManager()
    except rospy.ROSInterruptException:
        pass

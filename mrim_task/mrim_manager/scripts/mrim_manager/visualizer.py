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

# #{ class Visualizer

class Visualizer:
    def __init__(self, inspection_problem, jsk_msg_publishers, cone_publishers, inspection_points_publisher, viewpoints_publisher, collisions_publisher, start_positions_publisher, start_arrows_publisher, safety_area_publisher, pcl_publisher, mesh_publisher, path_publishers, odometry_publishers, horizon_publishers, obst_dist_publishers, mutual_dist_publishers, vel_publishers, acc_publishers, traveled_dist_publishers, time_elapsed_publisher, remaining_time_publisher, score_publisher, playback_status_publisher, diagnostic_publisher, solution_time_publisher, solution_penalty_publisher, fullscreen_msg_publisher, frame):
        self.jsk_msg_publishers = jsk_msg_publishers
        self.cone_publishers = cone_publishers
        self.ip_publisher = inspection_points_publisher
        self.collisions_publisher = collisions_publisher
        self.safety_area_publisher = safety_area_publisher
        self.pcl_publisher = pcl_publisher
        self.mesh_publisher = mesh_publisher
        self.vp_publisher = viewpoints_publisher
        self.start_positions_publisher = start_positions_publisher
        self.path_publishers = path_publishers
        self.odometry_publishers = odometry_publishers
        self.horizon_publishers = horizon_publishers
        self.obst_dist_publishers = obst_dist_publishers
        self.mutual_dist_publishers = mutual_dist_publishers
        self.acc_publishers = acc_publishers
        self.vel_publishers = vel_publishers
        self.traveled_dist_publishers = traveled_dist_publishers
        self.time_elapsed_publisher = time_elapsed_publisher
        self.remaining_time_publisher = remaining_time_publisher
        self.score_publisher = score_publisher
        self.playback_status_publisher = playback_status_publisher
        self.diagnostic_publisher = diagnostic_publisher
        self.solution_time_publisher = solution_time_publisher
        self.solution_penalty_publisher = solution_penalty_publisher
        self.fullscreen_msg_publisher = fullscreen_msg_publisher
        self.frame = frame
        self.safety_area_msg = self.createSafetyAreaMsg(inspection_problem)
        self.cloud_msg = self.createCloudMessage(inspection_problem)
        self.mesh_msg = self.createMeshMessage(inspection_problem)
        self.start_points_msg = self.createStartPointsMsg(inspection_problem.start_poses)
        self.start_arrows_publisher = start_arrows_publisher

    def setTrajectories(self, trajectories):
        self.start_arrow_msg = self.createStartArrowsMsg(trajectories)

    def setRvizConfig(self, input_file, output_file, max_score, mission_time_limit, solution_time_limit_soft, solution_time_limit_hard, uav_name_1, uav_name_2):
        fin = open(input_file, "rt")
        fout = open(output_file, "wt")
        score_found = False
        mission_time_found = False
        elapsed_time_found = False
        solution_time_found = False
        solution_penalty_found = False
        global_options_found = False

        for line in fin:
            if "Topic: /score" in line:
                score_found = True

            if "Topic: /mission_time" in line:
                mission_time_found = True

            if "Topic: /elapsed_time" in line:
                elapsed_time_found = True

            if "Topic: /visualization/solution_penalty" in line:
                solution_penalty_found = True

            if "Topic: /visualization/remaining_solution_time" in line:
                solution_time_found = True

            if "Global Options:" in line:
                global_options_found = True

            if score_found and "max value" in line:
                new_line = re.sub("max value: [0-9]*", "max value: " + str(max_score), line)
                score_found = False
            elif mission_time_found and "max value" in line:
                new_line = re.sub("max value: [0-9]*", "max value: " + str(mission_time_limit), line)
                mission_time_found = False
            elif elapsed_time_found and "max value" in line:
                new_line = re.sub("max value: [0-9]*", "max value: " + str(mission_time_limit), line)
                elapsed_time_found = False
            elif solution_penalty_found and "max value" in line:
                new_line = re.sub("max value: [0-9]*", "max value: " + str(solution_time_limit_hard - solution_time_limit_soft), line)
                solution_penalty_found = False
            elif solution_time_found and "max value" in line:
                new_line = re.sub("max value: [0-9]*", "max value: " + str(solution_time_limit_hard), line)
                solution_time_found = False
            elif global_options_found and "Fixed Frame" in line:
                new_line = re.sub("Fixed Frame: .*", "Fixed Frame: " + self.frame, line)
                global_options_found = False
            elif "uav1" in line:
                new_line = re.sub("uav1", uav_name_1, line)
            elif "uav2" in line:
                new_line = re.sub("uav2", uav_name_2, line)
            else:
                new_line = line

            fout.write(new_line)

        fin.close()
        fout.close()


    def generateJskMsg(self, uav_name, trajectory_length, trajectory_time, min_obst_dist, min_mutual_dist, dynamics_ok):
        text = uav_name + ":\nTrajectory length = " + "{:.1f}".format(trajectory_length) + " m\n"\
            + "Trajectory time      = " + "{:.1f}".format(trajectory_time) + " s\n" \
            + "Min. UAV-obst. dist. = " + "{:.1f}".format(min_obst_dist) + " m\n" \
            + "Min. UAV-UAV dist.   = " + "{:.1f}".format(min_mutual_dist) + " m\n " \
            + "Dynamics check: " + boolToString(dynamics_ok) + "\n"

        return text

    def publishDiagnosticMsg(self, input_text):
        text = OverlayText()
        text.text = input_text
        text.width = 600
        text.height = 50
        text.left = 10
        text.top = 10
        text.text_size = 16
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.fg_color = ColorRGBA(0.9, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        self.diagnostic_publisher.publish(text)

    def publishFullScreenMsg(self, input_text):
        text = OverlayText()
        text.text = input_text
        text.width = 900
        text.height = 90
        text.left = 10
        text.top = 10
        text.text_size = 24
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.fg_color = ColorRGBA(0.9, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        self.fullscreen_msg_publisher.publish(text)

    def publishJskMsg(self, input_text, overall_status, trajectory_idx):
        text = OverlayText()
        text.text = input_text
        text.width = 1400
        text.height = 200
        text.left = 10
        text.top = 10
        text.text_size = 14
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.bg_color = ColorRGBA(0.0, 0.8, 0.0, 1.0) if overall_status else ColorRGBA(0.8, 0.0, 0.0, 1.0)
        text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        self.jsk_msg_publishers[trajectory_idx].publish(text)

    def publishPlaybackStatus(self, playback_status):
        text = OverlayText()
        text.text = "PLAYBACK RUNNING" if not playback_status else "PLAYBACK PAUSED"
        text.width = 200
        text.height = 50
        text.left = 10
        text.top = 10
        text.text_size = 14
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.fg_color = ColorRGBA(0.1, 0.9, 0.1, 1.0) if not playback_status else ColorRGBA(0.9, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        self.playback_status_publisher.publish(text)

    def publishSolutionTime(self, remaining_solution_time, solution_time_penalty):
        self.solution_time_publisher.publish(Float32(remaining_solution_time))
        self.solution_penalty_publisher.publish(Float32(solution_time_penalty))

    def publishUavStatistics(self, trajectory_idx, obst_dist, mutual_dist, vel, acc, traveled_dist):
        self.obst_dist_publishers[trajectory_idx].publish(Float32(obst_dist))
        self.mutual_dist_publishers[trajectory_idx].publish(Float32(mutual_dist))
        self.vel_publishers[trajectory_idx].publish(Float32(vel))
        self.acc_publishers[trajectory_idx].publish(Float32(acc))
        self.traveled_dist_publishers[trajectory_idx].publish(Float32(traveled_dist))

    def publishMissionStatistics(self, mission_time, score, mission_time_limit):
        self.time_elapsed_publisher.publish(Float32(mission_time))
        self.remaining_time_publisher.publish(Float32(max(0, mission_time_limit - mission_time)))
        self.score_publisher.publish(Float32(score))

    def publishHorizon(self, trajectories, start_idx, horizon_length):
        for k in range(len(trajectories)):
            msg = PoseArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame
            start_idx_local = min(len(trajectories[k].poses) - 1, start_idx)
            end_idx = min(len(trajectories[k].poses), start_idx + horizon_length)

            for m in range(start_idx_local, end_idx):
                msg.poses.append(trajectoryPointToPoseMsg(trajectories[k].poses[m]))

            self.horizon_publishers[k].publish(msg)

    def publishPaths(self, trajectories):
        for k in range(len(trajectories)):
            path_k = trajectoryToPathMsg(trajectories[k], self.frame)
            self.path_publishers[k].publish(path_k)

    def publishCollisions(self, trajectories, collisions):
        msg = self.createCollisionsMsg(trajectories, collisions)
        self.collisions_publisher.publish(msg)

    def publishCone(self, pose, trajectory_idx):
        if(trajectory_idx == 1):
            cone = self.fov2
            cone.pose = pose
            self.cone_publishers[trajectory_idx].publish(cone)
        else:
            cone = self.fov1_rgb
            cone.pose = pose
            self.cone_publishers[trajectory_idx].publish(cone)
            cone = self.fov1_thermal
            cone.pose = pose
            self.cone_publishers[trajectory_idx].publish(cone)

    def publishOdometry(self, odom, trajectory_idx):
        self.odometry_publishers[trajectory_idx].publish(odom)

    def publishInspectionPoints(self, inspection_points, viewpoints):
        msg = self.createInspectionPointsMsg(inspection_points, viewpoints, 0.5)
        self.ip_publisher.publish(msg)

    def publishViewPoints(self, inspection_points, viewpoints):
        msg_vps = self.createViewPointsMsg(inspection_points, viewpoints, 0.3)
        self.vp_publisher.publish(msg_vps)

    def publishStartPositions(self):
        msg = self.start_points_msg
        self.start_positions_publisher.publish(msg)
        self.start_arrows_publisher.publish(self.start_arrow_msg)

    def publishStartPositionsWithoutArrows(self):
        msg = self.start_points_msg
        self.start_positions_publisher.publish(msg)

    def publishObstacles(self):
        msg = self.cloud_msg
        msg.header.stamp = rospy.Time.now()
        self.pcl_publisher.publish(msg)

        msg = self.mesh_msg
        msg.header.stamp = rospy.Time.now()
        self.mesh_publisher.publish(msg)

    def publishSafetyArea(self):
        msg = self.safety_area_msg
        msg.header.stamp = rospy.Time.now()
        self.safety_area_publisher.publish(msg)

    def setFov1(self, fov_length, aov_h, aov_v, x_scale, red, green, blue):
        self.fov1_rgb = self.createConeMsg(fov_length, aov_h, aov_v, x_scale, red, green, blue, 2, "RGB")
        self.fov1_thermal = self.createConeMsg(fov_length, aov_h, aov_v, x_scale, red, green, blue, 2, "Thermal")

    def setFov2(self, fov_length, aov_h, aov_v, x_scale, red, green, blue):
        self.fov2 = self.createConeMsg(fov_length, aov_h, aov_v, x_scale, red, green, blue, 1, "RGB")

    def createCloudMessage(self, inspection_problem):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame
        cloud_points = geometryMsgsPointsToPclCloud(inspection_problem.obstacle_points)
        pcl_msg = sensor_msgs.point_cloud2.create_cloud_xyz32(header, cloud_points)
        return pcl_msg

    def createMeshMessage(self, inspection_problem):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame
        mesh_msg = Marker()
        mesh_msg.header = header
        mesh_msg.type = Marker.MESH_RESOURCE
        mesh_msg.action = Marker.ADD
        mesh_msg.scale = Vector3(1.0, 1.0, 1.0)
        mesh_msg.color = ColorRGBA(0.5, 0.5, 0.5, 1.0)
        mesh_msg.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        mesh_msg.mesh_resource = inspection_problem.mesh_path
        mesh_msg.mesh_use_embedded_materials = True
        return mesh_msg

    def createConeMsg(self, fov_length, aov_h, aov_v, x_scale, red, green, blue, marker_id, camera_type):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp    = rospy.Time.now()
        marker.ns              = "cone_ns" + str(marker_id)
        marker.id              = marker_id
        marker.type            = Marker.LINE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = x_scale
        marker.scale.y         = 0.0
        marker.scale.z         = 0.0
        marker.color.r         = red
        marker.color.g         = green
        marker.color.b         = blue
        marker.color.a         = 1.0
        marker.frame_locked    = True
        fov_h2          = aov_h / 2.0
        fov_v2          = aov_v / 2.0
        x               = fov_length * math.tan(fov_v2)
        y               = fov_length * math.tan(fov_h2)
        z               = fov_length

        points = [];
        points.append([0, 0, 0]);
        points.append([x, y, z]);
        points.append([0, 0, 0]);
        points.append([-x, y, z]);
        points.append([0, 0, 0]);
        points.append([-x, -y, z]);
        points.append([0, 0, 0]);
        points.append([x, -y, z]);
        points.append([x, y, z]);
        points.append([-x, y, z]);
        points.append([-x, y, z]);
        points.append([-x, -y, z]);
        points.append([-x, -y, z]);
        points.append([x, -y, z]);
        points.append([x, -y, z]);
        points.append([x, y, z]);

        if(camera_type == "RGB" or camera_type == "rgb"):
            for k in range(len(points)):
                pt = Point()
                pt.x = points[k][2]*np.cos(-np.pi/4) + points[k][0]*np.sin(-np.pi/4)
                pt.y = points[k][1]
                pt.z = points[k][2]*np.sin(-np.pi/4) - points[k][0]*np.cos(-np.pi/4)
                marker.points.append(pt)
        else:
            for k in range(len(points)):
                pt = Point()
                pt.x = points[k][0]
                pt.y = points[k][1]
                pt.z = -points[k][2]
                marker.points.append(pt)
            marker.id = 0

        return marker

    def createSafetyAreaMsg(self, inspection_problem):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp    = rospy.Time.now()
        marker.ns              = "safety_area_ns";
        marker.id              = 1;
        marker.type            = Marker.LINE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = 0.2
        marker.scale.y         = 0.0
        marker.scale.z         = 0.0
        marker.color.r         = 1.0
        marker.color.g         = 0.0
        marker.color.b         = 0.0
        marker.color.a         = 0.4
        marker.frame_locked    = True
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        points = []
        for p in inspection_problem.safety_area:
            pt = Point()
            pt.x = p.x
            pt.y = p.y
            points.append(pt)

        for p in points:
            for height in [inspection_problem.min_height, inspection_problem.max_height]:
                pt = deepcopy(p)
                pt.z = height
                marker.points.append(pt)

        for height in [inspection_problem.min_height, inspection_problem.max_height]:
            row_points = []
            for p in points:
                pt = Point()
                pt.x = p.x
                pt.y = p.y
                pt.z = height
                row_points.append(pt)
                row_points.append(pt)

            row_points.append(row_points.pop(0))
            marker.points.extend(row_points)

        return marker

    def createViewPointsMsg(self, inspection_points, viewpoints, scale):
        vp_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.frame;
        marker.header.stamp    = rospy.Time.now()
        marker.ns              = "vp_ns";
        marker.id              = 1;
        marker.type            = Marker.CUBE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = scale
        marker.scale.y         = scale
        marker.scale.z         = scale
        marker.frame_locked    = True
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        lines = Marker()
        lines.header.frame_id = self.frame;
        lines.header.stamp    = rospy.Time.now()
        lines.ns              = "line_ns";
        lines.id              = 2;
        lines.type            = Marker.LINE_LIST
        lines.action          = Marker.ADD
        lines.scale.x         = 0.05
        lines.scale.y         = 0.05
        lines.scale.z         = 0.05
        lines.frame_locked    = True
        lines.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        red = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        green = ColorRGBA(0.0, 1.0, 0.0, 0.5)
        blue = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        purple = ColorRGBA(1.0, 0.0, 1.0, 0.5)

        for k in range(len(inspection_points)):
            pt = Point()
            pt.x = inspection_points[k].position.x
            pt.y = inspection_points[k].position.y
            pt.z = inspection_points[k].position.z
            pt2 = Point()
            pt2.x = viewpoints[k].x
            pt2.y = viewpoints[k].y
            pt2.z = viewpoints[k].z
            marker.points.append(pt2)
            lines.points.append(pt)
            lines.points.append(pt2)

            color = ColorRGBA()
            color = green if viewpoints[k].is_inspected else red if viewpoints[k].color_index == 1 else blue if viewpoints[k].color_index == 2 else purple
            marker.colors.append(color)
            lines.colors.append(color)
            lines.colors.append(color)

        vp_markers.markers.append(marker)
        vp_markers.markers.append(lines)

        return vp_markers

    def createCollisionsMsg(self, trajectories, collisions):

        lines = Marker()
        lines.header.frame_id = self.frame;
        lines.header.stamp    = rospy.Time.now()
        lines.ns              = "lines";
        lines.id              = 1;
        lines.type            = Marker.LINE_LIST
        lines.action          = Marker.ADD
        lines.scale.x         = 0.05
        lines.scale.y         = lines.scale.x
        lines.scale.z         = lines.scale.x
        lines.frame_locked    = True
        lines.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        black = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        for key in collisions:
            t, t_r    = key
            for i in collisions[key]:
                g_a, g_b = trajectories[t].poses[i], trajectories[t_r].poses[i]
                pt_a, pt_b = Point(), Point()
                pt_a.x, pt_a.y, pt_a.z = g_a.x, g_a.y, g_a.z
                pt_b.x, pt_b.y, pt_b.z = g_b.x, g_b.y, g_b.z

                lines.points.append(pt_a)
                lines.points.append(pt_b)
                lines.colors.append(black)
                lines.colors.append(black)

        return lines

    def createStartArrowsMsg(self, trajectories):
        marker_array = MarkerArray()
        for k in range(len(trajectories)):
            marker = Marker()
            marker.header.frame_id = self.frame;
            marker.header.stamp    = rospy.Time.now()
            marker.ns              = "arrow_ns";
            marker.id              = k;
            marker.type            = Marker.ARROW
            marker.action          = Marker.ADD
            marker.scale.x         = 0.12
            marker.scale.y         = 0.27
            marker.frame_locked    = True
            marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.3) if k == 0 else ColorRGBA(0.0, 0.0, 1.0, 0.3)

            min_idx = min(7, len(trajectories[k].poses) - 1)
            pt0 = Point()
            pt0.x = trajectories[k].poses[0].x
            pt0.y = trajectories[k].poses[0].y
            pt0.z = trajectories[k].poses[0].z
            marker.points.append(pt0)

            pt1 = Point()
            pt1.x = trajectories[k].poses[min_idx].x
            pt1.y = trajectories[k].poses[min_idx].y
            pt1.z = trajectories[k].poses[min_idx].z
            marker.points.append(pt1)

            marker_array.markers.append(marker)

        return marker_array

    def createInspectionPointsMsg(self, inspection_points, viewpoints, scale):
        marker = Marker()
        marker.header.frame_id = self.frame;
        marker.header.stamp    = rospy.Time.now()
        marker.ns              = "ip_ns";
        marker.id              = 1;
        marker.type            = Marker.SPHERE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = scale
        marker.scale.y         = scale
        marker.scale.z         = scale
        marker.frame_locked    = True
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        red = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        green = ColorRGBA(0.0, 1.0, 0.0, 0.5)
        blue = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        purple = ColorRGBA(1.0, 0.0, 1.0, 0.5)

        for k in range(len(inspection_points)):
            pt = Point()
            pt.x = inspection_points[k].position.x
            pt.y = inspection_points[k].position.y
            pt.z = inspection_points[k].position.z
            marker.points.append(pt)
            color = ColorRGBA()
            color = green if viewpoints[k].is_inspected else red if viewpoints[k].color_index == 1 else blue if viewpoints[k].color_index == 2 else purple
            marker.colors.append(color)

        return marker

    def createStartPointsMsg(self, start_poses):
        #future fix: fix for arbitrary number of start poses
        sp_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.frame;
        marker.header.stamp    = rospy.Time.now()
        marker.ns              = "sp_ns";
        marker.id              = 0;
        marker.type            = Marker.CYLINDER
        marker.action          = Marker.ADD
        marker.scale.x         = 1.0
        marker.scale.y         = 1.0
        marker.scale.z         = 0.3
        marker.frame_locked    = True
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        marker.pose.position = start_poses[0].position
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        sp_markers.markers.append(deepcopy(marker))
        marker.pose.position = start_poses[-1].position
        marker.id              = 1;
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        sp_markers.markers.append(marker)

        return sp_markers

# #} end of class Visualizer

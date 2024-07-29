#!/usr/bin/env python3

import rospy, rospkg, yaml, math

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from mpl_toolkits.mplot3d import Axes3D

from data_types import *

import numpy as np

COLORS = ['r', 'b']

## | ------------------------ Functions ----------------------- |

# #{ distEuclidean()

def distEuclidean(point_A, point_B):
    '''
    Euclidean distance between point_A and point_B.

    Parameters:
        point_A
        point_B

    Returns:
        float: Euclidean norm between the two 3D points
    '''

    if isinstance(point_A, Point) and isinstance(point_B, Point):
        return np.linalg.norm(point_A.asArray() - point_B.asArray())
    elif isinstance(point_A, Pose) and isinstance(point_B, Pose):
        return np.linalg.norm(point_A.point.asArray() - point_B.point.asArray())
    elif isinstance(point_A, (list, tuple)) and isinstance(point_B, (list, tuple)):
        return np.linalg.norm([point_A[0] - point_B[0], point_A[1] - point_B[1], point_A[2] - point_B[2]])

# #}

# #{ poseInDistance()

def poseInDistance(start, stop, dist):
    '''
    Returns a pose on line from start to stop at distance dist from start.

    Parameters:
        start (Pose): the start pose
        stop (Pose): the stop pose
        dist (float): the distance

    Returns:
        pose (Pose): Pose at given distance from start
    '''
    dist_tot = distEuclidean(start, stop)

    if dist_tot < 1e-4:
        return start

    ratio = dist / dist_tot

    x  = start.point.x + (stop.point.x - start.point.x) * ratio
    y  = start.point.y + (stop.point.y - start.point.y) * ratio
    z  = start.point.z + (stop.point.z - start.point.z) * ratio
    dh = wrapAngle(stop.heading - start.heading) * ratio
    h  = wrapAngle(start.heading + dh)

    return Pose(x, y, z, h)

# #}

# # #{ unwrapAngle()
def unwrapAngle(angle):
    '''
    Returns:
        float: Angle in interval <0, 2pi) (rad)
    '''
    return (angle + 2.0 * np.pi) % (2.0 * np.pi)
# # #}

# # #{ wrapAngle()
def wrapAngle(angle):
    '''
    Returns:
        float: Angle in interval <-pi, pi) (rad)
    '''
    # wrap to interval <-pi, pi)
    return (angle + np.pi) % (2 * np.pi) - np.pi
# # #}

# # #{ angleDiff()
def angleDiff(angle_from, angle_to):
    '''
    Parameters:
        angle_from (float): start angle (rad)
        angle_to (float): end angle (rad)

    Returns:
        float: shortest signed angle between the two angles
    '''
    # get smallest difference between two angles (orientation_included)
    ang_diff   = unwrapAngle(angle_to) - unwrapAngle(angle_from)
    ang_dir    = np.sign(ang_diff)
    change_dir = abs(ang_diff) > np.pi
    smallest_ang_diff = 2 * np.pi - abs(ang_diff) if change_dir else abs(ang_diff)
    ang_dir           = -1 * ang_dir if change_dir else ang_dir
    return ang_dir * smallest_ang_diff

# # #}

# #{ trajectoryToRosMsg()

def trajectoryToRosMsg(poses, frame_id="rtk_origin"):
    '''
    Converts Trajectory poses to ROS msg of type mrs_msgs/TrajectoryReference

    Parameters:
        poses (list[Pose]): the trajectory poses

    Returns:
        trajectory_msg (mrs_msgs/TrajectoryReference): ROS msg
    '''

    trajectory_msg                 = TrajectoryReference()
    trajectory_msg.fly_now         = False
    trajectory_msg.use_heading     = True
    trajectory_msg.loop            = False
    trajectory_msg.header.frame_id = frame_id
    trajectory_msg.header.stamp    = rospy.Time.now();

    for pose in poses:

        ref            = Reference()
        ref.position.x = pose.point.x
        ref.position.y = pose.point.y
        ref.position.z = pose.point.z
        ref.heading    = pose.heading
        trajectory_msg.points.append(ref)

    return trajectory_msg

# #}

# # #{ inspectionPointToViewPoint()
def inspectionPointToViewPoint(inspection_point, vp_distance):
    '''
    Parameters:
        inspection_point (mrim_resources/InspectionPoint): the inspection point to be converted
        vp_distance (float): distance of the viewpoint from the inspection point

    Returns:
        list[5]: the viewpoint in format [index, x, y, z, inspection angle (UAV heading)]
    '''
    x = inspection_point.position.x + vp_distance * np.cos(inspection_point.inspect_heading) * np.sin(inspection_point.inspect_tilt)
    y = inspection_point.position.y + vp_distance * np.sin(inspection_point.inspect_heading) * np.sin(inspection_point.inspect_tilt)
    z = inspection_point.position.z + vp_distance * np.cos(inspection_point.inspect_tilt)

    heading = wrapAngle(np.pi + inspection_point.inspect_heading)

    return Viewpoint(inspection_point.idx, Pose(x, y, z, heading), inspection_point.type)
# # #}

# # #{ pointCollidesWithPath()
def pointCollidesWithPath(point, path, safety_distance):
    '''
    Checks whether a 3D point collides with a path

    Parameters:
        point (Point): the point
        path (list[Pose]): list of 3D poses

    Returns:
        bool: True if pose collides, False otherwise
    '''
    for p in path:
        if distEuclidean(point, p.point) < safety_distance:
            return True
    return False
# # #}

# #{ simulateStep()
def simulateStep(pose_from, pose_to, speed, dt):
    '''
    Simulates one dynamic step from given pose and speed towards the given pose.

    Parameters:
        pose_from (Pose)
        pose_to (Pose)
        speed (float): magnitude of the velocity
        det (float): sampling distance

    Returns:
        pose (Pose): pose after one simulation step from pose_from to pose_to
    '''
    # print("Simulate step: current_pose = ", pose_from, ", pose_to = ", pose_to, "speed = ", speed)
    dir_length  = distEuclidean(pose_to.point, pose_from.point)
    step_length = min(dir_length, speed * dt)

    # print("Step length: ", step_length)
    point = pose_from.point + step_length * (pose_to.point - pose_from.point) / dir_length

    return Pose(point, pose_from.heading)
# #} end of simulateStep()

# #{ segmentPointDist()
def segmentPointDist(seg_start, seg_end, point):
    '''
    Returns distance of point from a segment.

    Parameters:
        seg_start (Point): x, y coordinates of segment beginning.
        seg_end (Point): x, y coordinates of segment end
        point (Point): x, y coordinates of point

    Returns:
        dist (float): Euclidean distance between segment and point.
    '''
    seg     = seg_end - seg_start
    len_seg = seg.norm()

    if len_seg < 1e-6:
        return distEuclidean(seg_start, point)

    t    = max(0.0, min(1.0, np.dot((point - seg_start).asArray(), seg.asArray()) / len_seg**2))
    proj = seg_start + t * seg

    return (point - proj).norm()
# #}

# #{ lineSphereIntersections()
def lineSphereIntersections(sphere_center, sphere_radius, point1, point2):
    '''
    Finds intersections between a line and a sphere.

    Parameters:
        sphere_center (Point): center of the sphere
        sphere_radius (float): radius of the sphere
        point1 (Point): one point on the line
        point2 (Point): second point on the line

    Returns:
        list[Point]: zero, one, or two intersections
    '''
    cx = sphere_center.x
    cy = sphere_center.y
    cz = sphere_center.z

    px = point1.x
    py = point1.y
    pz = point1.z

    vx = point2.x - px;
    vy = point2.y - py;
    vz = point2.z - pz;

    A = vx**2 + vy**2 + vz**2
    B = 2.0 * (px * vx + py * vy + pz * vz - vx * cx - vy * cy - vz * cz)
    C = px**2 - 2 * px * cx + cx**2 + py**2 - 2 * py * cy + cy**2 + pz**2 - 2 * pz * cz + cz**2 - sphere_radius**2

    # discriminant
    D = B**2 - 4 * A * C

    if (D < 0):
        return []

    t1 = (-B - math.sqrt(D)) / (2.0 * A)

    solution1 = Point(px * (1 - t1) + t1 * point2.x, py * (1 - t1) + t1 * point2.y, pz * (1 - t1) + t1 * point2.z)

    t2 = (-B + math.sqrt(D)) / (2.0 * A)
    solution2 = Point(px * (1 - t2) + t2 * point2.x, py * (1 - t2) + t2 * point2.y, pz * (1 - t2) + t2 * point2.z)

    if ((t1 > 1 or t1 < 0) and (t2 > 1 or t2 < 0)):
        return []
    elif (not (t1 > 1 or t1 < 0) and (t2 > 1 or t2 < 0)):
        return [solution1]
    elif ((t1 > 1 or t1 < 0) and not(t2 > 1 or t2 < 0)):
        return [solution2]
    elif (D == 0):
        return [solution1]
    else:
        return [solution1, solution2]
# #} end of lineSphereIntersections

# # #{ pointCollidesWithObstacles()
def pointCollidesWithObstacles(point, obstacles, safety_distance):
    '''
    Checks if a point collides with obstacles (is below safety_distance threshold).

    Parameters:
        point (Point): the point
        obstacles (list): the obstacles as list of Point objects
        safety_distance (float): the collision radius

    Returns:
        bool: True if collides with any obstacle point, False otherwise
    '''
    for obst in obstacles:
        if distEuclidean(point, obst) < safety_distance:
            return True
    return False
# # #}

## | ------------------------- Classes ------------------------ |

# # #{ class ProblemPlotter
class ProblemPlotter:

# # #{ __init__()
    def __init__(self, use):
        self.use = use
        self.legend_elements = []
# # #}

    # # #{ addProblem()
    def addProblem(self, problem, annotate=True):
        """ plot MRIM problem using matplotlib"""

        if not self.use:
            return

        # setup figure
        figsize = (10, 7)
        fig = plt.figure(figsize=figsize)

        self.ax = fig.add_subplot(projection='3d')
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_zlabel('z [m]')

        # add obstacles
        self.addObstaclePoints(problem.obstacle_points, ms=5)

        # add individual inspection points
        for i in range(len(problem.robot_ids)):
            robot_id = problem.robot_ids[i]
            ips = [ip for ip in problem.inspection_points if len(ip.inspectability) == 1 and robot_id in ip.inspectability]
            self.addInspectionPoints(ips, COLORS[i], 'IPs/VPs for id: ' + str(robot_id), hl=0.3, ms=60, lw=2.5, annotate=False)

            # add start pose as cartesian axes
            start_pose = problem.start_poses[i]
            self.addAxes(start_pose.position, start_pose.heading, axis_len=0.5, lw=2.0, annotate=annotate, annotation=' id: ' + str(robot_id), annotation_color=COLORS[i])

        # add shared inspection points
        ips = [ip for ip in problem.inspection_points if len(ip.inspectability) == 0 or len(ip.inspectability) > 1]
        self.addInspectionPoints(ips, 'purple', 'IPs/VPs for all robots', hl=0.3, ms=60, lw=2.5, annotate=False)

        # add safety area
        self.addSafetyArea(problem.safety_area)

    # # #}

    # # #{ show()
    def show(self, legend=True):

        if not self.use:
            return

        if legend:
            self.ax.legend(handles=self.legend_elements, loc='upper right')

        self.setEqualAxes()


        # scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz']); ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)

        # ax.auto_scale_xyz([min([c.x for c in problem.safety_area]), max([c.x for c in problem.safety_area])], [min([c.y for c in problem.safety_area]), max([c.y for c in problem.safety_area])], [min([o.z for o in problem.obstacle_points]), max([o.z for o in problem.obstacle_points])])

        # xy = [c.x for c in problem.safety_area] + [c.y for c in problem.safety_area]
        # MAX = max(xy)
        # for direction in (-1, 1):
        #     for point in np.diag(direction * MAX * np.array([1,1,1])):
        #         ax.plot([point[0]], [point[1]], [point[2]], 'w')

        # xyzlim = np.array([ax.get_xlim3d(),ax.get_ylim3d(),ax.get_zlim3d()]).T
        # XYZlim = [min(xyzlim[0]),max(xyzlim[1])]
        # ax.set_xlim3d(XYZlim)
        # ax.set_ylim3d(XYZlim)
        # ax.set_zlim3d(XYZlim)

        plt.show()
    # # #}

    # # #{ setEqualAxes()
    def setEqualAxes(self):
        if not self.use:
            return
        extents = np.array([getattr(self.ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        sz = extents[:,1] - extents[:,0]
        centers = np.mean(extents, axis=1)
        maxsize = max(abs(sz))
        r = maxsize/2
        for ctr, dim in zip(centers, 'xyz'):
            getattr(self.ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)
    # # #}

    # # #{ addInspectionPoints()
    def addInspectionPoints(self, ips, color, label, hl=0.5, ms=60, lw=2.5, annotate=False):
        if not self.use:
            return

        for ip in ips:

            if ip.type == 't':
                self.ax.scatter([ip.position.x], [ip.position.y], [ip.position.z], '.', color=color, s=ms)
                self.ax.plot([ip.position.x, ip.position.x + hl * np.cos(ip.inspect_heading)],
                             [ip.position.y, ip.position.y + hl * np.sin(ip.inspect_heading)],
                             [ip.position.z, ip.position.z], '-', color=color, lw=lw)
            elif ip.type == 's':
                self.ax.scatter([ip.position.x], [ip.position.y], [ip.position.z], '.', color=color, s=ms)
                self.ax.plot([ip.position.x, ip.position.x],
                             [ip.position.y, ip.position.y],
                             [ip.position.z, ip.position.z + hl], '-', color=color, lw=lw)
            else:
                raise Exception(f"Type '{vp.type}' of inspection point is not valid! Valid types are: 's' for solar panel and 't' for tower.")

            if annotate:
                self.ax.text(ip.position.x, ip.position.y, ip.position.z, ' id: ' + str(ip.idx), 'x', color='black')

        self.legend_elements.append(Line2D([0], [0], marker='.', label=label, markerfacecolor=color, color='white', markersize=20))
    # # #}

    # # #{ addObstaclePoints()
    def addObstaclePoints(self, ops, ms=5):
        if not self.use:
            return

        self.ax.scatter([item.x for item in ops], [item.y for item in ops], [item.z for item in ops], '.', color='black', s=ms)
        self.legend_elements.append(Line2D([0], [0], marker='.', label='obstacles', markerfacecolor='black', color='white', markersize=10))
    # # #}

    # # #{ addViewPoints()
    def addViewPoints(self, vps, t_vp_to_ip_dist, s_vp_to_ip_dist, hl=0.5, ms=60, lw=2.5, annotate=False):
        if not self.use:
            return

        for r in range(len(vps)):
            for vp in vps[r]:

                # skip start pose
                if vp.idx == 0:
                    continue

                point   = vp.pose.point
                heading = vp.pose.heading

                self.ax.scatter([point.x], [point.y], [point.z], '.', color=COLORS[r], s=ms)

                if vp.type == 't':
                    # inspection point on tower
                    self.ax.plot([point.x, point.x + hl * np.cos(heading)],
                                 [point.y, point.y + hl * np.sin(heading)],
                                 [point.z, point.z], '-', color=COLORS[r], lw=lw)
                    self.ax.plot([point.x, point.x + t_vp_to_ip_dist * np.cos(heading)],
                                 [point.y, point.y + t_vp_to_ip_dist * np.sin(heading)],
                                 [point.z, point.z], '-', color=COLORS[r], lw=0.2)
                elif vp.type == 's':
                    # inspection point on solar panel
                    self.ax.plot([point.x, point.x],
                                 [point.y, point.y],
                                 [point.z, point.z - hl], '-', color=COLORS[r], lw=lw)
                    self.ax.plot([point.x, point.x],
                                 [point.y, point.y],
                                 [point.z, point.z - s_vp_to_ip_dist], '-', color=COLORS[r], lw=0.2)
                else:
                    raise Exception(f"Type '{vp.type}' of inspection point is not valid! Valid types are: 's' for solar panel and 't' for tower.")


                if annotate:
                    self.ax.text(point.x, point.y, point.z, ' id: ' + str(vp.idx), 'x', color='black')

            # self.legend_elements.append(Line2D([0], [0], marker='.', label='viewpoint (id: ' + str(r) + ')', markerfacecolor=COLORS[r], color='white', markersize=20))
    # # #}

    # # #{ addAxes()
    def addAxes(self, position, heading, axis_len=0.5, lw=2.5, annotate=False, annotation='', annotation_color='black'):

        if not self.use:
            return

        self.ax.plot([position.x, position.x + axis_len * np.cos(heading)],
                     [position.y, position.y + axis_len * np.sin(heading)],
                     [position.z, position.z], '-', color='r', lw=lw)
        self.ax.plot([position.x, position.x - axis_len * np.sin(heading)],
                     [position.y, position.y + axis_len * np.cos(heading)],
                     [position.z, position.z], '-', color='g', lw=lw)
        self.ax.plot([position.x, position.x], [position.y, position.y],
                     [position.z, position.z + axis_len], '-', color='b', lw=lw)

        if annotate:
            self.ax.text(position.x, position.y, position.z, annotation, 'x', color=annotation_color)
    # # #}

    # # #{ addSafetyArea()
    def addSafetyArea(self, safety_area):
        if not self.use:
            return
        self.ax.plot([c.x for c in safety_area] + [safety_area[0].x], [c.y for c in safety_area] + [safety_area[0].y], [c.z for c in safety_area] + [safety_area[0].z], 'k-')
        self.legend_elements.append(Line2D([0], [0], label='flight arena borders', color='k'))
    # # #}

    # # #{ addWaypoints()
    def addWaypoints(self, waypoints, color, lw=1.2,label=''):
        if not self.use:
            return
        x = [wp.point.x for wp in waypoints]
        y = [wp.point.y for wp in waypoints]
        z = [wp.point.z for wp in waypoints]
        self.ax.plot(x, y, z, '-', color=color, lw=lw)
        self.legend_elements.append(Line2D([0], [0], label=label, color=color))
    # # #}

    # # #{ addTrajectoryPoses()
    def addTrajectoryPoses(self, poses, color, hl=0.15, lw=1.0, ms=2.2, markeredgewidth=0.4, label=''):
        if not self.use:
            return
        x = [p.point.x for p in poses]
        y = [p.point.y for p in poses]
        z = [p.point.z for p in poses]
        h = [p.heading for p in poses]
        for i in range(len(x)):
            self.ax.plot([x[i], x[i] + hl * np.cos(h[i])],
                         [y[i], y[i] + hl * np.sin(h[i])],
                         [z[i], z[i]], '-', color=color, lw=lw)
        self.ax.plot(x, y, z, 'o', markerfacecolor=color, markeredgecolor='k', ms=ms, markeredgewidth=markeredgewidth)
        self.legend_elements.append(Line2D([0], [0], marker='.', label=label, markerfacecolor=color, color='white', markersize=10))
    # # #}

    # # #{ addPoints()
    def addPoints(self, points, color, ms=10, marker='.', label=''):
        if not self.use:
            return
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        z = [p[2] for p in points]
        self.ax.scatter(x, y, z, marker=marker, color=color, s=ms)

        if label != '':
            self.legend_elements.append(Line2D([0], [0], marker='.', label=label, markerfacecolor=color, color='white', markersize=10))
    # # #}

    # # #{ addGeometricCollisions()
    def addGeometricCollisions(self, traj_a, traj_b, safety_distance, color='black', lw=1.5):
        if not self.use:
            return

        for s_a in traj_a:
            for s_b in traj_b:
                if distEuclidean(s_a, s_b) < safety_distance:
                    self.ax.plot([s_a[0], s_b[0]], [s_a[1], s_b[1]], [s_a[2], s_b[2]], '-', color=color, lw=lw)
    # # #}

    # #{ plotDynamics()
    def plotDynamics(self, trajectories, max_vel, max_acc, robot_ids, dt=0.2):

        if not self.use:
            return

        fig, (ax1, ax2) = plt.subplots(nrows=2, sharex=False)
        fig.suptitle('Dynamics of the UAVs')

        ax11 = ax1.twinx()
        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('velocity [m/s]')
        ax11.set_ylabel('acceleration [m/s^2]')

        ax22 = ax2.twinx()
        ax2.set_xlabel('time [s]')
        ax2.set_ylabel('velocity [m/s]')
        ax22.set_ylabel('acceleration [m/s^2]')

        axes = [ax1, ax2]
        for t in range(len(trajectories)):
            trajectory = trajectories[t]
            ax         = axes[t]

            velocities = [0]
            for i in range(1, len(trajectory)):
                dist = distEuclidean(trajectory[i - 1].point, trajectory[i].point)
                velocities.append(dist / dt)
            velocities_time = [dt * i for i in range(len(velocities))]

            accelerations = [0]
            for i in range(1, len(velocities)):
                vel_change = velocities[i] - velocities[i - 1]
                accelerations.append(vel_change / dt)

            accelerations_time = [dt * i for i in range(len(accelerations))]

            ax.set_title('UAV id: ' + str(robot_ids[t]))
            ax.axhline(max_vel, 0, len(velocities), ls='-', color='k')
            ax.plot(velocities_time, velocities, '-', color=COLORS[t], label='velocity')
            ax.axhline(max_acc, 0, len(accelerations), ls='-.', color='k')
            ax.axhline(-max_acc, 0, len(accelerations), ls='-.', color='k')
            ax.plot(accelerations_time, accelerations, '-.', color=COLORS[t], label='acceleration')
            ax.legend(loc='upper right')

    # #}
# # #}

# # #{ class ProblemLoader
class ProblemLoader:

    # # #{ loadProblem()
    def loadProblem(self, filepath):
        """ load .problem file """

        problem = InspectionProblem()

        fp = None
        try:
            fp = open(filepath, 'r')
        except:
            return None, str("error opening file: " + filepath)

        line = True
        while line != "EOF":

            line = fp.readline().strip()

            if line.startswith("#"):
                continue

            if line.startswith("NAME"):
                problem.name = self.getParamOneLiner(line)

            elif line.startswith("COMMENT"):
                problem.comment = self.getParamOneLiner(line)

            elif line.startswith("ROBOTS_START"):
                while line != "EOF":
                    line = fp.readline().strip()
                    if line.startswith("#"):
                        continue
                    if line.startswith("ROBOTS_END"):
                        break

                    params = line.split(" ")
                    params = [x for x in params if x != '']
                    if len(params) != 5:
                        return None, str("wrong problem specification (robot line should contain ID and starting pose in format: ID x y z heading)")

                    start_pose = Reference()
                    start_pose.position.x = float(params[1])
                    start_pose.position.y = float(params[2])
                    start_pose.position.z = float(params[3])
                    start_pose.heading    = float(params[4])

                    problem.robot_ids.append(int(params[0]))
                    problem.start_poses.append(start_pose)

            elif line.startswith("INSPECTION_POINTS_START"):
                while line != "EOF":
                    line = fp.readline().strip()
                    if line.startswith("#"):
                        continue
                    if line.startswith("INSPECTION_POINTS_END"):
                        break
                    if not line:
                        continue

                    params = line.split(" ")
                    params = [x for x in params if x != '']
                    if len(params) < 5:
                        return None, str("wrong problem specification (inspection point should contain at least idx x y z heading)")

                    inspection_point = InspectionPoint()
                    inspection_point.idx = int(params[0])
                    inspection_point.position.x = float(params[1])
                    inspection_point.position.y = float(params[2])
                    inspection_point.position.z = float(params[3])
                    inspection_point.inspect_heading = float(params[4])
                    inspection_point.inspect_tilt = float(params[5])
                    inspection_point.type = str(params[6])
                    inspection_point.inspectability = []
                    for p in params[7:]:
                        if p == '#':
                            break
                        inspection_point.inspectability.append(int(p))

                    problem.inspection_points.append(inspection_point)

            elif line.startswith("OBSTACLE_POINTS"):
                filename = self.getParamOneLiner(line)
                obstacles_filepath = rospkg.RosPack().get_path('mrim_resources') + '/obstacles/' + filename
                problem.obstacle_points = self.loadObstacleFile(obstacles_filepath)
                if problem.obstacle_points is None:
                    return None, str("error opening file: " + obstacles_filepath)

            elif line.startswith("WORLD"):
                filename = self.getParamOneLiner(line)
                world_filepath = rospkg.RosPack().get_path('mrim_resources') + '/worlds/' + filename
                world = self.loadWorld(world_filepath)
                if world is None:
                    return None, str("error opening file: " + world_filepath)
                problem.min_height, problem.max_height, problem.safety_area = world[0], world[1], world[2]

            elif line.startswith("MODEL"):
                problem.model_name = self.getParamOneLiner(line)
                problem.mesh_path = "package://mrim_resources/blender_files/" + problem.model_name

        fp.close()

        problem.number_of_robots = len(problem.robot_ids)
        problem.number_of_inspection_points = len(problem.inspection_points)
        problem.number_of_obstacle_points = len(problem.obstacle_points)

        if len(problem.start_poses) != problem.number_of_robots:
            return None, str("wrong problem specification (len of start poses does not match len of robot ids)")
        if len(problem.inspection_points) == 0:
            return None, str("no inspection points specified")

        # if not specified, point is inspectable by all robots
        for ip in problem.inspection_points:
            if len(ip.inspectability) == 0:
                ip.inspectability = problem.robot_ids

        if len(problem.obstacle_points) == 0:
            rospy.logwarn('empty obstacle file: no obstacles loaded')

        return problem, "success"

    # # #}

    # # #{ loadObstacleFile()
    def loadObstacleFile(self, filepath):
        fp = None
        try:
            fp = open(filepath, 'r')
        except:
            return None

        points = []
        line = fp.readline().strip()
        while line:
            line_split = line.split(" ")

            obstacle_point = ROSPoint()
            obstacle_point.x = float(line_split[0])
            obstacle_point.y = float(line_split[1])
            obstacle_point.z = float(line_split[2])
            points.append(obstacle_point)

            line = fp.readline().strip()

        fp.close()

        return points
    # # #}

    # # #{ loadWorld()
    def loadWorld(self, filepath):
        print('[ProblemLoader] Loading world file:', filepath)
        fp = None
        try:
            fp = open(filepath, 'r')
            data_loaded = yaml.safe_load(fp)
        except:
            print('[ERROR] Failed to load world file.')
            return None
        fp.close()

        min_height, max_height = float(data_loaded['safety_area']['vertical']['min_z']), float(data_loaded['safety_area']['vertical']['max_z'])
        lateral_corners = []
        for idx in range(0, len(data_loaded['safety_area']['horizontal']['points']), 2):
            corner = ROSPoint()
            corner.x = data_loaded['safety_area']['horizontal']['points'][idx]
            corner.y = data_loaded['safety_area']['horizontal']['points'][idx+1]
            corner.z = 0
            lateral_corners.append(corner)
        return min_height, max_height, lateral_corners

    # # #}

    # # #{ getParamOneLiner()
    def getParamOneLiner(self, line, type=str):
        """ parse and change type of variable on .tsp file line """
        line_split = line.split(":")
        if len(line_split) == 2:
            return type(line_split[1].strip())
        else:
            print("error loading parameter from line", line)
            quit()
    # # #}

# # #}

"""
Custom TSP Loader
@author: P. Petracek, V. Kratky
"""

import math
import dubins

from utils import segmentPointDist, distEuclidean, lineSphereIntersections, simulateStep, wrapAngle, angleDiff, poseInDistance
from data_types import Pose, Viewpoint
import numpy as np

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
ta.setup_logging("INFO")

## | ------------------------- Classes ------------------------ |

# # #{ class Segment
class Segment:

    def __init__(self, idx_from, idx_to, poses=[]):
        self.idx_from = idx_from
        self.idx_to   = idx_to
        self.poses    = poses

    def getTime(self, dT):
        return len(self.poses) * dT
# # #}

# # #{ class Trajectory
class Trajectory:

    def __init__(self, dT, waypoints):
        self.dT                    = dT
        self.poses                 = []
        self.length                = 0.0
        self.len_changed           = True

        self.waypoints        = waypoints

        self.segments = [Segment(i-1, i) for i in range(1, len(self.waypoints))]

    # # #{ setSegment()
    def setSegment(self, seg_idx_from, poses):
        self.segments[seg_idx_from].poses = poses
        self.len_changed                  = True
    # # #}

    # # #{ getPoses()
    def getPoses(self):
        poses = []
        for seg in self.segments:
            poses = poses + seg.poses
        return poses
    # # #}

    # # #{ delaySegment()
    def delaySegment(self, seg_idx, t, at_start=False):
        '''
        Delays particular segment by t seconds either in start or end point.

        Parameters:
            seg_idx (int): segment index
            t (float): delay in seconds
            at_start (bool): delay is added at the start of the segment if True, at the end otherwise
        '''

        if seg_idx >= len(self.segments):
            return

        segment = self.segments[seg_idx]
        if len(segment.poses) == 0:
            return

        # Count number of poses to be added
        count = int(t / self.dT)

        # Copy start/end sample count-times
        index       = 0 if at_start else -1
        sequence    = count * [segment.poses[index]]

        # Replace poses at given segment
        self.segments[seg_idx].poses = segment.poses[:index] + sequence + segment.poses[index:]

    # # #}

    # # #{ delayStart()
    def delayStart(self, t):
        '''
        Delays trajectory at its beginning by t seconds.

        Parameters:
            t (float): delay in seconds
        '''

        self.delaySegment(0, t, at_start=True)

    # # #}

    # # #{ print()
    def print(self):
        print('Trajectory:')
        print(' - time: {:.1f} s'.format(self.getTime()))
        print(' - length: {:.1f} m'.format(self.getLength()))
        print(' - poses:')
        poses = self.getPoses()
        for i in range(len(poses)):
            p        = poses[i]
            appendix = ''
            print('  [{:d}]: ({:.4f}, {:.4f}, {:.4f}) | hdg: {:.2f}{:s}'.format(i, p[0], p[1], p[2], p[3], appendix))
    # # #}

    # # #{ GETTERS
    def getTime(self):
        samples_count = sum([len(seg.poses) for seg in self.segments])
        return samples_count * self.dT

    def getLength(self):

        # if not cached
        if self.len_changed:
            self.len_changed = False
            self.length      = 0.0

            for seg in self.segments:
                for i in range(1, len(seg.poses)):
                    self.length += distEuclidean(seg.poses[i - 1], seg.poses[i])

        return self.length
    # # #}

# # #}

# # #{ class TrajectoryUtils
class TrajectoryUtils():

    # #{ __init__()
    def __init__(self, max_velocity, max_acceleration, dT):
        self.dT               = dT
        self.max_velocity     = max_velocity
        self.max_acceleration = max_acceleration
    # #}

    ## | ------------- Functions: trajectory (re)sampling ------------- |

    # # #{ interpolateHeading()

    def interpolateHeading(self, waypoints:Viewpoint):
        '''
        Interpolates linearly the UAV heading between waypoints.

        Parameters:
            waypoints (list[Pose]): list of poses where some of them contain None in place of heading

        Returns:
            wps_interp (list[Pose]): list of interpolated waypoints with size equaling the input waypoints
        '''

        assert waypoints[0].heading  is not None, "[tsp_trajectory::interpolateHeading()] Heading of first waypoints sample is None. Can't interpolate."
        assert waypoints[-1].heading is not None, "[tsp_trajectory::interpolateHeading()] Heading of last waypoints sample is None. Last sample should equal the start."

        wps_interp = []

        idx = 0
        while idx != (len(waypoints) - 1):
            g_from  = waypoints[idx]

            # Find subtraj: sequence of poses (included) with defined heading
            subtraj = [g_from]
            for i in range(idx + 1, len(waypoints)):
                pose = waypoints[i]
                subtraj.append(pose)

                # Break at first heading-less pose
                if pose.heading is not None:
                    idx = i
                    break

            # get desired heading change from the beginning to end of this subtrajectory
            g_to  = subtraj[-1]
            delta_heading = wrapAngle(g_to.heading - g_from.heading)

            # get initial heading and subtraj length
            hdg_from    = wrapAngle(g_from.heading)
            current_heading = hdg_from
            subtraj_len = self.getLength(subtraj)

            ## | ----------------------- Interpolate ---------------------- |

            # include start node
            wps_interp.append(subtraj[0])

            # interpolate headings
            for i in range(1, len(subtraj) - 1):

                subtraj_point_0 = subtraj[i - 1].point
                subtraj_point_1 = subtraj[i].point

                # [STUDENTS TODO] Implement heading interpolation here
                # Tips:
                #  - subtrajectory is a section of trajectory and consists of poses to follow during this segment
                #  - interpolate the heading linearly (create a function of distance between two points of the subpath)
                #  - do not forget to wrap angle to (-pi, pi) (see/use wrapAngle() in utils.py)
                #  - see/use distEuclidean() in utils.py

                # [STUDENTS TODO] Change variable 'desired_heading', nothing else
                desired_heading = waypoints[0].heading

                # replace heading
                current_heading   = desired_heading
                wp         = subtraj[i]
                wp.heading = desired_heading
                wps_interp.append(wp)

        # include the very last node
        wps_interp.append(waypoints[-1])

        return wps_interp

    # # #}

    # #{ sampleStraightSegmentWithStops()
    def sampleStraightSegmentWithStops(self, start, stop):
        '''
        Samples linear segment with zero velocity at start and stop.

        Parameters:
            start (Pose): start of the segment
            stop (Pose): end of the segment

        Returns:
            poses (list[Pose]): list of poses which respect dynamic constraints of the UAV
        '''

        # print("sampleWithStops from", start, "to", stop)

        init_velocity     = 0
        final_velocity    = 0
        sample_start_time = 0

        poses                = []
        trajectory_part_time = 0
        # acc = 0 # no jeck jet
        dist_total = distEuclidean(start, stop)
        #print("dist_total", dist_total)

        # [STUDENTS TODO] Rework the method to per-axis computation if you want to exploit the allowed dynamics in all axes
        # Set minimal velocity/acceleration to the axis limit with minimal constraint
        min_ax_vel = min(self.max_velocity)
        min_ax_acc = min(self.max_acceleration)

        time_from_init_to_max_vel  = (min_ax_vel - init_velocity) / min_ax_acc
        time_from_max_to_final_vel = (min_ax_vel - final_velocity) / min_ax_acc

        dist_from_init_to_max_vel  = 0.5 * (min_ax_vel + init_velocity) * time_from_init_to_max_vel  # average speed * time
        dist_from_max_vel_to_final = 0.5 * (min_ax_vel + final_velocity) * time_from_max_to_final_vel  # average speed * time

        """
        print("time_from_init_to_max_vel", time_from_init_to_max_vel, "s")
        print("time_from_max_to_final_vel", time_from_max_to_final_vel, "s")
        print("dist_from_init_to_max_vel", dist_from_init_to_max_vel, "m")
        print("dist_from_max_vel_to_final", dist_from_max_vel_to_final, "m")
        """

        if dist_total < dist_from_init_to_max_vel + dist_from_max_vel_to_final: # can not reach maximal speed in straight line
            #print("can not reach max vel in trajectory")
            t      = 0
            sample = 0

            if init_velocity == 0 and final_velocity == 0:

                time_to_possible_max_vel = math.sqrt(dist_total / min_ax_acc)
                velocity_in_middle       = time_to_possible_max_vel * min_ax_acc
                trajectory_part_time     = 2 * time_to_possible_max_vel

            elif init_velocity > final_velocity:  # initial velocity is larger than final, in the end is additinal decelerating

                time_final_decel         = (init_velocity - final_velocity) / min_ax_acc
                dist_final_decel         = time_final_decel * (init_velocity + final_velocity) * 0.5
                dist_acc_decc            = dist_total - dist_final_decel
                time_to_possible_max_vel = (-init_velocity + math.sqrt(init_velocity ** 2 + min_ax_acc * dist_acc_decc)) / min_ax_acc
                velocity_in_middle       = init_velocity + time_to_possible_max_vel * min_ax_acc
                trajectory_part_time     = time_to_possible_max_vel + time_final_decel

            else:

                time_init_accel          = (final_velocity - init_velocity) / min_ax_acc
                dist_init_accel          = time_init_accel * (init_velocity + final_velocity) * 0.5
                dist_acc_decc            = dist_total - dist_init_accel
                time_to_possible_max_vel = time_init_accel + (-final_velocity + math.sqrt(final_velocity ** 2 + min_ax_acc * dist_acc_decc)) / min_ax_acc
                velocity_in_middle       = init_velocity + time_to_possible_max_vel * min_ax_acc

                """
                print("time_init_accel", time_init_accel)
                print("dist_init_accel", dist_init_accel)
                print("dist_total", dist_total)
                print("dist_acc_decc", dist_acc_decc)
                print("such dist is", 0.5 * (velocity_in_middle + init_velocity) * time_to_possible_max_vel * 2)
                """
                trajectory_part_time = 2 * time_to_possible_max_vel - time_init_accel

            """
            print("time_to_possible_max_vel", time_to_possible_max_vel)
            print("velocity_in_middle", velocity_in_middle)
            print("sample_start_time", sample_start_time)
            """

            while (sample + 1) * self.dT <= time_to_possible_max_vel - sample_start_time:

                t = (sample + 1) * self.dT + sample_start_time
                v = init_velocity + min_ax_acc * t
                s = init_velocity * t + 0.5 * min_ax_acc * (t ** 2)

                pose_in_dist = poseInDistance(start, stop, s)
                poses.append(pose_in_dist)
                sample += 1
                #print("t", t, "v", v, "s", s, "sample", sample)


            #print("end acc")

            while (sample + 1) * self.dT <= trajectory_part_time - sample_start_time:

                t      = (sample + 1) * self.dT + sample_start_time
                t_part = t - time_to_possible_max_vel
                v      = velocity_in_middle - min_ax_acc * t_part
                s      = time_to_possible_max_vel * 0.5 * (velocity_in_middle + init_velocity) + velocity_in_middle * t_part - 0.5 * min_ax_acc * (t_part ** 2)

                pose_in_dist = poseInDistance(start, stop, s)
                poses.append(pose_in_dist)
                sample += 1
                #print("t", t, "v", v, "s", s, "sample", sample)

            #print("end decc")

        else:  # can reach maximal speed in straight line

            #print("can reach max vel")
            dist_constant_speed  = dist_total - dist_from_init_to_max_vel - dist_from_max_vel_to_final
            time_constant_speed  = dist_constant_speed / min_ax_vel
            trajectory_part_time = time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel

            """
            print("time_constant_speed", time_constant_speed, "s")
            print("dist_constant_speed", dist_constant_speed, "m")
            print("trajectory_part_time", trajectory_part_time)
            """
            t = 0
            sample = 0
            while (sample + 1) * self.dT <= time_from_init_to_max_vel - sample_start_time:

                t = (sample + 1) * self.dT + sample_start_time
                v = init_velocity + min_ax_acc * t
                s = init_velocity * t + 0.5 * min_ax_acc * (t ** 2)

                pose_in_dist = poseInDistance(start, stop, s)
                poses.append(pose_in_dist)
                sample += 1
                #print("t", t, "v", v, "s", s, "sample", sample)

            #print("end acc")

            while (sample + 1) * self.dT <= time_from_init_to_max_vel + time_constant_speed - sample_start_time:

                t      = (sample + 1) * self.dT + sample_start_time
                t_part = t - time_from_init_to_max_vel
                v      = min_ax_vel
                s      = dist_from_init_to_max_vel + v * t_part

                pose_in_dist = poseInDistance(start, stop, s)
                poses.append(pose_in_dist)
                sample += 1
                #print("t", t, "v", v, "s", s, "sample", sample)

            #print("end const")

            while (sample + 1) * self.dT <= time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel - sample_start_time:

                t      = (sample + 1) * self.dT + sample_start_time
                t_part = t - (time_from_init_to_max_vel + time_constant_speed)
                v      = min_ax_vel - min_ax_acc * t_part
                s      = (dist_total - dist_from_max_vel_to_final) + min_ax_vel * t_part - 0.5 * min_ax_acc * (t_part ** 2)

                pose_in_dist = poseInDistance(start, stop, s)
                poses.append(pose_in_dist)
                sample += 1
                #print("t", t, "v", v, "s", s, "sample", sample)

            if final_velocity == 0 and poses[-1] != stop:
                poses.append(stop)
                #print("t last", "v", 0, "s", dist_total)

        return poses, trajectory_part_time
    # #}

    # #{ sampleTrajectoryThroughWaypoints()
    def sampleTrajectoryThroughWaypoints(self, trajectory, with_stops, smooth_path, smoothing_la_dist, smoothing_sampling_step, velocity_limits, acceleration_limits):
        '''
        Samples trajectory such that it respects dynamic constraints

        Parameters:
            trajectory (Trajectory): the trajectory which waypoints should be sampled
            with_stops (bool): zero velocity will is required in every waypoint
            smooth_path (bool): if True, path will be smoothed
            smoothing_la_dist (float): smoothing look-ahead distance in meters
            smoothing_sampling_dist (float): smoothing sampling distance in meters
            velocity limits (4x1 list of floats): velocity limits for (x, y, z, heading)
            acceleration limits (4x1 list of floats): acceleration limits for (x, y, z, heading)

        Returns:
            trajectory (Trajectory): trajectory which samples respect dynamic constraints of the UAV
        '''

        if with_stops:

            # Interpolate heading between waypoints
            trajectory.waypoints = self.interpolateHeading(trajectory.waypoints)

            # Iterate through sequential waypoint pairs
            for w_idx in range(1, len(trajectory.waypoints)):
                pose_from = trajectory.waypoints[w_idx - 1]
                pose_to   = trajectory.waypoints[w_idx]

                # Sample waypoint-to-waypoint line segments with stops at each start and end
                poses, _ = self.sampleStraightSegmentWithStops(pose_from, pose_to)

                # Add starting pose
                if w_idx == 1:
                    poses = [pose_from] + poses

                trajectory.setSegment(w_idx - 1, poses)

        else:

            # If path smoothing is required, smooth the path
            if smooth_path:
                print('[SMOOTHING PATH]')
                waypoints = self.getSmoothPath(trajectory.waypoints, smoothing_la_dist, smoothing_sampling_step)

            # Otherwise, use the waypoints
            else:
                waypoints = trajectory.waypoints

            if not waypoints:
                return None

            # Interpolate heading between waypoints
            traj_hdg_interp = self.interpolateHeading(waypoints)
            # Parametrize trajectory
            toppra_trajectory = self.getParametrizedTrajectory(traj_hdg_interp, velocity_limits, acceleration_limits)

            sampling_step = trajectory.dT

            # STUDENTS TODO: Sample the path parametrization 'toppra_trajectory' (instance of TOPPRA library).
            raise NotImplementedError('[STUDENTS TODO] Trajectory sampling not finished. You have to implement it on your own.')
            # Tips:
            #  - check code examples for TOPPRA: https://hungpham2511.github.io/toppra/auto_examples/index.html
            #  - use 'toppra_trajectory' and the predefined sampling step 'sampling_step'

            samples = [] # [STUDENTS TODO] Fill this variable with trajectory samples

            # Convert to Trajectory class
            poses      = [Pose(q[0], q[1], q[2], q[3]) for q in samples]
            trajectory = self.posesToTrajectory(poses)

        return trajectory
    # #}

    # #{ checkCollisionsOnHorizonAndBreak()
    def checkCollisionsOnHorizonAndBreak(self, trajectory, reference_trajectory, horizon, safety_distance):
        '''
        TODO
        '''

        # TODO: there is still some edge-case deadlock

        total_delay     = 0

        ref_poses     = reference_trajectory.getPoses()
        ref_poses_len = len(ref_poses)

        traj_poses     = trajectory.getPoses()
        traj_poses_len = len(traj_poses)

        print('[checkCollisionsOnHorizonAndBreak] horizon count: {:d}'.format(horizon))

        s_idx = 0
        while s_idx < traj_poses_len:

            traj_poses_len = len(traj_poses)
            collision_idx  = None

            for i in range(s_idx, min(s_idx + horizon, traj_poses_len)):

                # Reference trajectory has ended
                if i == ref_poses_len:
                    s_idx = traj_poses_len
                    break

                # Check if trajectories at this index collide
                if distEuclidean(traj_poses[i].point, ref_poses[i].point) < safety_distance:
                    collision_idx = i
                    break

            # Go to next sample if no collision was found
            if collision_idx is None:
                s_idx += 1
                continue

            # Derive velocities
            traj_velocities = self.getDerivation(traj_poses)

            min_velocity, min_velocity_idx = float("inf"), 0

            # Find sample with minimal velocity
            for i in range(s_idx, collision_idx - 1):
                vel_norm = traj_velocities[i].point.norm()
                if vel_norm < min_velocity:
                    min_velocity, min_velocity_idx = vel_norm, i

            # Add extra samples to delay the trajectory
            i = 0
            while i == 0 or distEuclidean(traj_poses[collision_idx].point, ref_poses[collision_idx].point) < safety_distance:

                # TODO: slow down continually instead of as fast as possible

                # Slow down with maximal deceleration
                stopping_dist  = min(0.0, min_velocity * self.dT - 0.5 * min_ax_acc * (self.dT ** 2))

                # Compute the sample pose
                stopping_start = traj_poses[min_velocity_idx]
                if stopping_dist > 0.0:
                    stopping_end = traj_poses[min_velocity_idx + 1]
                    pose         = poseInDistance(stopping_start, stopping_end, stopping_dist)
                else:
                    pose = stopping_start

                # Add the sample pose
                traj_poses = traj_poses[:min_velocity_idx + i + 1] + [pose] + traj_poses[min_velocity_idx + i + 1:]
                total_delay += 1

                min_velocity = stopping_dist / self.dT
                i += 1

            # Go from the start again as new collisions may occur from the delays
            s_idx = 0

        # Convert to Trajectory class
        trajectory = self.posesToTrajectory(traj_poses)

        return trajectory, total_delay * self.dT
    # #}

    # #{ computeTimeParametrization()
    def computeTimeParametrization(self, waypoints, v_lims, a_lims):
        '''
        Computes time parametrization among the given waypoints.

        Parameters:
            waypoints (list[Pose]): the waypoints to be parametrized
            v_lims (4x1 list of floats): velocity limits for (x, y, z, heading)
            a_lims (4x1 list of floats): acceleration limits for (x, y, z, heading)

        Returns:
            trajectory (TOPPRA instance): time parametrization of the given waypoints
        '''
        def get_normalized_distances(waypoints):
            distances = [0]
            for i in range(1, len(waypoints)):
                dist = distEuclidean(waypoints[i].point, waypoints[i-1].point)
                distances.append(distances[-1] + dist)
            total_distance = distances[-1]
            return [d / total_distance for d in distances]

        wp_lists = [wp.asList() for wp in waypoints]
        normalized_distances = get_normalized_distances(waypoints)
        path = ta.SplineInterpolator(normalized_distances, wp_lists)
        pc_vel     = constraint.JointVelocityConstraint(v_lims)
        pc_acc     = constraint.JointAccelerationConstraint(a_lims)
        instance   = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel", gridpt_max_err_threshold=1e-7)
        trajectory = instance.compute_trajectory()

        return trajectory
    # #} end of computeTimeParametrization()

    ## | -------- Functions: collision prevention/avoidance ------- |

    # # #{ resolveCollisions()
    def resolveCollisions(self, method, problem, trajectories, safety_distance):
        '''
        Postprocesses given trajectories such that there are no collisions.

        Parameters:
            method (string)
            problem (mrim_resources/InspectionProblem)
            trajectories (list[N]): list of possibly colliding trajectories

        Returns:
            trajectories (list[N]): list of non-colliding trajectories (hopefully)
            delayed_robots (list[N]): indices of delayed robots
            delays (list[N]): delay in seconds for each robot
        '''

        # # #{ Resolve collisions and resample trajectories

        print("[COLLISION AVOIDANCE] method: {:s}".format(method))
        delay_robot_idx, delay_t = None, 0.0
        delayed_robots, delays   = [], []

        ## |  [COLLISION AVOIDANCE METHOD #1]: Delay 2nd UAV by the length of the 1st UAV trajectory  |
        if method == 'delay_2nd_till_1st_UAV_finishes':

            delay_robot_idx = problem.number_of_robots - 1
            delay_t         = trajectories[0].getTime()

            # Delay trajectory of the second UAV at start by the length of the first-UAV trajectory
            trajectories[1].delayStart(delay_t)

        ## |  [COLLISION AVOIDANCE METHOD #2]: Delay UAV with shorter trajectory at start until there is no collision occurring  |
        elif method == 'delay_till_no_collisions_occur':

            raise NotImplementedError('[STUDENTS TODO] Collision prevention method \'delay_till_no_collisions_occur\' not finished. You have to finish it on your own.')
            # Tips:
            #  - you might select which trajectory it is better to delay
            #  - the smallest delay step is the sampling step stored in variable 'self.dT'

            delay_step = self.dT
            traj_times = [t.getTime() for t in trajectories]
            traj_lens  = [t.getLength() for t in trajectories]

            # Decide which UAV should be delayed
            # [STUDENTS TODO] CHANGE BELOW
            delay_robot_idx, nondelay_robot_idx = 0, 1

            # TIP: use function `self.trajectoriesCollide()` to check if two trajectories are in collision
            collision_flag, collision_idx = ...

            while collision_flag:

                # delay the shorter-trajectory UAV at the start point by sampling period
                delay_t += delay_step

                # [STUDENTS TODO] use function `trajectory.delayStart(X)` to delay a UAV at the start location by X seconds

                # keep checking if the robot trajectories collide
                collision_flag, _ = ...

        # # #}

        if delay_robot_idx is not None:
            delayed_robots, delays = [delay_robot_idx], [delay_t]

        return trajectories, delayed_robots, delays
    # # #}

    ## | ---------------- Functions: path smoothing --------------- |

    # #{ getSmoothPath()
    def getSmoothPath(self, waypoints, lookahead_distance, sampling_step):
        '''
        Smooths given waypoint path.

        Parameters:
            waypoints (list[Pose]): the waypoints to be parametrized
            lookahead_distance (float): smoothing look-ahead distance in meters
            sampling_step (float): smoothing sampling distance in meters

        Returns:
            path (list[Pose]): smoothed waypoint sequence
        '''

        path       = []
        curr_pose  = waypoints[0]
        path_index = 0

        path.append(curr_pose)

        # Main control loop
        while True:

            prev_path_index  = path_index
            goal, path_index = self.getLookaheadPoint(curr_pose, waypoints, lookahead_distance, path_index)

            if goal is None:
                return None

            if path_index == len(waypoints) - 1 and distEuclidean(waypoints[-1], curr_pose) < sampling_step:

                path[-1].heading = waypoints[path_index].heading # preserve last heading
                print("Goal reached. End point = ", curr_pose, "Waypoints goal = ", waypoints[-1])
                break

            # preserve assigned required heading
            if not prev_path_index == path_index:
                path[-1].heading = waypoints[prev_path_index].heading

            velocity = 1.0
            dt       = sampling_step / velocity

            curr_pose         = simulateStep(curr_pose, goal, velocity, dt)
            curr_pose.heading = None
            path.append(curr_pose)

        return path

    # #}

    ## | -------------------- Helper functions -------------------- |

    # # #{ getDerivation()
    def getDerivation(self, samples):
        '''
        Computes 1st order derivation of given samples of dimension N.

        Parameters:
            samples (list[Pose]): the samples to be derived

        Returns:
            derivatives (list[Pose]): smoothed waypoint sequence of size Nx1
        '''
        derivatives = []
        derivatives.append(Pose(0.0, 0.0, 0.0, 0.0))

        for k in range(1, len(samples)):
            dx       = (samples[k].point.x - samples[k-1].point.x) / self.dT
            dy       = (samples[k].point.y - samples[k-1].point.y) / self.dT
            dz       = (samples[k].point.z - samples[k-1].point.z) / self.dT
            dheading = angleDiff(samples[k-1].heading, samples[k].heading) / self.dT
            derivatives.append(Pose(dx, dy, dz, dheading))

        return derivatives
    # # #}

    # # #{ trajectoriesCollide()
    def trajectoriesCollide(self, traj_a, traj_b, safety_distance):
        '''
        Computes whether and possibly where two trajectories collide.

        Parameters:
            traj_a (Trajectory): first trajectory
            traj_b (Trajectory): second trajectory
            safety_distance (float): minimal distance between any two trajectory samples

        Returns:
            bool: collision flag
            int: collision index
        '''
        samples_a, samples_b = traj_a.getPoses(), traj_b.getPoses()
        min_len              = min([len(samples_a), len(samples_b)])
        for i in range(min_len):
            if distEuclidean(samples_a[i].point, samples_b[i].point) < safety_distance:
                return True, i
        return False, 0
    # # #}

    # # #{ computeCollisionSegmentsOfTwoTrajectories()
    def computeCollisionSegmentsOfTwoTrajectories(self, traj_A, traj_B, safety_distance):
        '''
        Computes collision segments between two trajectories.

        Parameters:
            traj_a (Trajectory): first trajectory
            traj_b (Trajectory): second trajectory
            safety_distance (float): minimal distance between any two trajectory samples

        Returns:
            segments (list): list of tuples (start_idx, end_idx) of colliding segments start and end indices
        '''

        segments = []

        # Compute which trajectory is shorter/longer
        trajs                   = [traj_A, traj_B]
        poses                   = [t.getPoses() for t in trajs]
        traj_times              = [t.getTime() for t in trajs]
        shorter_idx, longer_idx = np.argmin(traj_times), np.argmax(traj_times)

        # Compute hysteresis window (max stopping length)
        max_stop_time = max([self.max_velocity[i] / self.max_acceleration[i] for i in range(3)])
        hysteresis    = int(np.ceil(max_stop_time / self.dT))

        # Iterate over the collisions list
        col_prev                                 = pointCollidesWithPath(poses[shorter_idx][0].point, poses[longer_idx], safety_distance)
        longest_col_seg_sample_count             = 0
        safety_flag, first_col_idx, last_col_idx = True, 0, 0

        for i in range(1, len(poses[shorter_idx])):
            col_now = pointCollidesWithPath(poses[shorter_idx][i].point, poses[longer_idx], safety_distance)

            # collision->collision: do nothing
            if col_prev and col_now:
                col_prev = col_now
                continue

            # safe->collision: mark unsafe and store the index of first collision
            elif not col_prev and col_now:
                safety_flag   = False
                first_col_idx = i

            # collision->safe: store the index of last collision
            elif col_prev and not col_now:
                last_col_idx = i - 1

            # safe->safe: use hysteresis to mark the collision segment over
            elif not safety_flag and i - last_col_idx > hysteresis:
                safety_flag = True
                segments.append((first_col_idx, last_col_idx))

            col_prev = col_now

        return segments
    # # #}

    # #{ getLookaheadPoint()
    def getLookaheadPoint(self, pose, path, la_dist, path_index):
        '''
        Finds a point on the path at certain distance.

        Parameters:
            pose (Pose): the initial pose from which the looking starts from
            path (list[Pose]): the list of poses
            la_dist (float): distance from pose
            path_index (int): index of the pose in the path

        Returns:
            Pose: the lookahead point
            int: path_index of the found lookahead point
        '''
        goal_candidates = []

        while True:
            if path_index > 0:
                v_begin = path[path_index - 1]
                v_end   = path[path_index]
            else:
                v_begin = pose
                v_end   = path[path_index]
            # print("v begin and end = ", v_begin, v_end)

            # go to next segment if dist traveled is lower than lookahead dist and end of segment is closer than la dist
            dist_to_seg = segmentPointDist(v_begin.point, v_end.point, pose.point)
            dist_to_end = distEuclidean(pose.point, v_end.point)
            # print("dist to seg, dist_to_end = ", dist_to_seg, dist_to_end)

            if dist_to_end < la_dist:

                if path_index == len(path) - 1: # end of the path too close
                    goal_candidates.append(v_end)
                    break
                else:
                    path_index = min(path_index + 1, len(path) - 1)

            else:
                intersections   = lineSphereIntersections(pose.point, la_dist, v_begin.point, v_end.point)
                goal_candidates = []
                for i in range(len(intersections)):
                    intersection = intersections[i]
                    goal_candidates.append(Pose(intersection, pose.heading))
                # print("Goal candidates: ", goal_candidates)
                break

        if len(goal_candidates) == 0:
            goal_candidates.append(v_end)
            print("[ERROR] No intersection found, something bad happened. Consider reparametrization.")
            # return None, None
        elif len(goal_candidates) == 1 and distEuclidean(goal_candidates[0].point, v_end.point) > distEuclidean(pose.point, v_end.point):
            goal_candidates[0] = v_end

        dists = [distEuclidean(gc.point, v_end.point) for gc in goal_candidates]
        # print("Goal candidates dists: ", dists)
        return goal_candidates[np.argmin(dists)], path_index

    # #} end of getLookaheadPoint()

    # #{ unwrapHeadingInPath()
    def unwrapHeadingInPath(self, path):
        '''
        Unwraps heading in the entire path.

        Parameters:
            path (list[Pose])

        Returns:
            path_unwrapped (list[Pose]): list of 3D poses with each pose having positive
        '''
        path_unwrapped = [path[0]]

        for k in range(1, len(path)):
            prev_heading = path_unwrapped[-1].heading
            if abs(prev_heading - path[k].heading) < np.pi:
                path_unwrapped.append(path[k])
            else:
                heading_diff = angleDiff(prev_heading, path[k].heading)
                pose         = Pose(path[k].point, prev_heading + heading_diff)
                path_unwrapped.append(pose)

        return path_unwrapped
    # #} end of unwrapHeadingInPath()

    # #{ getParametrizedTrajectory()
    def getParametrizedTrajectory(self, waypoints, v_lims, a_lims):
        '''
        Finds time parametrization throughout the given waypoints.

        Parameters:
            waypoints (list[Pose]): the waypoints to be parametrized
            v_lims (4x1 list of floats): velocity limits for (x, y, z, heading)
            a_lims (4x1 list of floats): acceleration limits for (x, y, z, heading)

        Returns:
            trajectory (TOPPRA instance): time parametrization of the given waypoints
        '''

        print('[PARAMETRIZING TRAJECTORY]')
        unwrapped_heading_path = self.unwrapHeadingInPath(waypoints)
        trajectory             = self.computeTimeParametrization(unwrapped_heading_path, v_lims, a_lims)

        return trajectory
    # #} end of

    ## | ----------------- Getters and conversions ---------------- |

    # # #{ getLength()
    def getLength(self, poses):
        '''
        Finds length of trajectory poses.

        Parameters:
            poses (list[Pose]): the poses list

        Returns:
            length (float): length of the trajectory
        '''
        length = 0.0
        for i in range(1, len(poses)):
            length += distEuclidean(poses[i - 1].point, poses[i].point)
        return length
    # # #}

    # # #{ posesToTrajectory()
    def posesToTrajectory(self, poses):
        '''
        Converts poses to Trajectory.

        Parameters:
            poses (list): the poses list

        Returns:
            trajectory (Trajectory): 1-segment trajectory
        '''

        # convert to Trajectory class (hack: with 1 segment only)
        trajectory = Trajectory(self.dT, [poses[0], poses[-1]])
        trajectory.setSegment(0, poses)

        return trajectory
    # # #}

# # #}

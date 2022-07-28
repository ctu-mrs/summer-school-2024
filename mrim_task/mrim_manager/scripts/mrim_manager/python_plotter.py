import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d
import math
from mrim_manager.utils import *

class PythonPlotter:
    def __init__(self):
        # load problem
        pass

    # #{ plotDynamics()

    def plotDynamics(self, trajectories, constraints, dt):

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

            vels_z = [vel.z for vel in trajectories[k].velocities]
            accs_z = [acc.z for acc in trajectories[k].accelerations]
            jerks_z = [jerk.z for jerk in trajectories[k].jerks]
            snaps_z = [snap.z for snap in trajectories[k].snaps]
            vels_heading = [vel.heading for vel in trajectories[k].velocities]
            accs_heading = [acc.heading for acc in trajectories[k].accelerations]
            jerks_heading = [jerk.heading for jerk in trajectories[k].jerks]
            snaps_heading = [snap.heading for snap in trajectories[k].snaps]

            max_time = len(trajectories[k].poses) * dt

            fig, axs = plt.subplots(3, 4, sharex=False, sharey=False)
            fig.suptitle("Dynamics: " + trajectories[k].trajectory_name)
            axs[0, 0].grid(True, linestyle='-.')
            axs[0, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_xy, 'b')
            axs[0, 0].plot([0.0, max_time], [constraints.horizontal.speed, constraints.horizontal.speed], 'r')
            axs[0, 0].set_title("Speed horizontal (m/s)")

            axs[0, 1].grid(True, linestyle='-.')
            axs[0, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_xy, 'b')
            axs[0, 1].plot([0.0, max_time], [constraints.horizontal.acceleration, constraints.horizontal.acceleration], 'r')
            axs[0, 1].set_title("Acceleration horizontal (m/s^2)")

            axs[0, 2].grid(True, linestyle='-.')
            axs[0, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_xy, 'b')
            axs[0, 2].plot([0.0, max_time], [constraints.horizontal.jerk, constraints.horizontal.jerk], 'r')
            axs[0, 2].set_title("Jerk horizontal (m/s^3)")

            axs[0, 3].grid(True, linestyle='-.')
            axs[0, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_xy, 'b')
            axs[0, 3].plot([0.0, max_time], [constraints.horizontal.snap, constraints.horizontal.snap], 'r')
            axs[0, 3].set_title("Snap horizontal (m/s^4)")

            axs[1, 0].grid(True, linestyle='-.')
            axs[1, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_z, 'b')
            axs[1, 0].plot([0.0, max_time], [constraints.ascending.speed, constraints.ascending.speed], 'r')
            axs[1, 0].plot([0.0, max_time], [-constraints.descending.speed, -constraints.descending.speed], 'r')
            axs[1, 0].set_title("Speed vertical (m/s)")

            axs[1, 1].grid(True, linestyle='-.')
            axs[1, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_z, 'b')
            axs[1, 1].plot([0.0, max_time], [constraints.ascending.acceleration, constraints.ascending.acceleration], 'r')
            axs[1, 1].plot([0.0, max_time], [-constraints.descending.acceleration, -constraints.descending.acceleration], 'r')
            axs[1, 1].set_title("Acceleration vertical (m/s^2)")

            axs[1, 2].grid(True, linestyle='-.')
            axs[1, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_z, 'b')
            axs[1, 2].plot([0.0, max_time], [constraints.ascending.jerk, constraints.ascending.jerk], 'r')
            axs[1, 2].plot([0.0, max_time], [-constraints.descending.jerk, -constraints.descending.jerk], 'r')
            axs[1, 2].set_title("Jerk vertical (m/s^3)")

            axs[1, 3].grid(True, linestyle='-.')
            axs[1, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_z, 'b')
            axs[1, 3].plot([0.0, max_time], [constraints.ascending.snap, constraints.ascending.snap], 'r')
            axs[1, 3].plot([0.0, max_time], [-constraints.descending.snap, -constraints.descending.snap], 'r')
            axs[1, 3].set_title("Snap vertical (m/s^4)")

            axs[2, 0].grid(True, linestyle='-.')
            axs[2, 0].plot(np.arange(0, len(trajectories[k].poses))*dt, vels_heading, 'b')
            axs[2, 0].plot([0.0, max_time], [constraints.heading.speed, constraints.heading.speed], 'r')
            axs[2, 0].plot([0.0, max_time], [-constraints.heading.speed, -constraints.heading.speed], 'r')
            axs[2, 0].set_title("Speed heading (rad/s)")

            axs[2, 1].grid(True, linestyle='-.')
            axs[2, 1].plot(np.arange(0, len(trajectories[k].poses))*dt, accs_heading, 'b')
            axs[2, 1].plot([0.0, max_time], [constraints.heading.acceleration, constraints.heading.acceleration], 'r')
            axs[2, 1].plot([0.0, max_time], [-constraints.heading.acceleration, -constraints.heading.acceleration], 'r')
            axs[2, 1].set_title("Acceleration heading (rad/s^2)")

            axs[2, 2].grid(True, linestyle='-.')
            axs[2, 2].plot(np.arange(0, len(trajectories[k].poses))*dt, jerks_heading, 'b')
            axs[2, 2].plot([0.0, max_time], [constraints.heading.jerk, constraints.heading.jerk], 'r')
            axs[2, 2].plot([0.0, max_time], [-constraints.heading.jerk, -constraints.heading.jerk], 'r')
            axs[2, 2].set_title("Jerk heading (rad/s^3)")

            axs[2, 3].grid(True, linestyle='-.')
            axs[2, 3].plot(np.arange(0, len(trajectories[k].poses))*dt, snaps_heading, 'b')
            axs[2, 3].plot([0.0, max_time], [constraints.heading.snap, constraints.heading.snap], 'r')
            axs[2, 3].plot([0.0, max_time], [-constraints.heading.snap, -constraints.heading.snap], 'r')
            axs[2, 3].set_title("Snap heading (rad/s^4)")

            mng = plt.get_current_fig_manager()
            mng.resize(*mng.window.maxsize())
            plt.show()

    # #} end of plotDynamics()

    # #{ plotPaths()
    def plotPaths(self, trajectories):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_title("Trajectories")
        for trajectory in trajectories:
            xs = [pose.x for pose in trajectory.poses]
            ys = [pose.y for pose in trajectory.poses]
            zs = [pose.z for pose in trajectory.poses]
            ax.plot(xs, ys, zs)
            idx_middle = math.floor(len(xs)*0.05)
            ax.scatter([xs[0], xs[idx_middle]], [ys[0], ys[idx_middle]], [zs[0], zs[idx_middle]], marker='o')
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_zlabel("z (m)")

        set_axes_equal(ax)
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotPaths()

    # #{ plotMutualDistances()

    def plotMutualDistances(self, trajectories, min_dists_list, dt, minimum_mutual_distance):


        if len(min_dists_list) < 2:
            rospy.loginfo('[MrimManager] Single trajectory present. Mutual distances cannot be plotted.')
            return
        elif len(min_dists_list) == 2:
            n_rows = 1
            n_cols = 2
        elif len(min_dists_list) == 4:
            n_rows = 2
            n_cols = 2
        elif len(min_dists_list) <= 9:
            n_rows = math.ceil(len(min_dists_list) / 3.0)
            n_cols = 3
        else:
            n_rows = math.ceil(len(min_dists_list) / 4.0)
            n_cols = 4

        fig, axs = plt.subplots(n_rows, n_cols, sharex=False, sharey=False)
        fig.suptitle("Minimum mutual distance (m)")

        for k in range(len(min_dists_list)):
            row_idx = math.floor(k / n_cols)
            col_idx = k % n_cols
            max_time = len(min_dists_list[k])*dt
            if n_rows == 1:
                axs[col_idx].grid(True, linestyle='-.')
                axs[col_idx].plot(np.arange(0, len(min_dists_list[k]))*dt, [p[0] for p in min_dists_list[k]], 'b')
                axs[col_idx].plot([0.0, max_time], [minimum_mutual_distance, minimum_mutual_distance], 'r')
                axs[col_idx].set_title("Trajectory: " + trajectories[k].trajectory_name)
            else:
                axs[row_idx, col_idx].grid(True, linestyle='-.')
                axs[row_idx, col_idx].plot(np.arange(0, len(min_dists_list[k]))*dt, [p[0] for p in min_dists_list[k]], 'b')
                axs[row_idx, col_idx].plot([0.0, max_time], [minimum_mutual_distance, minimum_mutual_distance], 'r')
                axs[row_idx, col_idx].set_title("Trajectory: " + trajectories[k].trajectory_name)

        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotMutualDistances()

    # #{ plotUavObstacleDistances()

    def plotUavObstacleDistances(self, trajectories, dists_list, dt, minimum_obstacle_distance):


        if len(dists_list) <= 2:
            n_rows = 1
            n_cols = len(dists_list)
        elif len(dists_list) == 4:
            n_rows = 2
            n_cols = 2
        elif len(dists_list) <= 9:
            n_rows = math.ceil(len(dists_list) / 3.0)
            n_cols = 3
        else:
            n_rows = math.ceil(len(dists_list) / 4.0)
            n_cols = 4

        fig, axs = plt.subplots(n_rows, n_cols, sharex=False, sharey=False)
        fig.suptitle("Minimum UAV-obstacle distance (m)")

        for k in range(len(dists_list)):
            row_idx = math.floor(k / n_cols)
            col_idx = k % n_cols
            max_time = len(dists_list[k])*dt
            if n_rows == 1:
                axs[col_idx].grid(True, linestyle='-.')
                axs[col_idx].plot(np.arange(0, len(dists_list[k]))*dt, [p[0] for p in dists_list[k]], 'b')
                axs[col_idx].plot([0.0, max_time], [minimum_obstacle_distance, minimum_obstacle_distance], 'r')
                axs[col_idx].set_title("Trajectory: " + trajectories[k].trajectory_name)
            else:
                axs[row_idx, col_idx].grid(True, linestyle='-.')
                axs[row_idx, col_idx].plot(np.arange(0, len(dists_list[k]))*dt, [p[0] for p in dists_list[k]], 'b')
                axs[row_idx, col_idx].plot([0.0, max_time], [minimum_obstacle_distance, minimum_obstacle_distance], 'r')
                axs[row_idx, col_idx].set_title("Trajectory: " + trajectories[k].trajectory_name)

        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotMutualDistances()

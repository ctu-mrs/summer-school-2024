"""
Various types of TSP utilizing local planners for distance estimation and path planning
@author: P. Petracek & V. Kratky & P.Vana & P.Cizek & R.Penicka
"""

import numpy as np

from random import randint

from sklearn.cluster import KMeans
from scipy.spatial.kdtree import KDTree

from utils import *
from path_planners.grid_based.grid_3d import Grid3D
from path_planners.grid_based.astar   import AStar
from path_planners.sampling_based.rrt import RRT

from solvers.LKHInvoker import LKHInvoker

class TSPSolver3D():

    ALLOWED_PATH_PLANNERS               = ('euclidean', 'astar', 'rrt', 'rrtstar')
    ALLOWED_DISTANCE_ESTIMATION_METHODS = ('euclidean', 'astar', 'rrt', 'rrtstar')
    GRID_PLANNERS                       = ('astar')

    def __init__(self):
        self.lkh = LKHInvoker()

    # # #{ setup()
    def setup(self, problem, path_planner, viewpoints):
        """setup objects required in path planning methods"""

        if path_planner is None:
            return

        assert path_planner['path_planning_method'] in self.ALLOWED_PATH_PLANNERS, 'Given method to compute path (%s) is not allowed. Allowed methods: %s' % (path_planner, self.ALLOWED_PATH_PLANNERS)
        assert path_planner['distance_estimation_method'] in self.ALLOWED_DISTANCE_ESTIMATION_METHODS, 'Given method for distance estimation (%s) is not allowed. Allowed methods: %s' % (path_planner, self.ALLOWED_DISTANCE_ESTIMATION_METHODS)

        # Setup environment
        if path_planner['path_planning_method'] != 'euclidean' or path_planner['distance_estimation_method'] != 'euclidean':

            # setup KD tree for collision queries
            obstacles_array = np.array([[opt.x, opt.y, opt.z] for opt in problem.obstacle_points])
            path_planner['obstacles_kdtree'] = KDTree(obstacles_array)

            # setup environment bounds
            xs = [p.x for p in problem.safety_area]
            ys = [p.y for p in problem.safety_area]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            path_planner['bounds'] = Bounds(Point(x_min, y_min, problem.min_height), Point(x_max, y_max, problem.max_height))

        # Setup 3D grid for grid-based planners
        if path_planner['path_planning_method'] in self.GRID_PLANNERS or path_planner['distance_estimation_method'] in self.GRID_PLANNERS:

            # construct grid
            x_list = [opt.x for opt in problem.obstacle_points]
            x_list.extend([vp.pose.point.x for vp in viewpoints])
            y_list = [opt.y for opt in problem.obstacle_points]
            y_list.extend([vp.pose.point.y for vp in viewpoints])
            z_list = [opt.z for opt in problem.obstacle_points]
            z_list.extend([vp.pose.point.z for vp in viewpoints])

            min_x = np.min(x_list) - path_planner['safety_distance']
            max_x = np.max(x_list) + path_planner['safety_distance']
            min_y = np.min(y_list) - path_planner['safety_distance']
            max_y = np.max(y_list) + path_planner['safety_distance']
            min_z = problem.min_height
            max_z = problem.max_height

            dim_x = int(np.floor((max_x - min_x) / path_planner['astar/grid_resolution']))+1
            dim_y = int(np.floor((max_y - min_y) / path_planner['astar/grid_resolution']))+1
            dim_z = int(np.floor((max_z - min_z) / path_planner['astar/grid_resolution']))+1

            path_planner['grid'] = Grid3D(idx_zero = (min_x, min_y,min_z), dimensions=(dim_x,dim_y,dim_z), resolution_xyz=path_planner['astar/grid_resolution'])
            path_planner['grid'].setObstacles(problem.obstacle_points, path_planner['safety_distance'])

    # # #}

    # #{ plan_tour()

    def plan_tour(self, problem, viewpoints, path_planner=None):
        '''
        Solve TSP on viewpoints with given goals and starts

        Parameters:
            problem (InspectionProblem): task problem
            viewpoints (list[Viewpoint]): list of Viewpoint objects
            path_planner (dict): dictionary of parameters

        Returns:
            path (list): sequence of points with start equaling the end
        '''

        # Setup 3D grid for grid-based planners and KDtree for sampling-based planners
        self.setup(problem, path_planner, viewpoints)

        n              = len(viewpoints)
        self.distances = np.zeros((n, n))
        self.paths = {}

        # find path between each pair of goals (a, b)
        for a in range(n):
            for b in range(n):
                if a == b:
                    continue

                # [STUDENTS TODO]
                #   - Play with distance estimates in TSP (tsp/distance_estimates parameter in config) and see how it influences the solution
                #   - You will probably see that computing for all poses from both sets takes a long time.
                #   - Think if you can reduce the number of computations.

                # get poses of the viewpoints
                g1 = viewpoints[a].pose
                g2 = viewpoints[b].pose

                # estimate distances between the viewpoints
                path, distance = self.compute_path(g1, g2, path_planner, path_planner['distance_estimation_method'])

                # store paths/distances in matrices
                self.paths[(a, b)]   = path
                self.distances[a][b] = distance

        # compute TSP tour
        path = self.compute_tsp_tour(viewpoints, path_planner)

        return path

    # #}

    # # #{ compute_path()

    def compute_path(self, p_from, p_to, path_planner, path_planner_method):
        '''
        Computes collision-free path (if feasible) between two points

        Parameters:
            p_from (Pose): start
            p_to (Pose): to
            path_planner (dict): dictionary of parameters
            path_planner_method (string): method of path planning

        Returns:
            path (list[Pose]): sequence of points
            distance (float): length of path
        '''
        path, distance = [], float('inf')

        # Use Euclidean metric
        if path_planner is None or path_planner_method == 'euclidean':

            path, distance = [p_from, p_to], distEuclidean(p_from, p_to)

        # Plan with A*
        elif path_planner_method == 'astar':

            astar = AStar(path_planner['grid'], path_planner['safety_distance'], path_planner['timeout'], path_planner['straighten'])
            path, distance = astar.generatePath(p_from.asList(), p_to.asList())
            if path:
                path = [Pose(p[0], p[1], p[2], p[3]) for p in path]

        # Plan with RRT/RRT*
        elif path_planner_method.startswith('rrt'):

            rrt = RRT()
            path, distance = rrt.generatePath(p_from.asList(), p_to.asList(), path_planner, rrtstar=(path_planner_method == 'rrtstar'), straighten=path_planner['straighten'])
            if path:
                path = [Pose(p[0], p[1], p[2], p[3]) for p in path]

        if path is None or len(path) == 0:
            rospy.logerr('No path found. Shutting down.')
            rospy.signal_shutdown('No path found. Shutting down.');
            exit(-2)

        return path, distance

    # # #}

    # #{ compute_tsp_tour()

    def compute_tsp_tour(self, viewpoints, path_planner):
        '''
        Compute the shortest tour based on the distance matrix (self.distances) and connect the path throught waypoints

        Parameters:
            viewpoints (list[Viewpoint]): list of VPs
            path_planner (dict): dictionary of parameters

        Returns:
            path (list[Poses]): sequence of points with start equaling the end
        '''

        # compute the shortest sequence given the distance matrix
        sequence = self.compute_tsp_sequence()

        path = []
        n    = len(self.distances)

        for a in range(n):
            b = (a + 1) % n
            a_idx       = sequence[a]
            b_idx       = sequence[b]

            # if the paths are already computed
            if path_planner['distance_estimation_method'] == path_planner['path_planning_method']:
                actual_path = self.paths[(a_idx, b_idx)]
            # if the path planning and distance estimation methods differ, we need to compute the path
            else:
                actual_path, _ = self.compute_path(viewpoints[a_idx].pose, viewpoints[b_idx].pose, path_planner, path_planner['path_planning_method'])

            # join paths
            path = path + actual_path[:-1]

            # force flight to end point
            if a == (n - 1):
                path = path + [viewpoints[b_idx].pose]

        return path

    # #}

    # # #{ compute_tsp_sequence()

    def compute_tsp_sequence(self):
        '''
        Compute the shortest sequence based on the distance matrix (self.distances) using LKH

        Returns:
            sequence (list): sequence of viewpoints ordered optimally w.r.t the distance matrix
        '''

        n = len(self.distances)

        fname_tsp = "problem"
        user_comment = "a comment by the user"
        self.lkh.writeTSPLIBfile_FE(fname_tsp, self.distances, user_comment)
        self.lkh.run_LKHsolver_cmd(fname_tsp, silent=True)
        sequence = self.lkh.read_LKHresult_cmd(fname_tsp)

        if len(sequence) > 0 and sequence[0] is not None:
            for i in range(len(sequence)):
                if sequence[i] is None:
                    new_sequence = sequence[i:len(sequence)] + sequence[:i]
                    sequence = new_sequence
                    break

        return sequence

    # # #}

    # #{ clusterViewpoints()

    def clusterViewpoints(self, problem, viewpoints, method):
        '''
        Clusters viewpoints into K (number of robots) clusters.

        Parameters:
            problem (InspectionProblem): task problem
            viewpoints (list): list of Viewpoint objects
            method (string): method ('random', 'kmeans')

        Returns:
            clusters (Kx list): clusters of points indexed for each robot:
        '''
        k = problem.number_of_robots

        ## | ------------------- K-Means clustering ------------------- |
        if method == 'kmeans':
            # Prepare positions of the viewpoints in the world
            positions = np.array([vp.pose.point.asList() for vp in viewpoints])

            raise NotImplementedError('[STUDENTS TODO] KMeans clustering of viewpoints not implemented. You have to finish it on your own')
            # Tips:
            #  - utilize sklearn.cluster.KMeans implementation (https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html)
            #  - after finding the labels, you may want to swap the classes (e.g., by looking at the distance of the UAVs from the cluster centers)
            #  - Find the start poses of the UAVs in problem.start_poses[r].position.{x,y,z}

            # TODO: fill 1D list 'labels' of size len(viewpoints) with indices of the robots
            labels = [randint(0, k - 1) for vp in viewpoints]

        ## | -------------------- Random clustering ------------------- |
        else:
            labels = [randint(0, k - 1) for vp in viewpoints]

        # Store as clusters (2D array of viewpoints)
        clusters = []
        for r in range(k):
            clusters.append([])

            for label in range(len(labels)):
                if labels[label] == r:
                    clusters[r].append(viewpoints[label])

        return clusters

    # #}

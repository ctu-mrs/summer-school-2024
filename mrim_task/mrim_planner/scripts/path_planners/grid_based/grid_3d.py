"""
Highly simplistic 3D grid for 3D path planners
@author: F. Nekovar
"""

import numpy as np
from scipy import ndimage

# # #{ class Grid3D
class Grid3D():

    occupied_cells = set()

    def __init__(self, idx_zero, dimensions, resolution_xyz, resolution_hdg=None):

        self.resolution_xyz = resolution_xyz
        self.dim            = dimensions
        self.idx_zero       = idx_zero
        self.array          = np.zeros(dimensions)

    # # #{ idxIsOccupied()
    def idxIsOccupied(self, idx):
        try:
            return bool(self.array[idx])
        except IndexError:
            return False
    # # #}

    # # #{ xyzIsOccupied()
    def xyzIsOccupied(self, xyz):
        idx = self.metricToIndex(xyz)
        return self.idxIsOccupied(tuple(map(sum, zip(idx, self.idx_zero))))
    # # #}

    # # #{ obstacleBetween()
    def obstacleBetween(self, pt1, pt2):

        (x1, y1, z1) = pt1
        (x2, y2, z2) = pt2
        ListOfPoints = []
        ListOfPoints.append(pt1)
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        if (x2 > x1):
            xs = 1
        else:
            xs = -1
        if (y2 > y1):
            ys = 1
        else:
            ys = -1
        if (z2 > z1):
            zs = 1
        else:
            zs = -1
      
        # Driving axis is X-axis"
        if (dx >= dy and dx >= dz):        
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while (x1 != x2):
                x1 += xs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dx
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                ListOfPoints.append((x1, y1, z1))
      
        # Driving axis is Y-axis"
        elif (dy >= dx and dy >= dz):       
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while (y1 != y2):
                y1 += ys
                if (p1 >= 0):
                    x1 += xs
                    p1 -= 2 * dy
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                ListOfPoints.append((x1, y1, z1))
      
        # Driving axis is Z-axis"
        else:        
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while (z1 != z2):
                z1 += zs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dz
                if (p2 >= 0):
                    x1 += xs
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
                ListOfPoints.append((x1, y1, z1))

        for point in ListOfPoints:
            if self.idxIsOccupied(point):
                return True
        return False
    # # #}

    # # #{ setObstacles()
    def setObstacles(self, obstacle_points, safety_distance):
        for opt in obstacle_points:
            idx = self.metricToIndex([opt.x, opt.y, opt.z])
            self.array[idx] = 1

        safety_steps = np.ceil(safety_distance / self.resolution_xyz).astype(np.int)
        x,y,z        = np.ogrid[-safety_steps:safety_steps+1, -safety_steps:safety_steps+1 , -safety_steps:safety_steps+1]
        mask         = x**2 + y**2 + z**2 <= safety_steps**2
        self.array   = ndimage.binary_dilation(self.array, mask)
    # # #}

    # # #{ metricToIndex()
    def metricToIndex(self, xyz):
        x, y, z = xyz[0], xyz[1], xyz[2]
        ix = np.round((x - self.idx_zero[0]) / self.resolution_xyz).astype(np.int)
        iy = np.round((y - self.idx_zero[1]) / self.resolution_xyz).astype(np.int)
        iz = np.round((z - self.idx_zero[2]) / self.resolution_xyz).astype(np.int)
        iz = 0 if iz < 0 else iz
        return (ix, iy, iz)
    # # #}

    # # #{ indexToMetric()
    def indexToMetric(self, idx):
        ix, iy, iz = idx[0], idx[1], idx[2]
        return (self.idx_zero[0] + ix * self.resolution_xyz,
                self.idx_zero[1] + iy * self.resolution_xyz,
                self.idx_zero[2] + iz * self.resolution_xyz,
                None)
    # # #}

# # #}

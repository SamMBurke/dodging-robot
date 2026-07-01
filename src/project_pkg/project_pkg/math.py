import math
import numpy as np
import open3d as o3d


def convert_to_cartesian(msg, to_3D):
    '''
    Converts 2D LiDAR polar coordinates to Cartesian coordinates.
    If to_3D is True, converts to 3D Cartesian coordinates by adding a z-coordinate of 0
    '''
    cartesian_points = []

    for i, distance in enumerate(msg.ranges):
        if distance < msg.range_min or distance > msg.range_max:
            continue # skip invalid distance measurements

        angle = msg.angle_min + i * msg.angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        cartesian_points.append((x, y))

    if to_3D:
        # convert 2D lidar points to 3D by adding a z-coordinate of 0
        cartesian_points_3d = np.array([[x, y, 0] for x, y in cartesian_points])
        return cartesian_points_3d

    return cartesian_points


def get_euclidean_clusters(cartesian_points_3d, search_radius=0.5):
    '''
    Perform euclidean clustering by first converting the point cloud data into a KD tree for efficient nearest neighbour searching, 
    then uses the kdtree.search_radius_vector_3d() function to find all points within a certain radius of each point in the point cloud
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cartesian_points_3d)

    kdtree = o3d.geometry.KDTreeFlann(pcd) # convert the 3D point cloud data into a KD tree using Open3D

    visited = set()
    clusters = []

    # DFS of the KD tree to find all points within a radius of search_radius meters
    for i in range(len(cartesian_points_3d)):
        if i in visited:
            continue

        cluster = []
        queue = [i]

        while queue:
            idx = queue.pop(0)
            if idx in visited:
                continue

            visited.add(idx)
            cluster.append(cartesian_points_3d[idx])

            # search for points within a radius of search_radius meters
            [_, neighbours, _] = kdtree.search_radius_vector_3d(pcd.points[idx], search_radius) 
            
            for neighbour in neighbours:
                if neighbour not in visited:
                    queue.append(neighbour)

        clusters.append(cluster)

    return clusters


def calculate_closeness_criterion(self, C1, C2, d0):
    c1_max = np.max(C1)
    c1_min = np.min(C1)
    c2_max = np.max(C2)
    c2_min = np.min(C2)

    v1 = np.array([c1_max - C1, C1 - c1_min])
    v2 = np.array([c2_max - C2, C2 - c2_min])

    D1 = np.argmin(v1) * np.linalg.norm(v1, ord=2)
    D2 = np.argmin(v2) * np.linalg.norm(v2, ord=2)

    B = 0

    for i in range(len(D1)):
        d = np.max(np.min(D1[i], D2[i]), d0)
        B = B + 1/d

    return B
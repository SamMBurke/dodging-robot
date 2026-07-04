import math
import numpy as np
import open3d as o3d
import time

from .objects import DetectedObject


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


def calculate_closeness_criterion(C1, C2, d0):
    '''
    Optimization function for the fit_rectangle function. Optimizes the rectangle 
    fitting based on the closeness of the cluster points to the fitted rectangle edges
    '''
    # c1_min and c1_max are the boundaries on axis e1_hat, the rectangle edge direction vector (same goes for e2_hat)
    c1_max = np.max(C1)
    c1_min = np.min(C1)
    c2_max = np.max(C2)
    c2_min = np.min(C2)

    D1 = np.minimum(c1_max - C1, C1 - c1_min)
    D2 = np.minimum(c2_max - C2, C2 - c2_min)

    d = np.maximum(np.minimum(D1, D2), d0)

    return np.sum(1.0 / d)


def fit_rectangle(cluster, d0, id):
    '''
    Implementation of the Search-Based Rectangle Fitting algorithm. For 2D LiDAR scans, points clusters
    are often seen as L-shapes. This function efficiently fits a rectangle to the given cluster of points.
    The points are already clustered via the Euclidean Clustering algorithm from the get_euclidean_clusters function
    '''
    # the inputs into the criterion functions are C1 and C2 which are the projections of all the range points on the two orthogonal edges determined by theta
    thetas = np.linspace(0, 90, 89, endpoint=False) * np.pi / 180
    q_max = 0
    optimal_theta = 0
    for theta in thetas:
        # rectangle edge direction vectors
        e1_hat = np.array([np.cos(theta), np.sin(theta)])
        e2_hat = np.array([-np.sin(theta), np.cos(theta)])

        # projections on to the edge
        C1 = np.dot(cluster, e1_hat)
        C2 = np.dot(cluster, e2_hat)

        q = calculate_closeness_criterion(C1, C2, d0)
        if q > q_max: 
            q_max = q
            optimal_theta = theta
    
    # with the optimal rectangle angle determined, we can now reconstruct the rectangle
    e1 = np.array([np.cos(optimal_theta), np.sin(optimal_theta)])
    e2 = np.array([-np.sin(optimal_theta), np.cos(optimal_theta)])

    C1 = cluster @ e1
    C2 = cluster @ e2

    c1_min = np.min(C1)
    c1_max = np.max(C1)
    c2_max = np.max(C2)
    c2_min = np.min(C2)

    e1 = np.array([np.cos(optimal_theta), np.sin(optimal_theta)])
    e2 = np.array([-np.sin(optimal_theta), np.cos(optimal_theta)])

    rectangle_corners = np.array([
        c1_min * e1 + c2_min * e2,
        c1_max * e1 + c2_min * e2,
        c1_max * e1 + c2_max * e2,
        c1_min * e1 + c2_max * e2
    ])

    rectangle_center = np.array([(rectangle_corners[0][0] + rectangle_corners[2][0])/2, (rectangle_corners[0][1] + rectangle_corners[2][1])/2])
    length = np.linalg.norm(np.array([rectangle_corners[0], rectangle_corners[1]]))
    width = np.linalg.norm(np.array([rectangle_corners[0], rectangle_corners[3]]))

    obj = DetectedObject(
        id = id,
        center = rectangle_center,
        heading = optimal_theta,
        length = length,
        width = width,
        corners = rectangle_corners
    )
    return obj

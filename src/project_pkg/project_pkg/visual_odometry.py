'''
This file will perform visual odometry to understand the object of motions

- It will intake LiDAR data and detect object motion
    - It will then send object motion data to the path_planmner.py file
'''

import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from project_pkg import math
import numpy as np


class LidarObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('lidar_object_tracker')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # msg.ranges is a list of LiDAR distrance measurements in meters
        self.get_logger().info(
            f"Received {len(msg.ranges)} measurements."
        )

        cartesian_points = math.convert_to_cartesian(msg)
        self.get_logger().info(
            f"Converted to Cartesian coordinates: {len(cartesian_points)} points."
        )

    def cluster_points(self, msg):
        '''
        Uses euclidean clustering to cluster LiDAR points into distinct objects. Converts point-cloud data (pcd) 
        into a KD tree for efficient nearest neighbour searching, then uses the kdtree.search_radius_vector_3d() 
        function to find all points within a certain radius of each point in the point cloud.
        '''
        cartesian_points_3d = math.convert_to_cartesian(msg, to_3D=True)
        clusters = math.get_euclidean_clusters(cartesian_points_3d, search_radius=0.5)
        
        return clusters
    
    def get_objects(self, clusters, d0):
        '''
        Fits rectangles to the clusters, optimizing for how tightly the rectangle fits around the given cluster,
        specifically, maximizing the points-to-edges closeness. Returns the line representations for the 4 edges 
        of the rectangle in cartesian space for each cluster. The line representations for the 4 edges are assumed 
        to be of the form ax + by = c. The variable d0 represents the minimum distance threshold which avoids divisions
        by 0 and ensures that points very close to an edge don't have too much influence in the criterion calculation.
        '''

        # use the L-fitting method for creating objects from the clusters and then user the rectangles from the fitting
        # and use a kalman filter to predict their future positions, then use the Hungarian algorithm to match the predicted
        # cluster rectangle positions to the actual cluster rectangle positions for object tracking
        # then from the tracked objects, determine the velocities with respect to the turtlebot
        # the actual algorithm itself is called search-based rectangle fitting

        # the inputs into the criterion functions are C1 and C2 which are the projections of all the range points on the two orthogonal edges determined by theta
        thetas = np.linspace(0, 90, 89, endpoint=False) * np.pi / 180
        for cluster in clusters:
            q_max = 0
            optimal_theta = 0
            for theta in thetas:
                # rectangle edge direction vectors
                e1_hat = np.array([np.cos(theta), np.sin(theta)])
                e2_hat = np.array([-np.sin(theta), np.cos(theta)])

                # projections on to the edge
                C1 = np.dot(cluster, e1_hat)
                C2 = np.dot(cluster, e2_hat)

                q = math.calculate_closeness_criterion(C1, C2, d0)
                if q > q_max: 
                    q_max = q
                    optimal_theta = theta
            
            # with the optimal rectangle angle determined, we can now construct the lines of the rectangle
            C1 = np.dot(cluster, np.array([np.cos(optimal_theta), np.sin(optimal_theta)]))
            C2 = np.dot(cluster, np.array([np.cos(optimal_theta), np.sin(optimal_theta)]))

            a1 = np.cos(optimal_theta)
            b1 = np.sin(optimal_theta)
            c1 = np.min(C1)

            a2 = -np.sin(optimal_theta)
            b2 = np.cos(optimal_theta)
            c2 = np.min(C2)

            a3 = np.cos(optimal_theta)
            b3 = np.sin(optimal_theta)
            c3 = np.max(C1)

            a4 = -np.sin(optimal_theta)
            b4 = np.cos(optimal_theta)
            c4 = np.max(C1)

        return


def main(args=None):
    rclpy.init(args=args)

    node = LidarObjectTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
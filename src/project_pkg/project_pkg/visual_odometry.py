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
        Uses euclidean clustering to cluster LiDAR points into distinct objects.
        Converts point-cloud data (pcd) into a KD tree for efficient nearest neighbour searching, then uses the 
        kdtree.search_radius_vector_3d() function to find all points within a certain radius of each point in the point cloud
        '''
        cartesian_points_3d = math.convert_to_cartesian(msg, to_3D=True)
        clusters = math.get_euclidean_clusters(cartesian_points_3d, search_radius=0.5)
        
        return clusters
    
    def get_objects(self, clusters):
        '''
        
        '''

        # use the L-fitting method for creating objects from the clusters and then user the rectangles from the fitting
        # and use a kalman filter to predict their future positions, then use the Hungarian algorithm to match the predicted
        # cluster rectangle positions to the actual cluster rectangle positions for object tracking
        # then from the tracked objects, determine the velocities with respect to the turtlebot
        # the actual algorithm itself is called search-based rectangle fitting
        for cluster in clusters:
            continue

        return


def main(args=None):
    rclpy.init(args=args)

    node = LidarObjectTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
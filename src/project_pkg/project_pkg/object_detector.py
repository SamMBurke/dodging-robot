import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np

from project_pkg.math import convert_to_cartesian, get_euclidean_clusters, fit_rectangle
from project_pkg.conversions import corners_to_ros
from project_interfaces.msg import DetectedROSObject, DetectedROSObjectArray


class LidarObjectDetectorNode(Node):
    '''
    This node subscribes to the LiDAR sensor, clusters the points clouds into objects and then publishes those objects
    '''
    def __init__(self):
        super().__init__('lidar_object_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            DetectedROSObjectArray,
            '/detected_objects',
            10
        )

    def scan_callback(self, msg):
        self.get_logger().info(f'Received scan with {len(msg.ranges)} points')
        self._clusters = self.cluster_points(msg) # cluster the data points
        self.get_logger().info(f'Found {len(self._clusters)} clusters')

        self.objects = self.get_objects(self._clusters) # fit rectangles onto those clusters to get objects
        self.get_logger().info(f'Fitted {len(self.objects)} objects')
        
        ros_msg = self.convert_to_ros(self.objects, msg.header) # convert those objects into a ROS2 message of detected objects
        self.publisher.publish(ros_msg) # publish the ROS2 message

    def cluster_points(self, msg):
        '''
        Uses euclidean clustering to cluster LiDAR points into distinct objects. Converts point-cloud data (pcd) 
        into a KD tree for efficient nearest neighbour searching, then uses the kdtree.search_radius_vector_3d() 
        function to find all points within a certain radius of each point in the point cloud.
        '''
        cartesian_points_3d = convert_to_cartesian(msg, to_3D=True)
        clusters = get_euclidean_clusters(cartesian_points_3d, search_radius=0.5)
        
        return clusters
    
    def get_objects(self, clusters):
        '''
        Performs search-based rectangle fitting. Fits rectangles to the clusters, optimizing for how tightly the rectangle 
        fits around the given cluster, specifically, maximizing the points-to-edges closeness. Returns the line 
        representations for the 4 edges of the rectangle in cartesian space for each cluster. The line representations for 
        the 4 edges are assumed to be of the form ax + by = c. The variable d0 (in meters?) represents the minimum distance threshold which 
        avoids divisions by 0 and ensures that points very close to an edge don't have too much influence in the criterion calculation.
        '''
        objects = []
        for i, cluster in enumerate(clusters):
            cluster_2d = np.asarray(cluster)[:, :2]  # convert 3D cluster to 2D by removing the z-coordinate
            objects.append(fit_rectangle(cluster_2d, d0=0.1, id=i))
        return objects
    
    def convert_to_ros(self, objects, header):
        msg = DetectedROSObjectArray()
        msg.header = header
        
        for obj in objects:
            ros_obj = DetectedROSObject()

            ros_obj.id = obj.id
            ros_obj.center.x = float(obj.center[0])
            ros_obj.center.y = float(obj.center[1])
            ros_obj.heading = float(obj.heading)
            ros_obj.length = float(obj.length)
            ros_obj.width = float(obj.width)
            ros_obj.corners = corners_to_ros(obj.corners)

            msg.objects.append(ros_obj)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = LidarObjectDetectorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
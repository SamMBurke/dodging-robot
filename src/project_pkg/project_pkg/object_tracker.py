import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Time

import numpy as np

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs # registers PointStamped transform support
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from project_pkg.conversions import corners_to_ros, corners_from_ros
from project_pkg.kalman_tracker import KalmanTracker
from project_pkg.objects import DetectedObject
from project_interfaces.msg import DetectedROSObjectArray, TrackedObject, TrackedObjectArray


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker')

        self.declare_parameter('fixed_frame', 'odom')
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            DetectedROSObjectArray,
            '/detected_objects',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray,
            '/tracked_objects',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tracker = KalmanTracker(max_age = 5, min_hits = 3, max_valid_distance = 1.5)
        self._last_stamp = None
        self._default_dt = 0.1

    def scan_callback(self, msg):
        dt = self._compute_dt(msg.header.stamp)

        # self.get_logger().info(f'Received message: {msg}')
        # self.get_logger().info(f'Computed dt: {dt}')
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,       # target frame: odom
                msg.header.frame_id,    # source frame: base_link
                msg.header.stamp,       # time of the transform
                timeout=rclpy.duration.Duration(seconds = 0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {msg.header.frame_id} -> {self.fixed_frame}: {ex}. Skipping this frame.')
            return

        detected_objects = self.convert_to_obj(msg, transform)
        # self.get_logger().info(f'Detected objects: {detected_objects}')
        tracks = self.tracker.step(detected_objects, dt)
        # self.get_logger().info(f'Made tracks: {tracks}')

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.fixed_frame

        ros_msg = self.convert_to_ros(tracks, header)
        self.publisher.publish(ros_msg)

    def _compute_dt(self, stamp):
        current_stamp = Time.from_msg(stamp)
        if self._last_stamp is None:
            dt = self._default_dt
        else:
            dt = (current_stamp - self._last_stamp).nanoseconds / 1e9
            if dt <= 0:
                dt = self._default_dt

        self._last_stamp = current_stamp
        return dt
    
    def convert_to_obj(self, msg, transform):
        '''
        Transforms each detection's corners into the fixed frame, then recomputes center/heading
        from those transformed corners (rather than transforming center/heading separately) so
        the three stay mutually consistent after an arbitrary rotation + translation.
        '''
        objects = []
        for object_msg in msg.objects:
            transformed_corners = []
            for corner in object_msg.corners:
                point = PointStamped()
                point.point.x = corner.x
                point.point.y = corner.y
                point.point.z = 0.0
                transformed = tf2_geometry_msgs.do_transform_point(point, transform) # converts the point from the LiDAR frame to the fixed (odom) frame
                transformed_corners.append([transformed.point.x, transformed.point.y])
            transformed_corners = np.array(transformed_corners)

            center = (transformed_corners[0] + transformed_corners[2]) / 2.0
            edge_vector = transformed_corners[1] - transformed_corners[0]
            heading = float(np.arctan2(edge_vector[1], edge_vector[0]))

            obj = DetectedObject(
                id = object_msg.id,
                center = center,
                heading = heading,
                length = object_msg.length,
                width = object_msg.width,
                corners = transformed_corners
            )
            objects.append(obj)

        return objects
    
    def convert_to_ros(self, tracks, header):
        msg = TrackedObjectArray()
        msg.header = header

        for track in tracks:
            ros_obj = TrackedObject()

            ros_obj.id = track.id
            ros_obj.center.x = float(track.center[0])
            ros_obj.center.y = float(track.center[1])
            ros_obj.velocity.x = float(track.velocity[0])
            ros_obj.velocity.y = float(track.velocity[1])
            ros_obj.heading = float(track.heading)
            ros_obj.length = float(track.length)
            ros_obj.width = float(track.width)
            ros_obj.corners = corners_to_ros(track.corners)
            ros_obj.age = track.age
            ros_obj.hits = track.hits

            msg.objects.append(ros_obj)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = ObjectTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()